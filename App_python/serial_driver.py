"""
Custom Serial Driver with dual-backend support:
1. PySerial (preferred, fully optimized for ESP32 with zero DTR/RTS reset glitch)
2. Windows ctypes kernel32 API (fallback for zero-dependency execution on Windows)
"""

import sys
import time
import re
import threading

try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False

if sys.platform == 'win32':
    import winreg
    import ctypes
    from ctypes import wintypes

    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    FILE_ATTRIBUTE_NORMAL = 0x80
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

    class DCB(ctypes.Structure):
        _fields_ = [
            ('DCBlength', wintypes.DWORD),
            ('BaudRate', wintypes.DWORD),
            ('fBinary', wintypes.DWORD, 1),
            ('fParity', wintypes.DWORD, 1),
            ('fOutxCtsFlow', wintypes.DWORD, 1),
            ('fOutxDsrFlow', wintypes.DWORD, 1),
            ('fDtrControl', wintypes.DWORD, 2),
            ('fDsrSensitivity', wintypes.DWORD, 1),
            ('fTXContinueOnXoff', wintypes.DWORD, 1),
            ('fOutX', wintypes.DWORD, 1),
            ('fInX', wintypes.DWORD, 1),
            ('fErrorChar', wintypes.DWORD, 1),
            ('fNull', wintypes.DWORD, 1),
            ('fRtsControl', wintypes.DWORD, 2),
            ('fAbortOnError', wintypes.DWORD, 1),
            ('fDummy2', wintypes.DWORD, 17),
            ('wReserved', wintypes.WORD),
            ('XonLim', wintypes.WORD),
            ('XoffLim', wintypes.WORD),
            ('ByteSize', wintypes.BYTE),
            ('Parity', wintypes.BYTE),
            ('StopBits', wintypes.BYTE),
            ('XonChar', ctypes.c_char),
            ('XoffChar', ctypes.c_char),
            ('ErrorChar', ctypes.c_char),
            ('EofChar', ctypes.c_char),
            ('EvtChar', ctypes.c_char),
            ('wReserved1', wintypes.WORD),
        ]

    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ('ReadIntervalTimeout', wintypes.DWORD),
            ('ReadTotalTimeoutMultiplier', wintypes.DWORD),
            ('ReadTotalTimeoutConstant', wintypes.DWORD),
            ('WriteTotalTimeoutMultiplier', wintypes.DWORD),
            ('WriteTotalTimeoutConstant', wintypes.DWORD),
        ]

    class COMSTAT(ctypes.Structure):
        _fields_ = [
            ('fCtsHold', wintypes.DWORD, 1),
            ('fDsrHold', wintypes.DWORD, 1),
            ('fRlsdHold', wintypes.DWORD, 1),
            ('fXoffHold', wintypes.DWORD, 1),
            ('fXoffSent', wintypes.DWORD, 1),
            ('fEof', wintypes.DWORD, 1),
            ('fTxim', wintypes.DWORD, 1),
            ('fReserved', wintypes.DWORD, 25),
            ('cbInQue', wintypes.DWORD),
            ('cbOutQue', wintypes.DWORD),
        ]

    # Explicitly configure Win32 function signatures to prevent 64-bit handle truncation
    k32 = ctypes.windll.kernel32
    k32.CreateFileW.restype = wintypes.HANDLE
    k32.CreateFileW.argtypes = [
        wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
        ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE
    ]
    k32.CloseHandle.restype = wintypes.BOOL
    k32.CloseHandle.argtypes = [wintypes.HANDLE]
    k32.GetCommState.restype = wintypes.BOOL
    k32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(DCB)]
    k32.SetCommState.restype = wintypes.BOOL
    k32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(DCB)]
    k32.SetCommTimeouts.restype = wintypes.BOOL
    k32.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.POINTER(COMMTIMEOUTS)]
    k32.ClearCommError.restype = wintypes.BOOL
    k32.ClearCommError.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD), ctypes.POINTER(COMSTAT)]
    k32.ReadFile.restype = wintypes.BOOL
    k32.ReadFile.argtypes = [wintypes.HANDLE, ctypes.c_char_p, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
    k32.WriteFile.restype = wintypes.BOOL
    k32.WriteFile.argtypes = [wintypes.HANDLE, ctypes.c_char_p, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
    k32.EscapeCommFunction.restype = wintypes.BOOL
    k32.EscapeCommFunction.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    k32.PurgeComm.restype = wintypes.BOOL
    k32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    k32.SetupComm.restype = wintypes.BOOL
    k32.SetupComm.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD]


ESP_CHIP_KEYWORDS = [
    "CP210", "CH340", "CH9102", "FTDI", "USB-SERIAL",
    "UART", "ESP32", "ESP", "SILICON LABS", "WCH"
]


def get_port_details():
    """
    Returns a list of dicts with rich port details:
    [
        {"port": "COM4", "desc": "Silicon Labs CP210x USB to UART Bridge (COM4)", "label": "COM4 - CP210x USB to UART", "is_esp": True},
        ...
    ]
    Sorted so that likely ESP32 / USB-UART dongle ports appear at the very top!
    """
    results = []

    def _sort_key(item):
        m = re.search(r"(\d+)", item["port"])
        num = int(m.group(1)) if m else 999
        return (not item.get("is_esp", False), num, item["port"])

    if HAS_PYSERIAL:
        try:
            for p in serial.tools.list_ports.comports():
                port = p.device
                desc = p.description or port
                hwid = p.hwid or ""
                search_text = f"{desc} {hwid}".upper()
                is_esp = any(k in search_text for k in ESP_CHIP_KEYWORDS)

                # Clean label for user dropdown
                clean_desc = desc
                if clean_desc.endswith(f"({port})"):
                    clean_desc = clean_desc[:-len(f"({port})")].strip()

                label = f"{port} ({clean_desc})" if clean_desc != port else port
                results.append({
                    "port": port,
                    "desc": desc,
                    "label": label,
                    "is_esp": is_esp
                })
            if results:
                # Rank ESP32 ports at the top, then sort by port number
                results.sort(key=_sort_key)
                return results
        except Exception:
            pass

    if sys.platform == 'win32':
        try:
            k = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'HARDWARE\DEVICEMAP\SERIALCOMM')
            for i in range(winreg.QueryInfoKey(k)[1]):
                val_name, val_data, _ = winreg.EnumValue(k, i)
                port = val_data
                is_esp = any(k in val_name.upper() for k in ESP_CHIP_KEYWORDS)
                results.append({
                    "port": port,
                    "desc": val_name,
                    "label": f"{port} ({val_name})" if val_name else port,
                    "is_esp": is_esp
                })
            winreg.CloseKey(k)
        except Exception:
            pass

    if not results:
        for i in range(1, 11):
            results.append({
                "port": f"COM{i}",
                "desc": f"Cổng COM{i}",
                "label": f"COM{i}",
                "is_esp": False
            })

    results.sort(key=_sort_key)
    return results


def list_com_ports():
    """Returns a list of available serial port names (e.g. ['COM3', 'COM4'])."""
    details = get_port_details()
    seen = set()
    ports = []
    for d in details:
        p = d["port"]
        if p not in seen:
            seen.add(p)
            ports.append(p)
    return ports


def find_best_esp32_port():
    """Returns the port name of the top-matched ESP32/USB-UART port, or None."""
    details = get_port_details()
    for d in details:
        if d["is_esp"]:
            return d["port"]
    if details:
        return details[0]["port"]
    return None


class SerialClient:
    """Unified Serial interface supporting either pyserial or Windows ctypes with zero-latency read."""

    def __init__(self, port, baudrate=115200, timeout=0.1):
        # Allow passing full label like 'COM4 (CP210x)'
        if " " in port:
            self.port = port.split()[0].strip()
        else:
            self.port = port.strip()

        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = False
        self._pyserial_obj = None
        self._win_handle = None
        self._lock = threading.Lock()

    def open(self):
        with self._lock:
            if self.is_open:
                return True

            if HAS_PYSERIAL:
                try:
                    # CRITICAL FOR ESP32 DEVKIT:
                    # Construct serial.Serial object first, explicitly configure DTR=False, RTS=False
                    # BEFORE calling open(). This prevents Windows serial driver from toggling DTR/RTS
                    # which triggers the auto-download transistor circuit and traps ESP32 in reset/bootloader!
                    ser = serial.Serial()
                    ser.port = self.port
                    ser.baudrate = self.baudrate
                    ser.timeout = self.timeout
                    ser.write_timeout = 0.5
                    ser.dtr = False
                    ser.rts = False
                    ser.open()

                    # Re-confirm DTR/RTS are low
                    ser.dtr = False
                    ser.rts = False

                    # Enlarge internal OS buffers for smooth high-speed streaming
                    if hasattr(ser, 'set_buffer_size'):
                        try:
                            ser.set_buffer_size(rx_size=8192, tx_size=8192)
                        except Exception:
                            pass

                    # Purge any old bytes
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()

                    self._pyserial_obj = ser
                    self.is_open = True
                    return True
                except Exception as e:
                    # Fallback to win32 ctypes if pyserial fails
                    pass

            if sys.platform == 'win32':
                port_path = f"\\\\.\\{self.port}"
                handle = ctypes.windll.kernel32.CreateFileW(
                    port_path,
                    GENERIC_READ | GENERIC_WRITE,
                    0,
                    None,
                    OPEN_EXISTING,
                    0,
                    None
                )
                if handle == INVALID_HANDLE_VALUE or handle is None:
                    err = ctypes.windll.kernel32.GetLastError()
                    raise RuntimeError(f"Cannot open port {self.port} (Win32 Error: {err})")

                # SetupComm to expand internal queues
                ctypes.windll.kernel32.SetupComm(handle, 4096, 4096)

                # Configure instant non-blocking read timeouts
                timeouts = COMMTIMEOUTS()
                timeouts.ReadIntervalTimeout = 0xFFFFFFFF  # MAXDWORD: Non-blocking read immediately
                timeouts.ReadTotalTimeoutMultiplier = 0
                timeouts.ReadTotalTimeoutConstant = 0
                timeouts.WriteTotalTimeoutMultiplier = 0
                timeouts.WriteTotalTimeoutConstant = 500
                ctypes.windll.kernel32.SetCommTimeouts(handle, ctypes.byref(timeouts))

                dcb = DCB()
                dcb.DCBlength = ctypes.sizeof(DCB)
                if not ctypes.windll.kernel32.GetCommState(handle, ctypes.byref(dcb)):
                    ctypes.windll.kernel32.CloseHandle(handle)
                    raise RuntimeError("Failed to get COM state")

                dcb.BaudRate = self.baudrate
                dcb.ByteSize = 8
                dcb.Parity = 0  # NOPARITY
                dcb.StopBits = 0  # ONESTOPBIT
                dcb.fBinary = 1
                dcb.fParity = 0
                dcb.fDtrControl = 0  # DTR_CONTROL_DISABLE (Crucial for ESP32)
                dcb.fRtsControl = 0  # RTS_CONTROL_DISABLE (Crucial for ESP32)

                if not ctypes.windll.kernel32.SetCommState(handle, ctypes.byref(dcb)):
                    ctypes.windll.kernel32.CloseHandle(handle)
                    raise RuntimeError("Failed to set COM state")

                # Tránh treo ESP32 ở chế độ Reset/Bootloader
                ctypes.windll.kernel32.EscapeCommFunction(handle, 6)  # CLRDTR
                ctypes.windll.kernel32.EscapeCommFunction(handle, 4)  # CLRRTS

                # Xóa sạch bộ đệm rác cũ (PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR)
                ctypes.windll.kernel32.PurgeComm(handle, 0x000F)

                self._win_handle = handle
                self.is_open = True
                return True

            raise RuntimeError("No serial driver available for this platform")

    def close(self):
        with self._lock:
            if not self.is_open:
                return
            if self._pyserial_obj is not None:
                try:
                    self._pyserial_obj.close()
                except Exception:
                    pass
                self._pyserial_obj = None

            if self._win_handle is not None and sys.platform == 'win32':
                try:
                    ctypes.windll.kernel32.CloseHandle(self._win_handle)
                except Exception:
                    pass
                self._win_handle = None

            self.is_open = False

    def write(self, data: bytes):
        with self._lock:
            if not self.is_open:
                return 0
            if self._pyserial_obj is not None:
                try:
                    res = self._pyserial_obj.write(data)
                    self._pyserial_obj.flush()
                    return res
                except Exception:
                    self.is_open = False
                    return 0

            if self._win_handle is not None and sys.platform == 'win32':
                try:
                    bytes_written = wintypes.DWORD(0)
                    buf = ctypes.create_string_buffer(data)
                    ctypes.windll.kernel32.WriteFile(
                        self._win_handle,
                        buf,
                        len(data),
                        ctypes.byref(bytes_written),
                        None
                    )
                    return bytes_written.value
                except Exception:
                    self.is_open = False
                    return 0
            return 0

    def write_line(self, text: str):
        if not text.endswith("\n"):
            text += "\n"
        return self.write(text.encode("utf-8"))

    def read(self, max_bytes=256) -> bytes:
        """
        Instant non-blocking read: returns available bytes immediately.
        Never blocks for timeout if the buffer is empty.
        """
        with self._lock:
            if not self.is_open:
                return b""
            if self._pyserial_obj is not None:
                try:
                    n = self._pyserial_obj.in_waiting
                    if n > 0:
                        return self._pyserial_obj.read(min(n, max_bytes))
                    return b""
                except Exception:
                    self.is_open = False
                    return b""

            if self._win_handle is not None and sys.platform == 'win32':
                try:
                    errors = wintypes.DWORD(0)
                    comstat = COMSTAT()
                    ctypes.windll.kernel32.ClearCommError(
                        self._win_handle,
                        ctypes.byref(errors),
                        ctypes.byref(comstat)
                    )

                    if comstat.cbInQue == 0:
                        return b""

                    to_read = min(comstat.cbInQue, max_bytes)
                    buf = ctypes.create_string_buffer(to_read)
                    bytes_read = wintypes.DWORD(0)
                    res = ctypes.windll.kernel32.ReadFile(
                        self._win_handle,
                        buf,
                        to_read,
                        ctypes.byref(bytes_read),
                        None
                    )
                    if res and bytes_read.value > 0:
                        return buf.raw[:bytes_read.value]
                    return b""
                except Exception:
                    self.is_open = False
                    return b""
            return b""

    @property
    def in_waiting(self) -> int:
        with self._lock:
            if not self.is_open:
                return 0
            if self._pyserial_obj is not None:
                try:
                    return self._pyserial_obj.in_waiting
                except Exception:
                    return 0
            if self._win_handle is not None and sys.platform == 'win32':
                try:
                    errors = wintypes.DWORD(0)
                    comstat = COMSTAT()
                    ctypes.windll.kernel32.ClearCommError(
                        self._win_handle,
                        ctypes.byref(errors),
                        ctypes.byref(comstat)
                    )
                    return comstat.cbInQue
                except Exception:
                    return 0
            return 0
