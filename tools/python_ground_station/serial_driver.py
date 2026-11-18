"""
Custom Serial Driver with dual-backend support:
1. PySerial (if installed)
2. Windows ctypes kernel32 API (fallback for zero-dependency execution on Windows)
"""

import sys
import time
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


def list_com_ports():
    """Returns a list of available serial port names (e.g. ['COM3', 'COM4'])."""
    ports = []
    if HAS_PYSERIAL:
        try:
            for p in serial.tools.list_ports.comports():
                ports.append(p.device)
            if ports:
                return sorted(ports)
        except Exception:
            pass

    if sys.platform == 'win32':
        try:
            k = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'HARDWARE\DEVICEMAP\SERIALCOMM')
            for i in range(winreg.QueryInfoKey(k)[1]):
                ports.append(winreg.EnumValue(k, i)[1])
            winreg.CloseKey(k)
        except Exception:
            pass

    if not ports:
        ports = [f"COM{i}" for i in range(1, 11)]

    # Remove duplicates while preserving order
    seen = set()
    return [x for x in ports if not (x in seen or seen.add(x))]


class SerialClient:
    """Unified Serial interface supporting either pyserial or Windows ctypes."""

    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.port = port
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
                    self._pyserial_obj = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                    self.is_open = True
                    return True
                except Exception as e:
                    # Fallback to win32 if pyserial fails or is absent
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
                dcb.fDtrControl = 0  # DTR_CONTROL_DISABLE
                dcb.fRtsControl = 0  # RTS_CONTROL_DISABLE

                if not ctypes.windll.kernel32.SetCommState(handle, ctypes.byref(dcb)):
                    ctypes.windll.kernel32.CloseHandle(handle)
                    raise RuntimeError("Failed to set COM state")

                # Tránh treo ESP32 ở chế độ Reset/Bootloader
                ctypes.windll.kernel32.EscapeCommFunction(handle, 6)  # CLRDTR
                ctypes.windll.kernel32.EscapeCommFunction(handle, 4)  # CLRRTS

                # Xóa sạch bộ đệm rác cũ
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
                return self._pyserial_obj.write(data)

            if self._win_handle is not None and sys.platform == 'win32':
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
            return 0

    def write_line(self, text: str):
        if not text.endswith("\n"):
            text += "\n"
        return self.write(text.encode("utf-8"))

    def read(self, max_bytes=256) -> bytes:
        with self._lock:
            if not self.is_open:
                return b""
            if self._pyserial_obj is not None:
                try:
                    if self._pyserial_obj.in_waiting > 0:
                        return self._pyserial_obj.read(min(self._pyserial_obj.in_waiting, max_bytes))
                    return self._pyserial_obj.read(1)
                except Exception:
                    return b""

            if self._win_handle is not None and sys.platform == 'win32':
                errors = wintypes.DWORD(0)
                comstat = COMSTAT()
                # Xóa cờ lỗi phần cứng (framing/parity/overrun) và lấy số byte thực tế trong hàng đợi
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
            return b""
