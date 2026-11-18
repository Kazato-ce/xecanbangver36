#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PROTOCOL_MAGIC_BYTE         0xAA
#define PROTOCOL_VERSION            0x01

typedef enum {
    CMD_DISARM  = 0x00,
    CMD_ARM     = 0x01,
    CMD_ESTOP   = 0x02,
    CMD_DRIVE   = 0x03,
    CMD_PING    = 0x04
} ProtocolCmd_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic;         /* 0xAA */
    uint8_t  version;       /* 0x01 */
    uint16_t seq;           /* Sequence number */
    uint8_t  cmd;           /* ProtocolCmd_t */
    int16_t  drive_cmd;     /* Signed forward/backward speed command (-1000..1000) */
    int16_t  steer_cmd;     /* Signed steering/yaw rate command (-1000..1000) */
    uint8_t  flags;         /* Bit 0: emergency stop, Bit 1: arm requested */
    uint8_t  reserved[5];
    uint16_t checksum;      /* Simple 16-bit sum or CRC */
} RadioPacket_t;
#pragma pack(pop)

/* Checksum calculation helper */
static inline uint16_t protocol_calc_checksum(const uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

#endif /* PROTOCOL_H */
