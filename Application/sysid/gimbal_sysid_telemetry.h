#ifndef GIMBAL_SYSID_TELEMETRY_H
#define GIMBAL_SYSID_TELEMETRY_H

#include <stdint.h>

#define GIMBAL_SYSID_TELEMETRY_SOF1          0xA6u
#define GIMBAL_SYSID_TELEMETRY_SOF2          0x6Au
#define GIMBAL_SYSID_TELEMETRY_FRAME_LEN     100u
#define GIMBAL_SYSID_TELEMETRY_CRC_INPUT_LEN 98u
#define GIMBAL_SYSID_TELEMETRY_TX_PERIOD_MS  5u

uint8_t GimbalSysIdTelemetry_BuildFrame(uint8_t *frame, uint16_t frame_len);

#endif
