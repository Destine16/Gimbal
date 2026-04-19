#ifndef VISION_COMM_H
#define VISION_COMM_H

#include <stdint.h>

typedef struct __attribute__((packed))
{
    int16_t delta_yaw_1e4rad;
    int16_t delta_pitch_1e4rad;
} VisionCmd_t;

typedef struct __attribute__((packed))
{
    int32_t yaw_actual_1e4rad;
    int32_t pitch_actual_1e4rad;
    int16_t last_rx_delta_yaw_1e4rad;
    int16_t last_rx_delta_pitch_1e4rad;
} VisionStatus_t;

typedef struct
{
    uint32_t usb_rx_packet_count;
    uint32_t usb_rx_byte_count;
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t last_rx_tick_ms;
    uint32_t last_valid_tick_ms;
    uint16_t last_calc_crc;
    uint16_t last_recv_crc;
    int16_t last_delta_yaw_1e4rad;
    int16_t last_delta_pitch_1e4rad;
    float last_delta_yaw_rad;
    float last_delta_pitch_rad;
    float last_delta_yaw_deg;
    float last_delta_pitch_deg;
    int32_t actual_yaw_1e4rad;
    int32_t actual_pitch_1e4rad;
    float actual_yaw_rad;
    float actual_pitch_rad;
    float actual_yaw_deg;
    float actual_pitch_deg;
    uint8_t last_usb_packet_len;
    uint8_t last_usb_packet_bytes[8];
    uint8_t last_candidate_frame[8];
    uint8_t last_valid_frame[8];
    uint8_t latest_cmd_valid;
    uint8_t latest_cmd_pending;
} VisionDebug_t;

extern volatile VisionDebug_t vision_debug;

void VisionComm_Init(void);
void VisionComm_Task(void);
void VisionComm_RxBytes(const uint8_t *data, uint16_t len);
uint8_t VisionComm_GetVisionCmd(VisionCmd_t *cmd);
void VisionComm_UpdateStatus(const VisionStatus_t *status);
uint8_t VisionComm_IsOnline(void);
void VisionComm_SetUsbHostReady(uint8_t ready);

#endif
