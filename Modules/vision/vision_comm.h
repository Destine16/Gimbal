#ifndef VISION_COMM_H
#define VISION_COMM_H

#include <stdint.h>

typedef struct
{
    uint8_t seq;
    uint8_t target_valid;
    int16_t delta_yaw_1e4rad;
    int16_t delta_pitch_1e4rad;
} VisionCmd_t;

typedef struct
{
    int32_t yaw_actual_1e4rad;
    int32_t pitch_actual_1e4rad;
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
    uint8_t last_seq;
    uint8_t seq_echo;
    uint8_t last_target_valid;
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
    float imu_yaw_gyro_raw_rad_s;
    float imu_yaw_gyro_bias_rad_s;
    float imu_yaw_gyro_corrected_rad_s;
    uint16_t yaw_encoder_raw;
    uint16_t pitch_encoder_raw;
    float yaw_encoder_single_round_rad;
    float pitch_encoder_single_round_rad;
    float yaw_encoder_total_angle_rad;
    float pitch_encoder_total_angle_rad;
    float yaw_encoder_speed_rad_s;
    float pitch_encoder_speed_rad_s;
    uint8_t last_usb_packet_len;
    uint8_t last_usb_packet_bytes[10];
    uint8_t last_candidate_frame[10];
    uint8_t last_valid_frame[10];
    uint8_t latest_cmd_valid;
    uint8_t latest_cmd_pending;
    uint32_t can_rx_total_count;
    uint32_t can_rx_matched_count;
    uint32_t can_rx_unmatched_count;
    uint32_t can_rx_0x206_count;
    uint32_t can_rx_0x208_count;
    uint16_t can_last_rx_std_id;
    uint16_t can_last_unmatched_rx_std_id;
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
