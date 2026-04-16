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

void VisionComm_Init(void);
void VisionComm_Task(void);
void VisionComm_RxBytes(const uint8_t *data, uint16_t len);
uint8_t VisionComm_GetVisionCmd(VisionCmd_t *cmd);
void VisionComm_UpdateStatus(const VisionStatus_t *status);
uint8_t VisionComm_IsOnline(void);
void VisionComm_SetUsbHostReady(uint8_t ready);

#endif
