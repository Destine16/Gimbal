#ifndef VISION_COMM_H
#define VISION_COMM_H

#include <stdint.h>

typedef struct __attribute__((packed))
{
    int16_t delta_yaw_1e4rad;
    int16_t delta_pitch_1e4rad;
} VisionCmd_t;

void VisionComm_Init(void);
void VisionComm_RxBytes(const uint8_t *data, uint16_t len);
uint8_t VisionComm_GetVisionCmd(VisionCmd_t *cmd);
uint8_t VisionComm_IsOnline(void);

#endif
