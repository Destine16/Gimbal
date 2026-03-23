#ifndef VISION_COMM_H
#define VISION_COMM_H

#include <stdint.h>

typedef struct __attribute__((packed))
{
    int16_t yaw_0p01deg;
    int16_t pitch_0p01deg;
    int16_t yaw_speed_0p01rad;
    int16_t pitch_speed_0p01rad;
    uint16_t distance_mm;
    uint8_t track_state;
    uint8_t fire_cmd;
} VisionCmd_t;

typedef struct __attribute__((packed))
{
    uint8_t enemy_color;
    uint16_t bullet_speed_0p01mps;
    uint8_t vision_mode;
    uint8_t fire_permission;
} ControlStatus_t;

void VisionComm_Init(void);
void VisionComm_Task(void);
void VisionComm_RxBytes(const uint8_t *data, uint16_t len);
uint8_t VisionComm_GetVisionCmd(VisionCmd_t *cmd);
void VisionComm_UpdateControlStatus(const ControlStatus_t *status);
uint8_t VisionComm_IsOnline(void);

#endif
