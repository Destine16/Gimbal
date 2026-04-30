#ifndef GIMBAL_SYSID_H
#define GIMBAL_SYSID_H

#include <stdint.h>

#include "robot_def.h"

typedef enum
{
    GIMBAL_SYSID_PHASE_IDLE = 0,
    GIMBAL_SYSID_PHASE_BASELINE,
    GIMBAL_SYSID_PHASE_PRBS,
    GIMBAL_SYSID_PHASE_RETURN,
    GIMBAL_SYSID_PHASE_DONE,
    GIMBAL_SYSID_PHASE_STEP,
    GIMBAL_SYSID_PHASE_SINE,
} GimbalSysIdPhase_e;

typedef struct
{
    uint8_t mode;
    uint8_t phase;
    uint16_t seq_index;
    uint32_t elapsed_ms;
    float yaw_offset_rad;
    float pitch_offset_rad;
    float target_offset_rad;
    float base_yaw_rad;
    float base_pitch_rad;
} GimbalSysIdDebug_s;

extern volatile GimbalSysIdDebug_s gimbal_sysid_debug;

void GimbalSysId_Init(void);
uint8_t GimbalSysId_Update(const Gimbal_Upload_Data_s *feedback, Gimbal_Ctrl_Cmd_s *cmd);

#endif
