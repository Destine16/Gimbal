#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include <stdint.h>

typedef struct
{
    uint8_t robot_state;
    uint8_t gimbal_ready;
    uint8_t gimbal_mode;
    uint8_t vision_target_valid;
    uint8_t vision_cmd_ready;
    uint8_t vision_cmd_target_valid;
    uint8_t sentry_state;
    uint8_t stall_axis;
    uint8_t imu_online;
    uint8_t yaw_motor_online;
    uint8_t pitch_motor_online;
    uint8_t stall_detected;
    uint32_t last_target_valid_tick_ms;
    uint32_t stall_recovery_end_tick_ms;
    float vision_target_yaw_rad;
    float vision_target_pitch_rad;
    float scan_yaw_target_rad;
    float scan_pitch_target_rad;
    float recovery_yaw_target_rad;
    float recovery_pitch_target_rad;
    float cmd_yaw_rad;
    float cmd_pitch_rad;
} RobotCmdDebug_s;

extern volatile RobotCmdDebug_s robot_cmd_debug;

void RobotCMDInit(void);
void RobotCMDTask(void);

#endif
