#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdint.h>
#include "ins_types.h"

#ifndef GIMBAL_PARAM_WARNING
#define GIMBAL_PARAM_WARNING
#pragma message "check motor ids / sign / limits in Application/robot_def.h and Application/gimbal/gimbal_params.c before hardware bring-up"
#endif

#define GIMBAL_YAW_MOTOR_ID        1u
#define GIMBAL_PITCH_MOTOR_ID      2u
#define GIMBAL_YAW_FEED_SIGN       1.0f
#define GIMBAL_PITCH_FEED_SIGN     1.0f
#define GIMBAL_YAW_GYRO_SIGN       1.0f
#define GIMBAL_PITCH_GYRO_SIGN     1.0f
#define GIMBAL_YAW_CURRENT_SIGN    1.0f
#define GIMBAL_PITCH_CURRENT_SIGN  1.0f
#define GIMBAL_YAW_OUTPUT_SIGN     1.0f
#define GIMBAL_PITCH_OUTPUT_SIGN   1.0f
#define GIMBAL_PITCH_MIN_DEG      (-20.0f)
#define GIMBAL_PITCH_MAX_DEG      (20.0f)
#define GIMBAL_IMU_OFFLINE_TIMEOUT_MS   20u
#define GIMBAL_MOTOR_OFFLINE_TIMEOUT_MS 50u
#define DAEMON_TASK_PERIOD_MS           10u
#define VISION_CMD_TIMEOUT_MS      100u
#define VISION_STATUS_TX_PERIOD_MS  20u

typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
} App_Status_e;

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_IMU_MODE,
} gimbal_mode_e;

typedef struct
{
    float yaw;
    float pitch;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
    uint8_t imu_online;
    uint8_t yaw_motor_online;
    uint8_t pitch_motor_online;
} Gimbal_Upload_Data_s;

#endif
