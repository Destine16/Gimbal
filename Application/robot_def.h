#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdint.h>
#include "ins_types.h"

#ifndef GIMBAL_PARAM_WARNING
#define GIMBAL_PARAM_WARNING
#pragma message "check motor ids / sign / limits in Application/robot_def.h and Application/gimbal/gimbal_params.c before hardware bring-up"
#endif

// 电机硬件 ID
#define GIMBAL_YAW_MOTOR_ID        1u
#define GIMBAL_PITCH_MOTOR_ID      2u

// 反馈/输出方向修正,装配方向相反时改成 -1.0f
#define GIMBAL_YAW_FEED_SIGN       1.0f
#define GIMBAL_PITCH_FEED_SIGN     1.0f
#define GIMBAL_YAW_GYRO_SIGN       1.0f
#define GIMBAL_PITCH_GYRO_SIGN     1.0f
#define GIMBAL_YAW_CURRENT_SIGN    1.0f
#define GIMBAL_PITCH_CURRENT_SIGN  1.0f
#define GIMBAL_YAW_OUTPUT_SIGN     1.0f
#define GIMBAL_PITCH_OUTPUT_SIGN   1.0f

// pitch 轴软件限位,单位 deg
#define GIMBAL_PITCH_MIN_DEG      (-20.0f)
#define GIMBAL_PITCH_MAX_DEG      (20.0f)

// 在线监测和视觉通信周期参数,单位 ms
#define GIMBAL_IMU_OFFLINE_TIMEOUT_MS   20u
#define GIMBAL_MOTOR_OFFLINE_TIMEOUT_MS 50u
#define DAEMON_TASK_PERIOD_MS           10u
#define VISION_CMD_TIMEOUT_MS      100u
#define VISION_STATUS_TX_PERIOD_MS  20u

typedef enum
{
    ROBOT_STOP = 0, // 整机未准备好
    ROBOT_READY,    // 整机已进入正常工作态
} Robot_Status_e;

typedef enum
{
    APP_OFFLINE = 0, // 模块离线
    APP_ONLINE,      // 模块在线
} App_Status_e;

typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电机断力/不输出
    GIMBAL_IMU_MODE,       // 基于 IMU 反馈的角度闭环
} gimbal_mode_e;

typedef struct
{
    float yaw;                 // 目标 yaw 角,单位 deg
    float pitch;               // 目标 pitch 角,单位 deg
    gimbal_mode_e gimbal_mode; // 当前云台控制模式
} Gimbal_Ctrl_Cmd_s;

typedef struct
{
    attitude_t gimbal_imu_data;         // 当前 IMU 姿态数据
    uint16_t yaw_motor_single_round_angle; // yaw 电机单圈编码器值
    uint8_t imu_online;                 // IMU 在线状态
    uint8_t yaw_motor_online;           // yaw 电机在线状态
    uint8_t pitch_motor_online;         // pitch 电机在线状态
} Gimbal_Upload_Data_s;

#endif
