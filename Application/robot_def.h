#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdint.h>
#include "ins_types.h"

#ifndef GIMBAL_PARAM_WARNING
#define GIMBAL_PARAM_WARNING
#pragma message "check motor ids / sign / limits in Application/robot_def.h and Application/gimbal/gimbal_params.c before hardware bring-up"
#endif

// 电机硬件 ID
#define GIMBAL_YAW_MOTOR_ID        2u
#define GIMBAL_PITCH_MOTOR_ID      4u

// 云台业务坐标约定:
// +yaw   = 从上往下看逆时针
// +pitch = 相机抬头
// BMI088 原始坐标到常见机器人右手坐标(+X 前, +Y 左, +Z 上)的映射在 BMI088_ToGimbalFrame() 中完成
#define GIMBAL_YAW_IMU_ANGLE_TO_AXIS_SIGN      1.0f
#define GIMBAL_PITCH_IMU_ANGLE_TO_AXIS_SIGN    (1.0f)
#define GIMBAL_YAW_IMU_GYRO_TO_AXIS_SIGN       1.0f
#define GIMBAL_PITCH_IMU_GYRO_TO_AXIS_SIGN     (1.0f)

// GM6020 官方电机方向适配:
// 这里仅处理“CAN 原始量”和“GM6020 官方正方向”的关系,通常保持 +1
#define GM6020_YAW_RAW_CURRENT_TO_OFFICIAL_SIGN      1.0f
#define GM6020_PITCH_RAW_CURRENT_TO_OFFICIAL_SIGN    1.0f
#define GM6020_YAW_OFFICIAL_OUTPUT_TO_CAN_SIGN       1.0f
#define GM6020_PITCH_OFFICIAL_OUTPUT_TO_CAN_SIGN     1.0f

// 云台业务轴到电机官方方向的适配:
// 当前机构中 +yaw 与 yaw GM6020 官方正方向一致;
// pitch GM6020 官方正方向为相机低头,而业务 +pitch 为相机抬头,因此 pitch 取 -1
#define GIMBAL_YAW_AXIS_TO_MOTOR_SIGN      1.0f
#define GIMBAL_PITCH_AXIS_TO_MOTOR_SIGN    (-1.0f)

// 控制器最终使用的符号。优先改上面三层定义,不要直接改这里。
#define GIMBAL_YAW_FEED_SIGN       GIMBAL_YAW_IMU_ANGLE_TO_AXIS_SIGN
#define GIMBAL_PITCH_FEED_SIGN     GIMBAL_PITCH_IMU_ANGLE_TO_AXIS_SIGN
#define GIMBAL_YAW_GYRO_SIGN       GIMBAL_YAW_IMU_GYRO_TO_AXIS_SIGN
#define GIMBAL_PITCH_GYRO_SIGN     GIMBAL_PITCH_IMU_GYRO_TO_AXIS_SIGN
#define GIMBAL_YAW_CURRENT_SIGN    (GIMBAL_YAW_AXIS_TO_MOTOR_SIGN * GM6020_YAW_RAW_CURRENT_TO_OFFICIAL_SIGN)
#define GIMBAL_PITCH_CURRENT_SIGN  (GIMBAL_PITCH_AXIS_TO_MOTOR_SIGN * GM6020_PITCH_RAW_CURRENT_TO_OFFICIAL_SIGN)
#define GIMBAL_YAW_OUTPUT_SIGN     (GIMBAL_YAW_AXIS_TO_MOTOR_SIGN * GM6020_YAW_OFFICIAL_OUTPUT_TO_CAN_SIGN)
#define GIMBAL_PITCH_OUTPUT_SIGN   (GIMBAL_PITCH_AXIS_TO_MOTOR_SIGN * GM6020_PITCH_OFFICIAL_OUTPUT_TO_CAN_SIGN)

// pitch 轴软件限位: 水平为 0 rad, 活动范围 +/-42 deg
#define GIMBAL_PITCH_MIN_RAD      (-0.73303829f)
#define GIMBAL_PITCH_MAX_RAD      (0.73303829f)

// 在线监测和视觉通信周期参数,单位 ms
#define GIMBAL_IMU_OFFLINE_TIMEOUT_MS   20u
#define GIMBAL_MOTOR_OFFLINE_TIMEOUT_MS 50u
#define DAEMON_TASK_PERIOD_MS           10u
#define VISION_CMD_TIMEOUT_MS      100u
#define VISION_STATUS_TX_PERIOD_MS  20u

// 视觉 delta 命令使用策略:
// CONTINUOUS: 视觉在线期间每周期使用 latest delta,适合视觉高频连续发送
// EVENT_TARGET: 每个新帧只消费一次 delta,转换成绝对目标后持续保持,适合视觉低频发送
#define VISION_CONTROL_CONTINUOUS    0u
#define VISION_CONTROL_EVENT_TARGET  1u
#ifndef VISION_CONTROL_MODE
#define VISION_CONTROL_MODE VISION_CONTROL_EVENT_TARGET
#endif
#if (VISION_CONTROL_MODE != VISION_CONTROL_CONTINUOUS) && (VISION_CONTROL_MODE != VISION_CONTROL_EVENT_TARGET)
#error "VISION_CONTROL_MODE must be VISION_CONTROL_CONTINUOUS(0) or VISION_CONTROL_EVENT_TARGET(1)"
#endif

// 云台系统辨识模式。默认关闭; 需要实验固件时通过 CMake 打开。
#define GIMBAL_SYSID_NONE      0u
#define GIMBAL_SYSID_YAW_PRBS  1u
#define GIMBAL_SYSID_PITCH_FF  2u
#define GIMBAL_SYSID_YAW_STEP  3u
#define GIMBAL_SYSID_PITCH_HYST 4u
#ifndef GIMBAL_SYSID_MODE
#define GIMBAL_SYSID_MODE GIMBAL_SYSID_NONE
#endif
#if (GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE) && \
    (GIMBAL_SYSID_MODE != GIMBAL_SYSID_YAW_PRBS) && \
    (GIMBAL_SYSID_MODE != GIMBAL_SYSID_PITCH_FF) && \
    (GIMBAL_SYSID_MODE != GIMBAL_SYSID_YAW_STEP) && \
    (GIMBAL_SYSID_MODE != GIMBAL_SYSID_PITCH_HYST)
#error "GIMBAL_SYSID_MODE must be 0=NONE, 1=YAW_PRBS, 2=PITCH_FF, 3=YAW_STEP, or 4=PITCH_HYST"
#endif

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
    float yaw;                 // 目标 yaw 角,单位 rad,+ 为从上往下看逆时针
    float pitch;               // 目标 pitch 角,单位 rad,+ 为相机抬头
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
