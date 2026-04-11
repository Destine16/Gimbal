#include "gimbal_params.h"
#include "general_def.h"
#include "gm6020.h"

#define GM6020_OLD_VOLTAGE_ANGLE_KP          12.0f
#define GM6020_OLD_VOLTAGE_SPEED_KP          600.0f
#define GM6020_OLD_VOLTAGE_SPEED_KI          1600.0f
#define GM6020_OLD_VOLTAGE_CURRENT_KP        0.8f
#define GM6020_OLD_VOLTAGE_CURRENT_KI        100.0f

// 由 pitch 自动阶跃平台段的平均 output_cmd 对 angle_feedback 拟合得到:
// output_ff_raw ~= a * sin(theta) + b
#define GIMBAL_PITCH_OUTPUT_FF_SIN_RAW       (-3815.4f)
#define GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW    (150.0f)

#ifndef GIMBAL_PITCH_OUTPUT_FF_ENABLE
#define GIMBAL_PITCH_OUTPUT_FF_ENABLE 1
#endif

#if GIMBAL_PITCH_OUTPUT_FF_ENABLE
#define GIMBAL_PITCH_OUTPUT_FF_SIN_ACTIVE_RAW       GIMBAL_PITCH_OUTPUT_FF_SIN_RAW
#define GIMBAL_PITCH_OUTPUT_FF_OFFSET_ACTIVE_RAW    GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW
#else
#define GIMBAL_PITCH_OUTPUT_FF_SIN_ACTIVE_RAW       0.0f
#define GIMBAL_PITCH_OUTPUT_FF_OFFSET_ACTIVE_RAW    0.0f
#endif

// Bring-up 安全限幅: 低于 GM6020 说明书硬规格,确认方向和闭环正常后再逐步放开。
#define GIMBAL_BRINGUP_SPEED_REF_MAX_RAD_S       2.0f
#define GIMBAL_BRINGUP_TORQUE_CURRENT_MAX_RAW    3000.0f
#define GIMBAL_BRINGUP_VOLTAGE_CMD_MAX_RAW       5000.0f

const GimbalMotorParam_s GimbalYawParam = {
    .motor_id = GIMBAL_YAW_MOTOR_ID,
    .angle_limit_enable = 0u,
    .angle_feedback_sign = GIMBAL_YAW_FEED_SIGN,
    .speed_feedback_sign = GIMBAL_YAW_GYRO_SIGN,
    .current_feedback_sign = GIMBAL_YAW_CURRENT_SIGN,
    .output_sign = GIMBAL_YAW_OUTPUT_SIGN,
    .max_output_raw = GIMBAL_BRINGUP_VOLTAGE_CMD_MAX_RAW,
    .min_angle_rad = 0.0f,
    .max_angle_rad = 0.0f,
    .output_ff_sin_raw = 0.0f,
    .output_ff_offset_raw = 0.0f,
    .angle_pid = {
        .Kp = GM6020_OLD_VOLTAGE_ANGLE_KP,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = GIMBAL_BRINGUP_SPEED_REF_MAX_RAD_S,
        .DeadBand = 0.0f,
        .Improve = PID_IMPROVE_NONE,
        .IntegralLimit = 0.0f,
    },
    .speed_pid = {
        .Kp = GM6020_OLD_VOLTAGE_SPEED_KP,
        .Ki = GM6020_OLD_VOLTAGE_SPEED_KI,
        .Kd = 0.0f,
        .MaxOut = GIMBAL_BRINGUP_TORQUE_CURRENT_MAX_RAW,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = GIMBAL_BRINGUP_TORQUE_CURRENT_MAX_RAW,
    },
    .current_pid = {
        .Kp = GM6020_OLD_VOLTAGE_CURRENT_KP,
        .Ki = GM6020_OLD_VOLTAGE_CURRENT_KI,
        .Kd = 0.0f,
        .MaxOut = GIMBAL_BRINGUP_VOLTAGE_CMD_MAX_RAW,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = GIMBAL_BRINGUP_VOLTAGE_CMD_MAX_RAW,
    },
};

const GimbalMotorParam_s GimbalPitchParam = {
    .motor_id = GIMBAL_PITCH_MOTOR_ID,
    .angle_limit_enable = 1u,
    .angle_feedback_sign = GIMBAL_PITCH_FEED_SIGN,
    .speed_feedback_sign = GIMBAL_PITCH_GYRO_SIGN,
    .current_feedback_sign = GIMBAL_PITCH_CURRENT_SIGN,
    .output_sign = GIMBAL_PITCH_OUTPUT_SIGN,
    .max_output_raw = GIMBAL_BRINGUP_VOLTAGE_CMD_MAX_RAW,
    .min_angle_rad = GIMBAL_PITCH_MIN_RAD,
    .max_angle_rad = GIMBAL_PITCH_MAX_RAD,
    .output_ff_sin_raw = GIMBAL_PITCH_OUTPUT_FF_SIN_ACTIVE_RAW,
    .output_ff_offset_raw = GIMBAL_PITCH_OUTPUT_FF_OFFSET_ACTIVE_RAW,
    .angle_pid = {
        .Kp = GM6020_OLD_VOLTAGE_ANGLE_KP,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = GIMBAL_BRINGUP_SPEED_REF_MAX_RAD_S,
        .Improve = PID_IMPROVE_NONE,
        .IntegralLimit = 0.0f,
    },
    .speed_pid = {
        .Kp = GM6020_OLD_VOLTAGE_SPEED_KP,
        .Ki = GM6020_OLD_VOLTAGE_SPEED_KI,
        .Kd = 0.0f,
        .MaxOut = GIMBAL_BRINGUP_TORQUE_CURRENT_MAX_RAW,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = GIMBAL_BRINGUP_TORQUE_CURRENT_MAX_RAW,
    },
    .current_pid = {
        .Kp = GM6020_OLD_VOLTAGE_CURRENT_KP,
        .Ki = GM6020_OLD_VOLTAGE_CURRENT_KI,
        .Kd = 0.0f,
        .MaxOut = GIMBAL_BRINGUP_VOLTAGE_CMD_MAX_RAW,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = GIMBAL_BRINGUP_VOLTAGE_CMD_MAX_RAW,
    },
};
