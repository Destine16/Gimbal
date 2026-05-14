#include "gimbal_params.h"
#include "general_def.h"
#include "gm6020.h"

#define GM6020_YAW_VOLTAGE_ANGLE_KP          48.0000f
#define GM6020_YAW_VOLTAGE_SPEED_KP          3600.0000f
#define GM6020_YAW_VOLTAGE_SPEED_KI          600.0000f
#define GM6020_PITCH_VOLTAGE_ANGLE_KP        28.0000f
#define GM6020_PITCH_VOLTAGE_SPEED_KP        2800.0000f
#define GM6020_PITCH_VOLTAGE_SPEED_KI        420.0000f
#define GM6020_OLD_VOLTAGE_CURRENT_KP        0.8f
#define GM6020_OLD_VOLTAGE_CURRENT_KI        100.0f

// Pitch feedforward: output_ff_raw ~= a * sin(theta) + b + clamp(kv * speed_ref) + h * motion_sign
#define GIMBAL_PITCH_OUTPUT_FF_SIN_RAW       (-1115.8459f)
#define GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW    (-97.4462f)
#define GIMBAL_PITCH_OUTPUT_FF_SPEED_RAW     (-486.6887f)
#define GIMBAL_PITCH_OUTPUT_FF_SPEED_MAX_RAW 339.2f
#define GIMBAL_PITCH_OUTPUT_FF_HYST_RAW      (-300.0000f)
#define GIMBAL_PITCH_OUTPUT_FF_HYST_TRANSITION_RAD_S 0.50f

#ifndef GIMBAL_PITCH_OUTPUT_FF_ENABLE
#define GIMBAL_PITCH_OUTPUT_FF_ENABLE 1
#endif

#ifndef GIMBAL_PITCH_OUTPUT_HYST_ENABLE
#define GIMBAL_PITCH_OUTPUT_HYST_ENABLE 0
#endif

#ifndef GIMBAL_PITCH_OUTPUT_SPEED_ENABLE
#define GIMBAL_PITCH_OUTPUT_SPEED_ENABLE 0
#endif

#if GIMBAL_PITCH_OUTPUT_FF_ENABLE
#define GIMBAL_PITCH_OUTPUT_FF_SIN_ACTIVE_RAW       GIMBAL_PITCH_OUTPUT_FF_SIN_RAW
#define GIMBAL_PITCH_OUTPUT_FF_OFFSET_ACTIVE_RAW    GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW
#else
#define GIMBAL_PITCH_OUTPUT_FF_SIN_ACTIVE_RAW       0.0f
#define GIMBAL_PITCH_OUTPUT_FF_OFFSET_ACTIVE_RAW    0.0f
#endif

#if GIMBAL_PITCH_OUTPUT_SPEED_ENABLE
#define GIMBAL_PITCH_OUTPUT_FF_SPEED_ACTIVE_RAW     GIMBAL_PITCH_OUTPUT_FF_SPEED_RAW
#define GIMBAL_PITCH_OUTPUT_FF_SPEED_MAX_ACTIVE_RAW GIMBAL_PITCH_OUTPUT_FF_SPEED_MAX_RAW
#else
#define GIMBAL_PITCH_OUTPUT_FF_SPEED_ACTIVE_RAW     0.0f
#define GIMBAL_PITCH_OUTPUT_FF_SPEED_MAX_ACTIVE_RAW 0.0f
#endif

#if GIMBAL_PITCH_OUTPUT_HYST_ENABLE
#define GIMBAL_PITCH_OUTPUT_FF_HYST_ACTIVE_RAW      GIMBAL_PITCH_OUTPUT_FF_HYST_RAW
#else
#define GIMBAL_PITCH_OUTPUT_FF_HYST_ACTIVE_RAW      0.0f
#endif

// Bring-up 安全限幅: 低于 GM6020 说明书硬规格,确认方向和闭环正常后再逐步放开。
#define GIMBAL_YAW_SPEED_REF_MAX_RAD_S           4.8f
#define GIMBAL_PITCH_SPEED_REF_MAX_RAD_S         3.6f
#define GIMBAL_BRINGUP_TORQUE_CURRENT_MAX_RAW    3800.0f
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
    .output_ff_speed_raw = 0.0f,
    .output_ff_speed_max_raw = 0.0f,
    .output_ff_hyst_raw = 0.0f,
    .output_ff_hyst_transition_rad_s = 0.0f,
    .angle_pid = {
        .Kp = GM6020_YAW_VOLTAGE_ANGLE_KP,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = GIMBAL_YAW_SPEED_REF_MAX_RAD_S,
        .DeadBand = 0.0f,
        .Improve = PID_IMPROVE_NONE,
        .IntegralLimit = 0.0f,
    },
    .speed_pid = {
        .Kp = GM6020_YAW_VOLTAGE_SPEED_KP,
        .Ki = GM6020_YAW_VOLTAGE_SPEED_KI,
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
    .output_ff_speed_raw = GIMBAL_PITCH_OUTPUT_FF_SPEED_ACTIVE_RAW,
    .output_ff_speed_max_raw = GIMBAL_PITCH_OUTPUT_FF_SPEED_MAX_ACTIVE_RAW,
    .output_ff_hyst_raw = GIMBAL_PITCH_OUTPUT_FF_HYST_ACTIVE_RAW,
    .output_ff_hyst_transition_rad_s = GIMBAL_PITCH_OUTPUT_FF_HYST_TRANSITION_RAD_S,
    .angle_pid = {
        .Kp = GM6020_PITCH_VOLTAGE_ANGLE_KP,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = GIMBAL_PITCH_SPEED_REF_MAX_RAD_S,
        .Improve = PID_IMPROVE_NONE,
        .IntegralLimit = 0.0f,
    },
    .speed_pid = {
        .Kp = GM6020_PITCH_VOLTAGE_SPEED_KP,
        .Ki = GM6020_PITCH_VOLTAGE_SPEED_KI,
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
