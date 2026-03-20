#ifndef GIMBAL_PARAMS_H
#define GIMBAL_PARAMS_H

#include "controller.h"
#include "robot_def.h"

typedef struct
{
    uint8_t motor_id;
    uint8_t angle_limit_enable;
    float angle_feedback_sign;
    float speed_feedback_sign;
    float current_feedback_sign;
    float output_sign;
    float min_angle_deg;
    float max_angle_deg;
    PID_Init_Config_s angle_pid;
    PID_Init_Config_s speed_pid;
    PID_Init_Config_s current_pid;
} GimbalMotorParam_s;

extern const GimbalMotorParam_s GimbalYawParam;
extern const GimbalMotorParam_s GimbalPitchParam;

#endif
