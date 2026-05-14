#include "gm6020_internal.h"

#include <math.h>

#define GM6020_OUTPUT_FF_HYST_TRANSITION_RAD_S 0.50f

static void GM6020_ResetPIDState(PIDInstance *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->ERRORHandler.ERRORCount = 0u;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
    DWT_GetDeltaT(&pid->DWT_CNT);
}

static int16_t GM6020_OutputClamp(float command, float max_output_raw)
{
    if ((max_output_raw <= 0.0f) || (max_output_raw > GM6020_VOLTAGE_CMD_MAX_RAW))
    {
        max_output_raw = GM6020_VOLTAGE_CMD_MAX_RAW;
    }

    if (command > max_output_raw)
    {
        return (int16_t)max_output_raw;
    }
    if (command < -max_output_raw)
    {
        return (int16_t)(-max_output_raw);
    }
    return (int16_t)command;
}

static float GM6020_SmoothFrictionSign(float speed_ref_rad_s, float transition_rad_s)
{
    if (transition_rad_s <= 0.0f)
    {
        transition_rad_s = GM6020_OUTPUT_FF_HYST_TRANSITION_RAD_S;
    }
    return tanhf(speed_ref_rad_s / transition_rad_s);
}

static float GM6020_ClampFloat(float value, float limit_abs)
{
    if (limit_abs <= 0.0f)
    {
        return value;
    }
    if (value > limit_abs)
    {
        return limit_abs;
    }
    if (value < -limit_abs)
    {
        return -limit_abs;
    }
    return value;
}

void GM6020_ResetControlState(GM6020_Instance *motor)
{
    if (motor == NULL)
    {
        return;
    }

    GM6020_ResetPIDState(&motor->angle_pid);
    GM6020_ResetPIDState(&motor->speed_pid);
    GM6020_ResetPIDState(&motor->current_pid);
    motor->output_ff_motion_sign = 0.0f;
}

// 逐个电机更新三环输出,发送动作由 CAN 层统一完成
static void GM6020_ControlStep(GM6020_Instance *motor, uint32_t now_tick)
{
    float angle_ref;
    float angle_feedback;
    float speed_ref;
    float speed_feedback;
    float current_ref;
    float current_feedback;
    float voltage_ref;
    float output_ff_raw;

    if (motor == NULL)
    {
        return;
    }

    if (!motor->enabled)
    {
        motor->output_cmd = 0;
        motor->speed_ref_rad_s = 0.0f;
        motor->current_ref_raw = 0.0f;
        motor->voltage_ref_raw = 0.0f;
        motor->output_ff_raw = 0.0f;
        return;
    }

    if (!GM6020_RuntimeOnline(motor, now_tick))
    {
        motor->output_cmd = 0;
        motor->speed_ref_rad_s = 0.0f;
        motor->current_ref_raw = 0.0f;
        motor->voltage_ref_raw = 0.0f;
        motor->output_ff_raw = 0.0f;
        GM6020_ResetControlState(motor);
        return;
    }

    angle_feedback = (motor->angle_feedback_ptr != NULL) ? *motor->angle_feedback_ptr : 0.0f;
    speed_feedback = (motor->speed_feedback_ptr != NULL) ? *motor->speed_feedback_ptr : 0.0f;
    current_feedback = motor->current_feedback_sign * (float)motor->measure.real_current;
    angle_ref = motor->angle_ref_rad;
    speed_ref = PIDCalculate(&motor->angle_pid, angle_feedback, angle_ref);
    current_ref = PIDCalculate(&motor->speed_pid, speed_feedback, speed_ref);
    voltage_ref = PIDCalculate(&motor->current_pid, current_feedback, current_ref);
    if (motor->output_ff_hyst_raw != 0.0f)
    {
        motor->output_ff_motion_sign = GM6020_SmoothFrictionSign(speed_ref,
                                                                 motor->output_ff_hyst_transition_rad_s);
    }
    else
    {
        motor->output_ff_motion_sign = 0.0f;
    }
    output_ff_raw = motor->output_ff_sin_raw * sinf(angle_feedback) +
                    motor->output_ff_offset_raw +
                    GM6020_ClampFloat(motor->output_ff_speed_raw * speed_ref,
                                      motor->output_ff_speed_max_raw) +
                    motor->output_ff_hyst_raw * motor->output_ff_motion_sign;

    motor->angle_feedback_rad = angle_feedback;
    motor->speed_ref_rad_s = speed_ref;
    motor->speed_feedback_rad_s = speed_feedback;
    motor->current_ref_raw = current_ref;
    motor->current_feedback_raw = current_feedback;
    motor->voltage_ref_raw = voltage_ref;
    motor->output_ff_raw = output_ff_raw;

    motor->output_cmd = GM6020_OutputClamp(motor->output_sign * voltage_ref + output_ff_raw,
                                           motor->max_output_raw);
}

void GM6020_UpdateAll(void)
{
    uint32_t now_tick = HAL_GetTick();

    for (uint8_t i = 0; i < gm6020_count; ++i)
    {
        GM6020_ControlStep(&gm6020_list[i], now_tick);
    }
}

void GM6020_ControlAll(void)
{
    // 先更新每个电机的输出命令,再按 CAN 总线统一发送
    GM6020_UpdateAll();
    GM6020_SendAll();
}
