#include "gm6020_internal.h"

#include <string.h>

#include "robot_def.h"

GM6020_Instance gm6020_list[GM6020_MAX_NUM]; // 运行时电机实例数组,调试器里可直接 watch gm6020_list[i]
uint8_t gm6020_count = 0u;
volatile GM6020_Debug_s gm6020_debug;

uint16_t GM6020_DaemonReloadCount(void)
{
    uint32_t count = (GIMBAL_MOTOR_OFFLINE_TIMEOUT_MS + DAEMON_TASK_PERIOD_MS - 1u) / DAEMON_TASK_PERIOD_MS;
    return (uint16_t)((count == 0u) ? 1u : count);
}

uint8_t GM6020_RuntimeOnline(const GM6020_Instance *motor, uint32_t now_tick)
{
    if ((motor != NULL) && (motor->daemon != NULL))
    {
        (void)now_tick;
        return DaemonIsOnline(motor->daemon);
    }

    return (uint8_t)((motor != NULL) &&
                     (motor->last_rx_tick != 0u) &&
                     ((now_tick - motor->last_rx_tick) <= GIMBAL_MOTOR_OFFLINE_TIMEOUT_MS));
}

void GM6020_AssignTxGroup(GM6020_Instance *motor)
{
    if (motor == NULL)
    {
        return;
    }

    if (motor->motor_id <= 4u)
    {
        motor->tx_group = 0u;
        motor->tx_index = (uint8_t)(motor->motor_id - 1u);
    }
    else
    {
        motor->tx_group = 1u;
        motor->tx_index = (uint8_t)(motor->motor_id - 5u);
    }
}

GM6020_Instance *GM6020_Init(const GM6020_Init_Config_s *config)
{
    GM6020_Instance *motor;

    // 配置为空或实例池已满时,初始化失败
    if ((config == NULL) || (gm6020_count >= GM6020_MAX_NUM))
    {
        return NULL;
    }

    // 从静态实例池中取出一个空槽,作为新的 GM6020 运行时对象
    motor = &gm6020_list[gm6020_count++];
    memset(motor, 0, sizeof(*motor));

    // 记录总线、ID 和反馈来源; 符号为 0 时按 1.0f 处理,避免把方向配置误写成 0 后整条链失效
    motor->can_handle = config->can_handle;
    motor->motor_id = config->motor_id;
    motor->angle_feedback_ptr = config->angle_feedback_ptr;
    motor->speed_feedback_ptr = config->speed_feedback_ptr;
    motor->current_feedback_sign = (config->current_feedback_sign == 0.0f) ? 1.0f : config->current_feedback_sign;
    motor->output_sign = (config->output_sign == 0.0f) ? 1.0f : config->output_sign;
    motor->max_output_raw = config->max_output_raw;
    motor->output_ff_sin_raw = config->output_ff_sin_raw;
    motor->output_ff_offset_raw = config->output_ff_offset_raw;
    if ((motor->max_output_raw <= 0.0f) || (motor->max_output_raw > GM6020_VOLTAGE_CMD_MAX_RAW))
    {
        motor->max_output_raw = GM6020_VOLTAGE_CMD_MAX_RAW;
    }
    // 初始化后默认不输出,由上层显式 Enable() 再进入控制
    motor->enabled = 0u;

    // 三环 PID 各自按配置初始化; 这里不直接使能输出
    PIDInit(&motor->angle_pid, (PID_Init_Config_s *)&config->angle_pid_config);
    PIDInit(&motor->speed_pid, (PID_Init_Config_s *)&config->speed_pid_config);
    PIDInit(&motor->current_pid, (PID_Init_Config_s *)&config->current_pid_config);
    // 为该电机注册在线监测实例; 初始先按离线处理,等待首次收到反馈帧后再上线
    motor->daemon = DaemonRegister(&(Daemon_Init_Config_s){
        .reload_count = GM6020_DaemonReloadCount(),
        .init_count = 0u,
        .callback = NULL,
        .owner_id = motor,
    });

    // 根据电机 ID 计算其属于 0x1FF 还是 0x2FF 发送组,以及组内索引
    GM6020_AssignTxGroup(motor);
    // 同一 CAN 总线只会被初始化一次
    GM6020_CAN_Init(config->can_handle);
    return motor;
}

void GM6020_SetAngleRef(GM6020_Instance *motor, float angle_rad)
{
    if (motor == NULL)
    {
        return;
    }
    motor->angle_ref_rad = angle_rad;
}

void GM6020_Enable(GM6020_Instance *motor)
{
    if (motor != NULL)
    {
        motor->enabled = 1u;
    }
}

void GM6020_Stop(GM6020_Instance *motor)
{
    if (motor != NULL)
    {
        motor->enabled = 0u;
        motor->output_cmd = 0;
        GM6020_ResetControlState(motor);
    }
}

uint8_t GM6020_IsOnline(const GM6020_Instance *motor, uint32_t now_tick)
{
    return GM6020_RuntimeOnline(motor, now_tick);
}

uint8_t GM6020_GetControlSnapshot(uint8_t motor_id, GM6020_ControlSnapshot_s *snapshot)
{
    uint32_t now_tick = HAL_GetTick();

    if (snapshot == NULL)
    {
        return 0u;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    for (uint8_t i = 0u; i < gm6020_count; ++i)
    {
        GM6020_Instance *motor = &gm6020_list[i];
        if (motor->motor_id != motor_id)
        {
            continue;
        }

        snapshot->valid = 1u;
        snapshot->enabled = motor->enabled;
        snapshot->online = GM6020_RuntimeOnline(motor, now_tick);
        snapshot->motor_id = motor->motor_id;
        snapshot->angle_ref_rad = motor->angle_ref_rad;
        snapshot->angle_feedback_rad = motor->angle_feedback_rad;
        snapshot->speed_ref_rad_s = motor->speed_ref_rad_s;
        snapshot->speed_feedback_rad_s = motor->speed_feedback_rad_s;
        snapshot->current_ref_raw = motor->current_ref_raw;
        snapshot->current_feedback_raw = motor->current_feedback_raw;
        snapshot->voltage_ref_raw = motor->voltage_ref_raw;
        snapshot->output_ff_raw = motor->output_ff_raw;
        snapshot->output_cmd = motor->output_cmd;
        snapshot->real_current = motor->measure.real_current;
        snapshot->encoder_raw = motor->measure.ecd;
        snapshot->encoder_total_round = motor->measure.total_round;
        snapshot->encoder_single_round_rad = motor->measure.angle_single_round_rad;
        snapshot->encoder_total_angle_rad = motor->measure.total_angle_rad;
        snapshot->motor_speed_rad_s = motor->measure.speed_rad_s;
        snapshot->angle_pid_pout = motor->angle_pid.Pout;
        snapshot->angle_pid_iout = motor->angle_pid.Iout;
        snapshot->angle_pid_dout = motor->angle_pid.Dout;
        snapshot->angle_pid_output = motor->angle_pid.Output;
        snapshot->speed_pid_pout = motor->speed_pid.Pout;
        snapshot->speed_pid_iout = motor->speed_pid.Iout;
        snapshot->speed_pid_dout = motor->speed_pid.Dout;
        snapshot->speed_pid_output = motor->speed_pid.Output;
        snapshot->current_pid_pout = motor->current_pid.Pout;
        snapshot->current_pid_iout = motor->current_pid.Iout;
        snapshot->current_pid_dout = motor->current_pid.Dout;
        snapshot->current_pid_output = motor->current_pid.Output;
        return 1u;
    }

    return 0u;
}
