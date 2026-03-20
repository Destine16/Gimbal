#include "gm6020.h"

#include <string.h>

#include "general_def.h"
#include "robot_def.h"

static GM6020_Instance gm6020_list[GM6020_MAX_NUM];
static uint8_t gm6020_count = 0;

static uint8_t can1_ready = 0;

static uint16_t GM6020_DaemonReloadCount(void)
{
    uint32_t count = (GIMBAL_MOTOR_OFFLINE_TIMEOUT_MS + DAEMON_TASK_PERIOD_MS - 1u) / DAEMON_TASK_PERIOD_MS;
    return (uint16_t)((count == 0u) ? 1u : count);
}

static uint8_t GM6020_RuntimeOnline(const GM6020_Instance *motor, uint32_t now_tick)
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

static int16_t GM6020_OutputClamp(float command)
{
    if (command > 30000.0f)
    {
        return 30000;
    }
    if (command < -30000.0f)
    {
        return -30000;
    }
    return (int16_t)command;
}

static void GM6020_AssignTxGroup(GM6020_Instance *motor)
{
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

void GM6020_CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};

    if ((hcan == &hcan1) && can1_ready)
    {
        return;
    }

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &filter);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    if (hcan == &hcan1)
    {
        can1_ready = 1;
    }
}

GM6020_Instance *GM6020_Init(const GM6020_Init_Config_s *config)
{
    GM6020_Instance *motor;

    if ((config == NULL) || (gm6020_count >= GM6020_MAX_NUM))
    {
        return NULL;
    }

    motor = &gm6020_list[gm6020_count++];
    memset(motor, 0, sizeof(*motor));

    motor->can_handle = config->can_handle;
    motor->motor_id = config->motor_id;
    motor->angle_feedback_ptr = config->angle_feedback_ptr;
    motor->speed_feedback_ptr = config->speed_feedback_ptr;
    motor->current_feedback_sign = (config->current_feedback_sign == 0.0f) ? 1.0f : config->current_feedback_sign;
    motor->output_sign = (config->output_sign == 0.0f) ? 1.0f : config->output_sign;
    motor->enabled = 0u;

    PIDInit(&motor->angle_pid, (PID_Init_Config_s *)&config->angle_pid_config);
    PIDInit(&motor->speed_pid, (PID_Init_Config_s *)&config->speed_pid_config);
    PIDInit(&motor->current_pid, (PID_Init_Config_s *)&config->current_pid_config);
    motor->daemon = DaemonRegister(&(Daemon_Init_Config_s){
        .reload_count = GM6020_DaemonReloadCount(),
        .init_count = 0u,
        .callback = NULL,
        .owner_id = motor,
    });

    GM6020_AssignTxGroup(motor);
    GM6020_CAN_Init(config->can_handle);
    return motor;
}

void GM6020_SetAngleRef(GM6020_Instance *motor, float angle_deg)
{
    if (motor == NULL)
    {
        return;
    }
    motor->angle_ref_deg = angle_deg;
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
        GM6020_ResetPIDState(&motor->angle_pid);
        GM6020_ResetPIDState(&motor->speed_pid);
        GM6020_ResetPIDState(&motor->current_pid);
    }
}

uint8_t GM6020_IsOnline(const GM6020_Instance *motor, uint32_t now_tick)
{
    return GM6020_RuntimeOnline(motor, now_tick);
}

void GM6020_ControlAll(void)
{
    static const uint16_t tx_std_id[2] = {0x1FF, 0x2FF};
    uint8_t tx_buf[2][8] = {{0}};
    CAN_TxHeaderTypeDef tx_header = {
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t tx_mailbox;

    for (uint8_t i = 0; i < gm6020_count; ++i)
    {
        GM6020_Instance *motor = &gm6020_list[i];
        uint32_t now_tick = HAL_GetTick();
        float angle_ref;
        float angle_feedback;
        float speed_ref;
        float speed_feedback;
        float current_ref;
        float current_feedback;
        float voltage_ref;

        if ((motor->can_handle != &hcan1) || (motor->tx_group > 1u))
        {
            continue;
        }

        if (!motor->enabled)
        {
            motor->output_cmd = 0;
        }
        else if (!GM6020_RuntimeOnline(motor, now_tick))
        {
            motor->output_cmd = 0;
            GM6020_ResetPIDState(&motor->angle_pid);
            GM6020_ResetPIDState(&motor->speed_pid);
            GM6020_ResetPIDState(&motor->current_pid);
        }
        else
        {
            angle_ref = motor->angle_ref_deg;
            angle_feedback = (motor->angle_feedback_ptr != NULL) ? *motor->angle_feedback_ptr : 0.0f;
            speed_feedback = (motor->speed_feedback_ptr != NULL) ? *motor->speed_feedback_ptr : 0.0f;
            current_feedback = motor->current_feedback_sign * (float)motor->measure.real_current;

            speed_ref = PIDCalculate(&motor->angle_pid, angle_feedback, angle_ref);
            current_ref = PIDCalculate(&motor->speed_pid, speed_feedback, speed_ref);
            voltage_ref = PIDCalculate(&motor->current_pid, current_feedback, current_ref);
            motor->output_cmd = GM6020_OutputClamp(motor->output_sign * voltage_ref);
        }

        tx_buf[motor->tx_group][2u * motor->tx_index] = (uint8_t)(motor->output_cmd >> 8);
        tx_buf[motor->tx_group][2u * motor->tx_index + 1u] = (uint8_t)(motor->output_cmd & 0xFF);
    }

    for (uint8_t group = 0; group < 2u; ++group)
    {
        tx_header.StdId = tx_std_id[group];
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_buf[group], &tx_mailbox);
    }
}

void GM6020_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        for (uint8_t i = 0; i < gm6020_count; ++i)
        {
            GM6020_Instance *motor = &gm6020_list[i];
            int16_t speed_rpm;

            if ((motor->can_handle != hcan) || (rx_header.StdId != (uint32_t)(0x204u + motor->motor_id)))
            {
                continue;
            }

            motor->measure.last_ecd = motor->measure.ecd;
            motor->measure.ecd = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
            speed_rpm = (int16_t)((rx_data[2] << 8) | rx_data[3]);
            motor->measure.speed_deg_s = (float)speed_rpm * RPM_2_ANGLE_PER_SEC;
            motor->measure.real_current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
            motor->measure.temperature = rx_data[6];
            motor->measure.angle_single_round_deg = (float)motor->measure.ecd * GM6020_ECD_TO_DEG;
            motor->last_rx_tick = HAL_GetTick();
            // 收到一帧电机反馈即刷新在线状态
            DaemonReload(motor->daemon);

            if ((int32_t)motor->measure.ecd - (int32_t)motor->measure.last_ecd > 4096)
            {
                motor->measure.total_round--;
            }
            else if ((int32_t)motor->measure.ecd - (int32_t)motor->measure.last_ecd < -4096)
            {
                motor->measure.total_round++;
            }

            motor->measure.total_angle_deg = 360.0f * (float)motor->measure.total_round + motor->measure.angle_single_round_deg;
            break;
        }
    }
}
