#include "gm6020_internal.h"

#include "general_def.h"

static CAN_HandleTypeDef *gm6020_can_ready_list[GM6020_MAX_NUM];
static uint8_t gm6020_can_ready_count = 0u;

static uint8_t GM6020_CANReady(const CAN_HandleTypeDef *hcan)
{
    for (uint8_t i = 0; i < gm6020_can_ready_count; ++i)
    {
        if (gm6020_can_ready_list[i] == hcan)
        {
            return 1u;
        }
    }
    return 0u;
}

static void GM6020_CANMarkReady(CAN_HandleTypeDef *hcan)
{
    if ((hcan == NULL) || GM6020_CANReady(hcan) || (gm6020_can_ready_count >= GM6020_MAX_NUM))
    {
        return;
    }

    gm6020_can_ready_list[gm6020_can_ready_count++] = hcan;
}

// 将同一 CAN 总线上的电机输出按 0x1FF/0x2FF 两组打包
static void GM6020_PackTxForBus(CAN_HandleTypeDef *hcan, uint8_t tx_buf[2][8], uint8_t active_group[2])
{
    for (uint8_t i = 0; i < gm6020_count; ++i)
    {
        GM6020_Instance *motor = &gm6020_list[i];

        if ((motor->can_handle != hcan) || (motor->tx_group > 1u))
        {
            continue;
        }

        tx_buf[motor->tx_group][2u * motor->tx_index] = (uint8_t)(motor->output_cmd >> 8);
        tx_buf[motor->tx_group][2u * motor->tx_index + 1u] = (uint8_t)(motor->output_cmd & 0xFF);
        active_group[motor->tx_group] = 1u;
    }
}

static void GM6020_SendBus(CAN_HandleTypeDef *hcan)
{
    static const uint16_t tx_std_id[2] = {0x1FFu, 0x2FFu};
    uint8_t tx_buf[2][8] = {{0}};
    uint8_t active_group[2] = {0};
    CAN_TxHeaderTypeDef tx_header = {
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t tx_mailbox;

    if (hcan == NULL)
    {
        return;
    }

    GM6020_PackTxForBus(hcan, tx_buf, active_group);

    for (uint8_t group = 0; group < 2u; ++group)
    {
        if (!active_group[group])
        {
            continue;
        }

        tx_header.StdId = tx_std_id[group];
        // 发送失败时保留本轮 output_cmd,等待下一周期重发
        if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_buf[group], &tx_mailbox) != HAL_OK)
        {
            break;
        }
    }
}

void GM6020_CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};
    const uint32_t can2_start_filter_bank = 14u;

    if ((hcan == NULL) || GM6020_CANReady(hcan))
    {
        return;
    }

    filter.SlaveStartFilterBank = can2_start_filter_bank;
#if defined(CAN2)
    filter.FilterBank = (hcan->Instance == CAN2) ? can2_start_filter_bank : 0u;
#else
    filter.FilterBank = 0u;
#endif
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(hcan, &filter);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    GM6020_CANMarkReady(hcan);
}

void GM6020_SendAll(void)
{
    CAN_HandleTypeDef *sent_bus[GM6020_MAX_NUM] = {0};
    uint8_t sent_count = 0u;

    // 一条 CAN 总线只发一次,避免同总线多电机重复打包
    for (uint8_t i = 0; i < gm6020_count; ++i)
    {
        CAN_HandleTypeDef *hcan = gm6020_list[i].can_handle;
        uint8_t duplicate = 0u;

        if (hcan == NULL)
        {
            continue;
        }

        for (uint8_t j = 0; j < sent_count; ++j)
        {
            if (sent_bus[j] == hcan)
            {
                duplicate = 1u;
                break;
            }
        }

        if (duplicate)
        {
            continue;
        }

        sent_bus[sent_count++] = hcan;
        GM6020_SendBus(hcan);
    }
}

void GM6020_ParseFeedback(GM6020_Instance *motor, const uint8_t rx_data[8], uint32_t now_tick)
{
    int16_t speed_rpm;

    if ((motor == NULL) || (rx_data == NULL))
    {
        return;
    }

    motor->measure.last_ecd = motor->measure.ecd;
    motor->measure.ecd = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
    speed_rpm = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    motor->measure.speed_rad_s = (float)speed_rpm * RPM_2_RAD_PER_SEC;
    motor->measure.real_current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    motor->measure.temperature = rx_data[6];
    motor->measure.angle_single_round_rad = (float)motor->measure.ecd * GM6020_ECD_TO_RAD;
    motor->last_rx_tick = now_tick;
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

    motor->measure.total_angle_rad = PI2 * (float)motor->measure.total_round + motor->measure.angle_single_round_rad;
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

            if ((motor->can_handle != hcan) || (rx_header.StdId != (uint32_t)(0x204u + motor->motor_id)))
            {
                continue;
            }

            GM6020_ParseFeedback(motor, rx_data, HAL_GetTick());
            break;
        }
    }
}
