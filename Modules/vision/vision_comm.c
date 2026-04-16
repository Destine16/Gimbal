#include "vision_comm.h"

#include <string.h>

#include "crc16.h"
#include "daemon.h"
#include "robot_def.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

#define VISION_CMD_SOF1               0xA5u
#define VISION_CMD_SOF2               0x5Au
#define VISION_CMD_FRAME_LEN           8u
#define VISION_CMD_CRC_INPUT_LEN       6u
#define VISION_STATUS_SOF1            0x5Au
#define VISION_STATUS_SOF2            0xA5u
#define VISION_STATUS_FRAME_LEN       16u
#define VISION_STATUS_CRC_INPUT_LEN   14u

typedef enum
{
    RX_WAIT_SOF1 = 0,
    RX_WAIT_SOF2,
    RX_WAIT_FRAME_TAIL,
} VisionRxState_e;

typedef struct
{
    VisionRxState_e state;
    uint8_t frame[VISION_CMD_FRAME_LEN];
    uint8_t index;
} VisionRxParser_t;

static VisionRxParser_t vision_rx_parser;
static VisionCmd_t latest_vision_cmd;
static VisionStatus_t latest_vision_status;
static uint32_t latest_status_tx_tick;
static uint8_t latest_cmd_valid;
static uint8_t usb_host_ready;
static uint8_t vision_status_frame[VISION_STATUS_FRAME_LEN];
static DaemonInstance *vision_daemon_instance; // 视觉命令在线监测

static uint16_t VisionComm_DaemonReloadCount(void)
{
    uint32_t count = (VISION_CMD_TIMEOUT_MS + DAEMON_TASK_PERIOD_MS - 1u) / DAEMON_TASK_PERIOD_MS;
    return (uint16_t)((count == 0u) ? 1u : count);
}

static void VisionComm_ResetParser(void)
{
    memset(&vision_rx_parser, 0, sizeof(vision_rx_parser));
    vision_rx_parser.state = RX_WAIT_SOF1;
}

static void VisionComm_HandleFrame(const uint8_t frame[VISION_CMD_FRAME_LEN])
{
    memcpy(&latest_vision_cmd, &frame[2], sizeof(VisionCmd_t));
    latest_cmd_valid = 1u;
    // 收到一帧合法视觉命令即认为视觉链路仍在线
    DaemonReload(vision_daemon_instance);
}

static uint8_t VisionComm_TxStatusFrame(void)
{
    uint16_t crc;
    VisionStatus_t status_frame_data = latest_vision_status;

    status_frame_data.last_rx_delta_yaw_1e4rad = latest_vision_cmd.delta_yaw_1e4rad;
    status_frame_data.last_rx_delta_pitch_1e4rad = latest_vision_cmd.delta_pitch_1e4rad;

    vision_status_frame[0] = VISION_STATUS_SOF1;
    vision_status_frame[1] = VISION_STATUS_SOF2;
    memcpy(&vision_status_frame[2], &status_frame_data, sizeof(status_frame_data));
    crc = crc_modbus(vision_status_frame, VISION_STATUS_CRC_INPUT_LEN);
    vision_status_frame[14] = (uint8_t)(crc & 0x00FFu);
    vision_status_frame[15] = (uint8_t)((crc >> 8) & 0x00FFu);
    return CDC_Transmit_FS(vision_status_frame, VISION_STATUS_FRAME_LEN);
}

static void VisionComm_ParseByte(uint8_t byte)
{
    uint16_t calc_crc;
    uint16_t recv_crc;

    switch (vision_rx_parser.state)
    {
    case RX_WAIT_SOF1:
        if (byte == VISION_CMD_SOF1)
        {
            vision_rx_parser.frame[0] = byte;
            vision_rx_parser.state = RX_WAIT_SOF2;
        }
        break;

    case RX_WAIT_SOF2:
        if (byte == VISION_CMD_SOF2)
        {
            vision_rx_parser.frame[1] = byte;
            vision_rx_parser.index = 2u;
            vision_rx_parser.state = RX_WAIT_FRAME_TAIL;
        }
        else
        {
            VisionComm_ResetParser();
            if (byte == VISION_CMD_SOF1)
            {
                vision_rx_parser.frame[0] = byte;
                vision_rx_parser.state = RX_WAIT_SOF2;
            }
        }
        break;

    case RX_WAIT_FRAME_TAIL:
        vision_rx_parser.frame[vision_rx_parser.index++] = byte;
        if (vision_rx_parser.index >= VISION_CMD_FRAME_LEN)
        {
            calc_crc = crc_modbus(vision_rx_parser.frame, VISION_CMD_CRC_INPUT_LEN);
            recv_crc = (uint16_t)vision_rx_parser.frame[6] |
                       ((uint16_t)vision_rx_parser.frame[7] << 8);
            if (calc_crc == recv_crc)
            {
                VisionComm_HandleFrame(vision_rx_parser.frame);
            }
            VisionComm_ResetParser();
        }
        break;

    default:
        VisionComm_ResetParser();
        break;
    }
}

void VisionComm_Init(void)
{
    memset(&latest_vision_cmd, 0, sizeof(latest_vision_cmd));
    memset(&latest_vision_status, 0, sizeof(latest_vision_status));
    latest_status_tx_tick = 0u;
    latest_cmd_valid = 0u;
    usb_host_ready = 0u;
    if (vision_daemon_instance == NULL)
    {
        vision_daemon_instance = DaemonRegister(&(Daemon_Init_Config_s){
            .reload_count = VisionComm_DaemonReloadCount(),
            .init_count = 0u,
            .callback = NULL,
            .owner_id = NULL,
        });
    }
    VisionComm_ResetParser();
}

void VisionComm_Task(void)
{
    uint32_t now_tick = HAL_GetTick();

    if (!usb_host_ready)
    {
        return;
    }

    if ((now_tick - latest_status_tx_tick) >= VISION_STATUS_TX_PERIOD_MS)
    {
        if (VisionComm_TxStatusFrame() == USBD_OK)
        {
            latest_status_tx_tick = now_tick;
        }
    }
}

void VisionComm_RxBytes(const uint8_t *data, uint16_t len)
{
    usb_host_ready = 1u;
    for (uint16_t i = 0; i < len; ++i)
    {
        VisionComm_ParseByte(data[i]);
    }
}

uint8_t VisionComm_GetVisionCmd(VisionCmd_t *cmd)
{
    if (!latest_cmd_valid || !VisionComm_IsOnline())
    {
        return 0u;
    }

    memcpy(cmd, &latest_vision_cmd, sizeof(VisionCmd_t));
    return 1u;
}

void VisionComm_UpdateStatus(const VisionStatus_t *status)
{
    memcpy(&latest_vision_status, status, sizeof(latest_vision_status));
}

uint8_t VisionComm_IsOnline(void)
{
    return (uint8_t)(latest_cmd_valid && DaemonIsOnline(vision_daemon_instance));
}

void VisionComm_SetUsbHostReady(uint8_t ready)
{
    usb_host_ready = ready ? 1u : 0u;
    if (!usb_host_ready)
    {
        latest_status_tx_tick = 0u;
    }
}
