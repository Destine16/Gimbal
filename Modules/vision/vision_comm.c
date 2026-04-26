#include "vision_comm.h"

#include <string.h>

#include "crc16.h"
#include "daemon.h"
#include "robot_def.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

#define VISION_CMD_SOF1               0xA5u
#define VISION_CMD_SOF2               0x5Au
#define VISION_CMD_FRAME_LEN           10u
#define VISION_CMD_CRC_INPUT_LEN       8u
#define VISION_STATUS_SOF1            0x5Au
#define VISION_STATUS_SOF2            0xA5u
#define VISION_STATUS_FRAME_LEN       13u
#define VISION_STATUS_CRC_INPUT_LEN   11u
#define VISION_1E4RAD_TO_RAD          0.0001f
#define VISION_RAD_TO_DEG             57.29577951f

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
static uint8_t latest_cmd_pending;
static uint8_t latest_cmd_seq_echo;
static uint8_t usb_host_ready;
static uint8_t vision_status_frame[VISION_STATUS_FRAME_LEN];
static DaemonInstance *vision_daemon_instance; // 视觉命令在线监测

volatile VisionDebug_t vision_debug;

static void VisionComm_DebugCopyBytes(volatile uint8_t *dst, const uint8_t *src, uint8_t len)
{
    for (uint8_t i = 0u; i < len; ++i)
    {
        dst[i] = src[i];
    }
}

static uint16_t VisionComm_DaemonReloadCount(void)
{
    uint32_t count = (VISION_CMD_TIMEOUT_MS + DAEMON_TASK_PERIOD_MS - 1u) / DAEMON_TASK_PERIOD_MS;
    return (uint16_t)((count == 0u) ? 1u : count);
}

static int16_t VisionComm_GetI16Le(const uint8_t *data)
{
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (int16_t)value;
}

static void VisionComm_PutI32Le(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw & 0x000000FFu);
    data[1] = (uint8_t)((raw >> 8) & 0x000000FFu);
    data[2] = (uint8_t)((raw >> 16) & 0x000000FFu);
    data[3] = (uint8_t)((raw >> 24) & 0x000000FFu);
}

static void VisionComm_ResetParser(void)
{
    memset(&vision_rx_parser, 0, sizeof(vision_rx_parser));
    vision_rx_parser.state = RX_WAIT_SOF1;
}

static void VisionComm_HandleFrame(const uint8_t frame[VISION_CMD_FRAME_LEN])
{
    latest_vision_cmd.seq = frame[2];
    latest_vision_cmd.target_valid = (frame[3] != 0u) ? 1u : 0u;
    latest_vision_cmd.delta_yaw_1e4rad = VisionComm_GetI16Le(&frame[4]);
    latest_vision_cmd.delta_pitch_1e4rad = VisionComm_GetI16Le(&frame[6]);
    latest_cmd_valid = 1u;
    latest_cmd_pending = 1u;
    vision_debug.valid_frame_count++;
    vision_debug.last_valid_tick_ms = HAL_GetTick();
    vision_debug.last_seq = latest_vision_cmd.seq;
    vision_debug.last_target_valid = latest_vision_cmd.target_valid;
    vision_debug.last_delta_yaw_1e4rad = latest_vision_cmd.delta_yaw_1e4rad;
    vision_debug.last_delta_pitch_1e4rad = latest_vision_cmd.delta_pitch_1e4rad;
    vision_debug.last_delta_yaw_rad =
        VISION_1E4RAD_TO_RAD * (float)latest_vision_cmd.delta_yaw_1e4rad;
    vision_debug.last_delta_pitch_rad =
        VISION_1E4RAD_TO_RAD * (float)latest_vision_cmd.delta_pitch_1e4rad;
    vision_debug.last_delta_yaw_deg = VISION_RAD_TO_DEG * vision_debug.last_delta_yaw_rad;
    vision_debug.last_delta_pitch_deg = VISION_RAD_TO_DEG * vision_debug.last_delta_pitch_rad;
    vision_debug.latest_cmd_valid = latest_cmd_valid;
    vision_debug.latest_cmd_pending = latest_cmd_pending;
    VisionComm_DebugCopyBytes(vision_debug.last_valid_frame, frame, VISION_CMD_FRAME_LEN);
    // 收到一帧合法视觉命令即认为视觉链路仍在线
    DaemonReload(vision_daemon_instance);
}

static uint8_t VisionComm_TxStatusFrame(void)
{
    uint16_t crc;

    vision_status_frame[0] = VISION_STATUS_SOF1;
    vision_status_frame[1] = VISION_STATUS_SOF2;
    vision_status_frame[2] = latest_cmd_seq_echo;
    VisionComm_PutI32Le(&vision_status_frame[3], latest_vision_status.yaw_actual_1e4rad);
    VisionComm_PutI32Le(&vision_status_frame[7], latest_vision_status.pitch_actual_1e4rad);
    crc = crc_modbus(vision_status_frame, VISION_STATUS_CRC_INPUT_LEN);
    vision_status_frame[11] = (uint8_t)(crc & 0x00FFu);
    vision_status_frame[12] = (uint8_t)((crc >> 8) & 0x00FFu);
    vision_debug.seq_echo = latest_cmd_seq_echo;
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
            recv_crc = (uint16_t)vision_rx_parser.frame[8] |
                       ((uint16_t)vision_rx_parser.frame[9] << 8);
            vision_debug.last_calc_crc = calc_crc;
            vision_debug.last_recv_crc = recv_crc;
            VisionComm_DebugCopyBytes(vision_debug.last_candidate_frame,
                                      vision_rx_parser.frame,
                                      VISION_CMD_FRAME_LEN);
            if (calc_crc == recv_crc)
            {
                VisionComm_HandleFrame(vision_rx_parser.frame);
            }
            else
            {
                vision_debug.crc_error_count++;
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
    latest_cmd_pending = 0u;
    latest_cmd_seq_echo = 0u;
    usb_host_ready = 0u;
    memset((void *)&vision_debug, 0, sizeof(vision_debug));
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
    vision_debug.usb_rx_packet_count++;
    vision_debug.usb_rx_byte_count += len;
    vision_debug.last_rx_tick_ms = HAL_GetTick();
    vision_debug.last_usb_packet_len = (uint8_t)((len > VISION_CMD_FRAME_LEN) ? VISION_CMD_FRAME_LEN : len);
    VisionComm_DebugCopyBytes(vision_debug.last_usb_packet_bytes,
                              data,
                              vision_debug.last_usb_packet_len);
    for (uint16_t i = 0; i < len; ++i)
    {
        VisionComm_ParseByte(data[i]);
    }
}

uint8_t VisionComm_GetVisionCmd(VisionCmd_t *cmd)
{
#if VISION_CONTROL_MODE == VISION_CONTROL_EVENT_TARGET
    if (!latest_cmd_pending)
    {
        return 0u;
    }

    memcpy(cmd, &latest_vision_cmd, sizeof(VisionCmd_t));
    latest_cmd_pending = 0u;
    latest_cmd_seq_echo = latest_vision_cmd.seq;
    vision_debug.seq_echo = latest_cmd_seq_echo;
    vision_debug.latest_cmd_pending = latest_cmd_pending;
    return 1u;
#else
    if (!latest_cmd_valid || !VisionComm_IsOnline())
    {
        return 0u;
    }

    memcpy(cmd, &latest_vision_cmd, sizeof(VisionCmd_t));
    latest_cmd_seq_echo = latest_vision_cmd.seq;
    vision_debug.seq_echo = latest_cmd_seq_echo;
    return 1u;
#endif
}

void VisionComm_UpdateStatus(const VisionStatus_t *status)
{
    memcpy(&latest_vision_status, status, sizeof(latest_vision_status));
    vision_debug.actual_yaw_1e4rad = status->yaw_actual_1e4rad;
    vision_debug.actual_pitch_1e4rad = status->pitch_actual_1e4rad;
    vision_debug.actual_yaw_rad = VISION_1E4RAD_TO_RAD * (float)status->yaw_actual_1e4rad;
    vision_debug.actual_pitch_rad = VISION_1E4RAD_TO_RAD * (float)status->pitch_actual_1e4rad;
    vision_debug.actual_yaw_deg = VISION_RAD_TO_DEG * vision_debug.actual_yaw_rad;
    vision_debug.actual_pitch_deg = VISION_RAD_TO_DEG * vision_debug.actual_pitch_rad;
}

uint8_t VisionComm_IsOnline(void)
{
    vision_debug.latest_cmd_valid = latest_cmd_valid;
    vision_debug.latest_cmd_pending = latest_cmd_pending;
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
