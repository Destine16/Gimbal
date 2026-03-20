#include "vision_comm.h"

#include <string.h>

#include "crc16.h"
#include "daemon.h"
#include "main.h"
#include "robot_def.h"
#include "usbd_cdc_if.h"

#define VISION_SOF1               0xA5u
#define VISION_SOF2               0x5Au
#define VISION_PKT_CMD            0x01u
#define VISION_PKT_STATUS         0x02u
#define VISION_MAX_PAYLOAD_LEN    32u
#define VISION_FRAME_OVERHEAD      6u

typedef enum
{
    RX_WAIT_SOF1 = 0,
    RX_WAIT_SOF2,
    RX_WAIT_TYPE,
    RX_WAIT_LEN,
    RX_WAIT_PAYLOAD,
    RX_WAIT_CRC1,
    RX_WAIT_CRC2,
} VisionRxState_e;

typedef struct
{
    VisionRxState_e state;
    uint8_t type;
    uint8_t len;
    uint8_t payload[VISION_MAX_PAYLOAD_LEN];
    uint8_t payload_index;
    uint8_t crc_lsb;
} VisionRxParser_t;

static VisionRxParser_t vision_rx_parser;
static VisionCmd_t latest_vision_cmd;
static ControlStatus_t latest_control_status;
static uint32_t latest_status_tx_tick;
static uint8_t latest_cmd_valid;
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

static void VisionComm_HandleFrame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    if ((type == VISION_PKT_CMD) && (len == sizeof(VisionCmd_t)))
    {
        memcpy(&latest_vision_cmd, payload, sizeof(VisionCmd_t));
        latest_cmd_valid = 1u;
        // 收到一帧合法视觉命令即认为视觉链路仍在线
        DaemonReload(vision_daemon_instance);
    }
}

static void VisionComm_ParseByte(uint8_t byte)
{
    uint16_t calc_crc;
    uint16_t recv_crc;
    uint8_t crc_input[2 + VISION_MAX_PAYLOAD_LEN];

    switch (vision_rx_parser.state)
    {
    case RX_WAIT_SOF1:
        if (byte == VISION_SOF1)
        {
            vision_rx_parser.state = RX_WAIT_SOF2;
        }
        break;

    case RX_WAIT_SOF2:
        if (byte == VISION_SOF2)
        {
            vision_rx_parser.state = RX_WAIT_TYPE;
        }
        else
        {
            VisionComm_ResetParser();
        }
        break;

    case RX_WAIT_TYPE:
        vision_rx_parser.type = byte;
        vision_rx_parser.state = RX_WAIT_LEN;
        break;

    case RX_WAIT_LEN:
        if (byte > VISION_MAX_PAYLOAD_LEN)
        {
            VisionComm_ResetParser();
            break;
        }
        vision_rx_parser.len = byte;
        vision_rx_parser.payload_index = 0u;
        vision_rx_parser.state = (byte == 0u) ? RX_WAIT_CRC1 : RX_WAIT_PAYLOAD;
        break;

    case RX_WAIT_PAYLOAD:
        vision_rx_parser.payload[vision_rx_parser.payload_index++] = byte;
        if (vision_rx_parser.payload_index >= vision_rx_parser.len)
        {
            vision_rx_parser.state = RX_WAIT_CRC1;
        }
        break;

    case RX_WAIT_CRC1:
        vision_rx_parser.crc_lsb = byte;
        vision_rx_parser.state = RX_WAIT_CRC2;
        break;

    case RX_WAIT_CRC2:
        crc_input[0] = vision_rx_parser.type;
        crc_input[1] = vision_rx_parser.len;
        memcpy(&crc_input[2], vision_rx_parser.payload, vision_rx_parser.len);
        calc_crc = crc_modbus(crc_input, (uint16_t)(2u + vision_rx_parser.len));
        recv_crc = (uint16_t)vision_rx_parser.crc_lsb | ((uint16_t)byte << 8);
        if (calc_crc == recv_crc)
        {
            VisionComm_HandleFrame(vision_rx_parser.type,
                                   vision_rx_parser.payload,
                                   vision_rx_parser.len);
        }
        VisionComm_ResetParser();
        break;

    default:
        VisionComm_ResetParser();
        break;
    }
}

static uint8_t VisionComm_TxFrame(uint8_t type, const void *payload, uint8_t len)
{
    uint16_t crc;
    uint8_t frame[VISION_MAX_PAYLOAD_LEN + VISION_FRAME_OVERHEAD];

    frame[0] = VISION_SOF1;
    frame[1] = VISION_SOF2;
    frame[2] = type;
    frame[3] = len;
    memcpy(&frame[4], payload, len);
    crc = crc_modbus(&frame[2], (uint16_t)(2u + len));
    frame[4 + len] = (uint8_t)(crc & 0x00FFu);
    frame[5 + len] = (uint8_t)((crc >> 8) & 0x00FFu);

    return (uint8_t)(CDC_Transmit_FS(frame, (uint16_t)(len + VISION_FRAME_OVERHEAD)) == USBD_OK);
}

void VisionComm_Init(void)
{
    memset(&latest_vision_cmd, 0, sizeof(latest_vision_cmd));
    memset(&latest_control_status, 0, sizeof(latest_control_status));
    latest_status_tx_tick = 0u;
    latest_cmd_valid = 0u;
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
    uint32_t now = HAL_GetTick();

    if (now - latest_status_tx_tick >= VISION_STATUS_TX_PERIOD_MS)
    {
        if (VisionComm_TxFrame(VISION_PKT_STATUS,
                               &latest_control_status,
                               (uint8_t)sizeof(ControlStatus_t)))
        {
            latest_status_tx_tick = now;
        }
    }
}

void VisionComm_RxBytes(const uint8_t *data, uint16_t len)
{
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

void VisionComm_UpdateControlStatus(const ControlStatus_t *status)
{
    memcpy(&latest_control_status, status, sizeof(ControlStatus_t));
}

uint8_t VisionComm_IsOnline(void)
{
    return (uint8_t)(latest_cmd_valid && DaemonIsOnline(vision_daemon_instance));
}
