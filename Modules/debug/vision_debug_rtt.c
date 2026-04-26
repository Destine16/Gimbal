#include "vision_debug_rtt.h"

#include <stdint.h>
#include <string.h>

#include "crc16.h"
#include "gm6020.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "rtt_backend.h"
#include "stm32f4xx_hal.h"
#include "vision_comm.h"

#if VISION_DEBUG_RTT_ENABLE
#include "SEGGER_RTT.h"
#endif

#define VISION_DEBUG_RTT_UP_BUFFER_INDEX 2u
#define VISION_DEBUG_RTT_UP_BUFFER_SIZE  8192u
#define VISION_DEBUG_RTT_SOF1            0xA7u
#define VISION_DEBUG_RTT_SOF2            0x7Au
#define VISION_DEBUG_RTT_VERSION         1u
#define VISION_DEBUG_RTT_FRAME_LEN       189u
#define VISION_DEBUG_RTT_CRC_INPUT_LEN   187u

volatile VisionDebugRttDebug_s vision_debug_rtt_debug;

#if VISION_DEBUG_RTT_ENABLE
static uint8_t vision_debug_rtt_frame[VISION_DEBUG_RTT_FRAME_LEN];
static uint8_t vision_debug_rtt_up_buffer[VISION_DEBUG_RTT_UP_BUFFER_SIZE];
static uint32_t vision_debug_rtt_latest_tx_tick;
static uint32_t vision_debug_rtt_seq;
static uint8_t vision_debug_rtt_ready;

static void VisionDebugRtt_PutU8(uint8_t *frame, uint16_t *index, uint8_t value)
{
    frame[(*index)++] = value;
}

static void VisionDebugRtt_PutU16(uint8_t *frame, uint16_t *index, uint16_t value)
{
    frame[(*index)++] = (uint8_t)(value & 0x00FFu);
    frame[(*index)++] = (uint8_t)((value >> 8) & 0x00FFu);
}

static void VisionDebugRtt_PutI16(uint8_t *frame, uint16_t *index, int16_t value)
{
    VisionDebugRtt_PutU16(frame, index, (uint16_t)value);
}

static void VisionDebugRtt_PutU32(uint8_t *frame, uint16_t *index, uint32_t value)
{
    frame[(*index)++] = (uint8_t)(value & 0x000000FFu);
    frame[(*index)++] = (uint8_t)((value >> 8) & 0x000000FFu);
    frame[(*index)++] = (uint8_t)((value >> 16) & 0x000000FFu);
    frame[(*index)++] = (uint8_t)((value >> 24) & 0x000000FFu);
}

static void VisionDebugRtt_PutI32(uint8_t *frame, uint16_t *index, int32_t value)
{
    VisionDebugRtt_PutU32(frame, index, (uint32_t)value);
}

static int32_t VisionDebugRtt_ScaleI32(float value, float scale)
{
    const float max_scaled = 2147483647.0f;
    const float min_scaled = -2147483648.0f;
    float scaled = value * scale;

    if (scaled >= max_scaled)
    {
        return INT32_MAX;
    }
    if (scaled <= min_scaled)
    {
        return INT32_MIN;
    }
    return (int32_t)scaled;
}

static uint32_t VisionDebugRtt_AgeMs(uint32_t now_tick, uint32_t count, uint32_t event_tick)
{
    return (count == 0u) ? 0u : (now_tick - event_tick);
}

static void VisionDebugRtt_PutMotorSnapshot(uint8_t *frame,
                                            uint16_t *index,
                                            const GM6020_ControlSnapshot_s *snapshot)
{
    if ((frame == NULL) || (index == NULL) || (snapshot == NULL))
    {
        return;
    }

    VisionDebugRtt_PutU8(frame, index, snapshot->valid);
    VisionDebugRtt_PutU8(frame, index, snapshot->enabled);
    VisionDebugRtt_PutU8(frame, index, snapshot->online);
    VisionDebugRtt_PutU8(frame, index, snapshot->motor_id);
    VisionDebugRtt_PutI32(frame, index, VisionDebugRtt_ScaleI32(snapshot->angle_ref_rad, 1000000.0f));
    VisionDebugRtt_PutI32(frame, index, VisionDebugRtt_ScaleI32(snapshot->angle_feedback_rad, 1000000.0f));
    VisionDebugRtt_PutI32(frame, index, VisionDebugRtt_ScaleI32(snapshot->speed_ref_rad_s, 1000000.0f));
    VisionDebugRtt_PutI32(frame, index, VisionDebugRtt_ScaleI32(snapshot->speed_feedback_rad_s, 1000000.0f));
    VisionDebugRtt_PutI32(frame, index, VisionDebugRtt_ScaleI32(snapshot->current_ref_raw, 1000.0f));
    VisionDebugRtt_PutI32(frame, index, VisionDebugRtt_ScaleI32(snapshot->current_feedback_raw, 1000.0f));
    VisionDebugRtt_PutI32(frame, index, VisionDebugRtt_ScaleI32(snapshot->voltage_ref_raw, 1000.0f));
    VisionDebugRtt_PutI32(frame, index, VisionDebugRtt_ScaleI32(snapshot->output_ff_raw, 1000.0f));
    VisionDebugRtt_PutI16(frame, index, snapshot->output_cmd);
    VisionDebugRtt_PutI16(frame, index, snapshot->real_current);
}

static uint8_t VisionDebugRtt_BuildFrame(uint8_t *frame, uint16_t frame_len)
{
    uint16_t idx = 0u;
    uint16_t crc;
    uint32_t now_tick = HAL_GetTick();
    GM6020_ControlSnapshot_s yaw_snapshot;
    GM6020_ControlSnapshot_s pitch_snapshot;

    if ((frame == NULL) || (frame_len < VISION_DEBUG_RTT_FRAME_LEN))
    {
        return 0u;
    }

    (void)GM6020_GetControlSnapshot(GIMBAL_YAW_MOTOR_ID, &yaw_snapshot);
    (void)GM6020_GetControlSnapshot(GIMBAL_PITCH_MOTOR_ID, &pitch_snapshot);

    VisionDebugRtt_PutU8(frame, &idx, VISION_DEBUG_RTT_SOF1);
    VisionDebugRtt_PutU8(frame, &idx, VISION_DEBUG_RTT_SOF2);
    VisionDebugRtt_PutU8(frame, &idx, VISION_DEBUG_RTT_VERSION);
    VisionDebugRtt_PutU8(frame, &idx, VISION_DEBUG_RTT_FRAME_LEN);

    VisionDebugRtt_PutU32(frame, &idx, now_tick);
    VisionDebugRtt_PutU32(frame, &idx, vision_debug_rtt_seq++);
    VisionDebugRtt_PutU32(frame, &idx, vision_debug.usb_rx_packet_count);
    VisionDebugRtt_PutU32(frame, &idx, vision_debug.usb_rx_byte_count);
    VisionDebugRtt_PutU32(frame, &idx, vision_debug.valid_frame_count);
    VisionDebugRtt_PutU32(frame, &idx, vision_debug.crc_error_count);
    VisionDebugRtt_PutU32(frame, &idx, VisionDebugRtt_AgeMs(now_tick,
                                                           vision_debug.usb_rx_packet_count,
                                                           vision_debug.last_rx_tick_ms));
    VisionDebugRtt_PutU32(frame, &idx, VisionDebugRtt_AgeMs(now_tick,
                                                           vision_debug.valid_frame_count,
                                                           vision_debug.last_valid_tick_ms));
    VisionDebugRtt_PutU8(frame, &idx, vision_debug.last_seq);
    VisionDebugRtt_PutU8(frame, &idx, vision_debug.seq_echo);
    VisionDebugRtt_PutU8(frame, &idx, vision_debug.last_target_valid);

    VisionDebugRtt_PutI32(frame, &idx, VisionDebugRtt_ScaleI32(vision_debug.last_delta_yaw_rad, 1000000.0f));
    VisionDebugRtt_PutI32(frame, &idx, VisionDebugRtt_ScaleI32(vision_debug.last_delta_pitch_rad, 1000000.0f));
    VisionDebugRtt_PutI32(frame, &idx, VisionDebugRtt_ScaleI32(vision_debug.actual_yaw_rad, 1000000.0f));
    VisionDebugRtt_PutI32(frame, &idx, VisionDebugRtt_ScaleI32(vision_debug.actual_pitch_rad, 1000000.0f));

    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.robot_state);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.gimbal_ready);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.gimbal_mode);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.vision_target_valid);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.vision_cmd_ready);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.vision_cmd_target_valid);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.sentry_state);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.stall_axis);
    VisionDebugRtt_PutI32(frame, &idx, VisionDebugRtt_ScaleI32(robot_cmd_debug.vision_target_yaw_rad, 1000000.0f));
    VisionDebugRtt_PutI32(frame, &idx, VisionDebugRtt_ScaleI32(robot_cmd_debug.vision_target_pitch_rad, 1000000.0f));
    VisionDebugRtt_PutI32(frame, &idx, VisionDebugRtt_ScaleI32(robot_cmd_debug.cmd_yaw_rad, 1000000.0f));
    VisionDebugRtt_PutI32(frame, &idx, VisionDebugRtt_ScaleI32(robot_cmd_debug.cmd_pitch_rad, 1000000.0f));

    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.imu_online);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.yaw_motor_online);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.pitch_motor_online);
    VisionDebugRtt_PutU8(frame, &idx, robot_cmd_debug.stall_detected);

    VisionDebugRtt_PutU32(frame, &idx, gm6020_debug.tx_attempt_count);
    VisionDebugRtt_PutU32(frame, &idx, gm6020_debug.tx_success_count);
    VisionDebugRtt_PutU32(frame, &idx, gm6020_debug.tx_fail_count);
    VisionDebugRtt_PutU32(frame, &idx, gm6020_debug.tx_abort_count);
    VisionDebugRtt_PutU32(frame, &idx, gm6020_debug.last_hal_error);
    VisionDebugRtt_PutU32(frame, &idx, gm6020_debug.last_can_error_code);

    VisionDebugRtt_PutMotorSnapshot(frame, &idx, &yaw_snapshot);
    VisionDebugRtt_PutMotorSnapshot(frame, &idx, &pitch_snapshot);

    crc = crc_modbus(frame, VISION_DEBUG_RTT_CRC_INPUT_LEN);
    VisionDebugRtt_PutU16(frame, &idx, crc);
    return (uint8_t)(idx == VISION_DEBUG_RTT_FRAME_LEN);
}
#endif

void VisionDebugRtt_Init(void)
{
    memset((void *)&vision_debug_rtt_debug, 0, sizeof(vision_debug_rtt_debug));

#if VISION_DEBUG_RTT_ENABLE
    RttBackend_Init();
    SEGGER_RTT_ConfigUpBuffer(VISION_DEBUG_RTT_UP_BUFFER_INDEX,
                              "vision_dbg",
                              vision_debug_rtt_up_buffer,
                              sizeof(vision_debug_rtt_up_buffer),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    vision_debug_rtt_latest_tx_tick = 0u;
    vision_debug_rtt_seq = 0u;
    vision_debug_rtt_ready = 1u;
    vision_debug_rtt_debug.init_count++;
#endif
}

void VisionDebugRtt_Task(void)
{
#if VISION_DEBUG_RTT_ENABLE
    uint32_t now_tick = HAL_GetTick();
    uint32_t avail;
    unsigned bytes_written;

    if (!vision_debug_rtt_ready)
    {
        return;
    }

    if ((now_tick - vision_debug_rtt_latest_tx_tick) < VISION_DEBUG_RTT_TX_PERIOD_MS)
    {
        return;
    }

    vision_debug_rtt_debug.tx_attempt_count++;
    if (!VisionDebugRtt_BuildFrame(vision_debug_rtt_frame, sizeof(vision_debug_rtt_frame)))
    {
        vision_debug_rtt_debug.build_fail_count++;
        vision_debug_rtt_latest_tx_tick = now_tick;
        return;
    }

    avail = SEGGER_RTT_GetAvailWriteSpace(VISION_DEBUG_RTT_UP_BUFFER_INDEX);
    vision_debug_rtt_debug.last_avail_write_space = avail;
    if (avail < VISION_DEBUG_RTT_FRAME_LEN)
    {
        vision_debug_rtt_debug.tx_skip_count++;
        vision_debug_rtt_latest_tx_tick = now_tick;
        return;
    }

    bytes_written = SEGGER_RTT_WriteNoLock(VISION_DEBUG_RTT_UP_BUFFER_INDEX,
                                           vision_debug_rtt_frame,
                                           VISION_DEBUG_RTT_FRAME_LEN);
    vision_debug_rtt_debug.last_bytes_written = bytes_written;
    if (bytes_written == VISION_DEBUG_RTT_FRAME_LEN)
    {
        vision_debug_rtt_debug.tx_success_count++;
        vision_debug_rtt_debug.last_tx_tick_ms = now_tick;
    }
    else
    {
        vision_debug_rtt_debug.tx_skip_count++;
    }
    vision_debug_rtt_latest_tx_tick = now_tick;
#endif
}
