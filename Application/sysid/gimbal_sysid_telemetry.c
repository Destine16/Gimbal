#include "gimbal_sysid_telemetry.h"

#include "crc16.h"
#include "gimbal_sysid.h"
#include "gm6020.h"
#include "robot_def.h"
#include "stm32f4xx_hal.h"

#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
static void GimbalSysIdTelemetry_PutU16(uint8_t *frame, uint8_t *index, uint16_t value)
{
    frame[(*index)++] = (uint8_t)(value & 0x00FFu);
    frame[(*index)++] = (uint8_t)((value >> 8) & 0x00FFu);
}

static void GimbalSysIdTelemetry_PutI16(uint8_t *frame, uint8_t *index, int16_t value)
{
    GimbalSysIdTelemetry_PutU16(frame, index, (uint16_t)value);
}

static void GimbalSysIdTelemetry_PutU32(uint8_t *frame, uint8_t *index, uint32_t value)
{
    frame[(*index)++] = (uint8_t)(value & 0x000000FFu);
    frame[(*index)++] = (uint8_t)((value >> 8) & 0x000000FFu);
    frame[(*index)++] = (uint8_t)((value >> 16) & 0x000000FFu);
    frame[(*index)++] = (uint8_t)((value >> 24) & 0x000000FFu);
}

static void GimbalSysIdTelemetry_PutI32(uint8_t *frame, uint8_t *index, int32_t value)
{
    GimbalSysIdTelemetry_PutU32(frame, index, (uint32_t)value);
}

static int32_t GimbalSysIdTelemetry_ScaleI32(float value, float scale)
{
    return (int32_t)(value * scale);
}
#endif

#if (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_HYST) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_STEP) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_SINE) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PRBS) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FAST_MULTISINE) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF_SWEEP) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_STATIC_FF_MAP)
static uint8_t GimbalSysIdTelemetry_GetSnapshot(GM6020_ControlSnapshot_s *snapshot)
{
    return GM6020_GetControlSnapshot(GIMBAL_PITCH_MOTOR_ID, snapshot);
}
#elif GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
static uint8_t GimbalSysIdTelemetry_GetSnapshot(GM6020_ControlSnapshot_s *snapshot)
{
    return GM6020_GetControlSnapshot(GIMBAL_YAW_MOTOR_ID, snapshot);
}
#else
#endif

uint8_t GimbalSysIdTelemetry_BuildFrame(uint8_t *frame, uint16_t frame_len)
{
#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
    uint8_t idx = 0u;
    uint16_t crc;
    GM6020_ControlSnapshot_s snapshot;

    if ((frame == NULL) || (frame_len < GIMBAL_SYSID_TELEMETRY_FRAME_LEN))
    {
        return 0u;
    }

    if (!GimbalSysIdTelemetry_GetSnapshot(&snapshot))
    {
        return 0u;
    }

    frame[idx++] = GIMBAL_SYSID_TELEMETRY_SOF1;
    frame[idx++] = GIMBAL_SYSID_TELEMETRY_SOF2;
    GimbalSysIdTelemetry_PutU32(frame, &idx, HAL_GetTick());
    GimbalSysIdTelemetry_PutU16(frame, &idx, gimbal_sysid_debug.seq_index);
    frame[idx++] = gimbal_sysid_debug.mode;
    frame[idx++] = gimbal_sysid_debug.phase;
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.angle_ref_rad, 1000000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.angle_feedback_rad, 1000000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.speed_ref_rad_s, 1000000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.speed_feedback_rad_s, 1000000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.current_ref_raw, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.current_feedback_raw, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.voltage_ref_raw, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.output_ff_raw, 1000.0f));
    GimbalSysIdTelemetry_PutI16(frame, &idx, snapshot.output_cmd);
    GimbalSysIdTelemetry_PutI16(frame, &idx, snapshot.real_current);
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.motor_speed_rad_s, 1000000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.angle_pid_pout, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.angle_pid_iout, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.angle_pid_dout, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.angle_pid_output, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.speed_pid_pout, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.speed_pid_iout, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.speed_pid_dout, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.speed_pid_output, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.current_pid_pout, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.current_pid_iout, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.current_pid_dout, 1000.0f));
    GimbalSysIdTelemetry_PutI32(frame, &idx, GimbalSysIdTelemetry_ScaleI32(snapshot.current_pid_output, 1000.0f));
    crc = crc_modbus(frame, GIMBAL_SYSID_TELEMETRY_CRC_INPUT_LEN);
    GimbalSysIdTelemetry_PutU16(frame, &idx, crc);
    return (uint8_t)(idx == GIMBAL_SYSID_TELEMETRY_FRAME_LEN);
#else
    (void)frame;
    (void)frame_len;
    return 0u;
#endif
}
