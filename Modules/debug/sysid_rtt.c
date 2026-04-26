#include "sysid_rtt.h"

#include <string.h>

#include "gimbal_sysid_telemetry.h"
#include "robot_def.h"
#include "rtt_backend.h"
#include "stm32f4xx_hal.h"

#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
#include "SEGGER_RTT.h"
#endif

#define SYSID_RTT_UP_BUFFER_INDEX 1u
#define SYSID_RTT_UP_BUFFER_SIZE  8192u

volatile SysIdRttDebug_s sysid_rtt_debug;

#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
static uint8_t sysid_rtt_frame[GIMBAL_SYSID_TELEMETRY_FRAME_LEN];
static uint8_t sysid_rtt_up_buffer[SYSID_RTT_UP_BUFFER_SIZE];
static uint32_t sysid_rtt_latest_tx_tick;
static uint8_t sysid_rtt_ready;
#endif

void SysIdRtt_Init(void)
{
    memset((void *)&sysid_rtt_debug, 0, sizeof(sysid_rtt_debug));

#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
    RttBackend_Init();
    SEGGER_RTT_ConfigUpBuffer(SYSID_RTT_UP_BUFFER_INDEX,
                              "sysid",
                              sysid_rtt_up_buffer,
                              sizeof(sysid_rtt_up_buffer),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    sysid_rtt_latest_tx_tick = 0u;
    sysid_rtt_ready = 1u;
    sysid_rtt_debug.init_count++;
#endif
}

void SysIdRtt_Task(void)
{
#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
    uint32_t now_tick = HAL_GetTick();
    uint32_t avail;
    unsigned bytes_written;

    if (!sysid_rtt_ready)
    {
        return;
    }

    if ((now_tick - sysid_rtt_latest_tx_tick) < GIMBAL_SYSID_TELEMETRY_TX_PERIOD_MS)
    {
        return;
    }

    sysid_rtt_debug.tx_attempt_count++;
    if (!GimbalSysIdTelemetry_BuildFrame(sysid_rtt_frame, sizeof(sysid_rtt_frame)))
    {
        sysid_rtt_debug.build_fail_count++;
        sysid_rtt_latest_tx_tick = now_tick;
        return;
    }

    avail = SEGGER_RTT_GetAvailWriteSpace(SYSID_RTT_UP_BUFFER_INDEX);
    sysid_rtt_debug.last_avail_write_space = avail;
    if (avail < GIMBAL_SYSID_TELEMETRY_FRAME_LEN)
    {
        sysid_rtt_debug.tx_skip_count++;
        sysid_rtt_latest_tx_tick = now_tick;
        return;
    }

    bytes_written = SEGGER_RTT_WriteNoLock(SYSID_RTT_UP_BUFFER_INDEX,
                                           sysid_rtt_frame,
                                           GIMBAL_SYSID_TELEMETRY_FRAME_LEN);
    sysid_rtt_debug.last_bytes_written = bytes_written;
    if (bytes_written == GIMBAL_SYSID_TELEMETRY_FRAME_LEN)
    {
        sysid_rtt_debug.tx_success_count++;
        sysid_rtt_debug.last_tx_tick_ms = now_tick;
    }
    else
    {
        sysid_rtt_debug.tx_skip_count++;
    }
    sysid_rtt_latest_tx_tick = now_tick;
#endif
}
