#include "gimbal_sysid.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "gimbal.h"
#include "general_def.h"
#include "stm32f4xx_hal.h"

#define GIMBAL_SYSID_BASELINE_MS       5000u
#define GIMBAL_SYSID_PRBS_MS          60000u
#define GIMBAL_SYSID_RETURN_MS        10000u
#define GIMBAL_SYSID_PITCH_FF_STAGE_MS 5000u
#define GIMBAL_SYSID_PITCH_HYST_STAGE_MS 4000u
#define GIMBAL_SYSID_YAW_STEP_BASELINE_MS 3000u
#define GIMBAL_SYSID_YAW_STEP_STAGE_MS    2000u
#define GIMBAL_SYSID_YAW_STEP_RETURN_MS   3000u
#define GIMBAL_SYSID_PERF_STEP_BASELINE_MS 3000u
#define GIMBAL_SYSID_PERF_STEP_STAGE_MS    2500u
#define GIMBAL_SYSID_PERF_STEP_RETURN_MS   3000u
#define GIMBAL_SYSID_PERF_SINE_BASELINE_MS 3000u
#define GIMBAL_SYSID_PERF_SINE_STAGE_MS    8000u
#define GIMBAL_SYSID_PERF_SINE_RETURN_MS   3000u
#define GIMBAL_SYSID_TOTAL_MS \
    (GIMBAL_SYSID_BASELINE_MS + GIMBAL_SYSID_PRBS_MS + GIMBAL_SYSID_RETURN_MS)
#define GIMBAL_SYSID_PRBS_SEED   0x4C6F6755u
#define GIMBAL_SYSID_TWO_PI      6.28318531f

typedef struct
{
    uint8_t started;
    uint8_t current_offset_index;
    uint32_t start_tick_ms;
    uint32_t stage_start_tick_ms;
    uint32_t stage_hold_ms;
    uint32_t rng_state;
    uint16_t seq_index;
    float base_yaw_rad;
    float base_pitch_rad;
    float current_offset_rad;
} GimbalSysIdState_s;

volatile GimbalSysIdDebug_s gimbal_sysid_debug;

#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
static GimbalSysIdState_s sysid_state;
#endif

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS
static const float yaw_prbs_offsets_rad[] = {
    -0.87266463f, // -50 deg
    -0.61086524f, // -35 deg
    -0.38397244f, // -22 deg
    -0.20943951f, // -12 deg
    0.0f,
    0.20943951f,  // +12 deg
    0.38397244f,  // +22 deg
    0.61086524f,  // +35 deg
    0.87266463f,  // +50 deg
};
#endif

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_STEP
static const float yaw_step_offsets_rad[] = {
    0.0f,
    0.17453293f,  // +10 deg
    0.0f,
    -0.17453293f, // -10 deg
    0.0f,
};
#endif

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PERF_STEP
static const float perf_step_offsets_rad[] = {
    0.0f,
    0.05235988f, // +3 deg
    0.0f,
    0.08726646f, // +5 deg
    0.0f,
    0.17453293f, // +10 deg
    0.0f,
    0.34906585f, // +20 deg
    0.0f,
};
#endif

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_STEP
static const float perf_step_offsets_rad[] = {
    0.0f,
    0.05235988f,  // +3 deg
    0.0f,
    -0.05235988f, // -3 deg
    0.0f,
    -0.17453293f, // -10 deg
    0.0f,
    0.17453293f,  // +10 deg
    0.0f,
};
#endif

#if (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PERF_SINE) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_SINE)
typedef struct
{
    float amplitude_rad;
    float period_s;
} GimbalSysIdSineCase_s;

static const GimbalSysIdSineCase_s perf_sine_cases[] = {
    {0.08726646f, 1.0f}, // A=5 deg, T=1 s
    {0.34906585f, 2.0f}, // A=20 deg, T=2 s
};
#endif

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF
static const float pitch_ff_offsets_rad[] = {
    0.0f,
    0.17453293f,  // +10 deg
    0.0f,
    -0.17453293f, // -10 deg
    0.0f,
    0.34906585f,  // +20 deg
    0.0f,
    -0.34906585f, // -20 deg
    0.0f,
    0.52359878f,  // +30 deg
    0.0f,
    -0.52359878f, // -30 deg
    0.0f,
    0.66322512f,  // +38 deg
    0.0f,
    -0.66322512f, // -38 deg
    0.0f,
};
#endif

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_HYST
static const float pitch_hyst_offsets_rad[] = {
    0.0f,
    0.17453293f,  // +10 deg
    0.34906585f,  // +20 deg
    0.52359878f,  // +30 deg
    0.66322512f,  // +38 deg
    0.52359878f,  // +30 deg
    0.34906585f,  // +20 deg
    0.17453293f,  // +10 deg
    0.0f,
    -0.17453293f, // -10 deg
    -0.34906585f, // -20 deg
    -0.52359878f, // -30 deg
    -0.66322512f, // -38 deg
    -0.52359878f, // -30 deg
    -0.34906585f, // -20 deg
    -0.17453293f, // -10 deg
    0.0f,
    0.17453293f,  // +10 deg
    0.34906585f,  // +20 deg
    0.52359878f,  // +30 deg
    0.66322512f,  // +38 deg
    0.52359878f,  // +30 deg
    0.34906585f,  // +20 deg
    0.17453293f,  // +10 deg
    0.0f,
};
#endif

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS
static uint32_t GimbalSysId_Rand(void)
{
    uint32_t x = sysid_state.rng_state;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    sysid_state.rng_state = (x == 0u) ? GIMBAL_SYSID_PRBS_SEED : x;
    return sysid_state.rng_state;
}

static uint32_t GimbalSysId_NextHoldMs(void)
{
    uint32_t bucket = GimbalSysId_Rand() % 20u;

    if (bucket < 5u)
    {
        return 250u;
    }
    if (bucket < 11u)
    {
        return 350u;
    }
    if (bucket < 17u)
    {
        return 500u;
    }
    return 800u;
}

static uint8_t GimbalSysId_NextOffsetIndex(void)
{
    const uint8_t count = (uint8_t)(sizeof(yaw_prbs_offsets_rad) / sizeof(yaw_prbs_offsets_rad[0]));
    uint8_t candidate;

    do
    {
        candidate = (uint8_t)(GimbalSysId_Rand() % count);
    } while ((candidate == sysid_state.current_offset_index) ||
             ((candidate == 0u) && (sysid_state.current_offset_index == (count - 1u))) ||
             ((candidate == (count - 1u)) && (sysid_state.current_offset_index == 0u)));

    return candidate;
}
#endif

#if (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_HYST) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_STEP) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_SINE)
static float GimbalSysId_ClampF(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}
#endif

#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
static void GimbalSysId_DebugUpdate(uint8_t phase, uint32_t elapsed_ms)
{
    gimbal_sysid_debug.mode = GIMBAL_SYSID_MODE;
    gimbal_sysid_debug.phase = phase;
    gimbal_sysid_debug.seq_index = sysid_state.seq_index;
    gimbal_sysid_debug.elapsed_ms = elapsed_ms;
    gimbal_sysid_debug.base_yaw_rad = sysid_state.base_yaw_rad;
    gimbal_sysid_debug.base_pitch_rad = sysid_state.base_pitch_rad;
    gimbal_sysid_debug.target_offset_rad = sysid_state.current_offset_rad;
#if (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_STEP) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PERF_STEP) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PERF_SINE)
    gimbal_sysid_debug.yaw_offset_rad = sysid_state.current_offset_rad;
    gimbal_sysid_debug.pitch_offset_rad = 0.0f;
#elif (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_HYST) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_STEP) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_SINE)
    gimbal_sysid_debug.yaw_offset_rad = 0.0f;
    gimbal_sysid_debug.pitch_offset_rad = sysid_state.current_offset_rad;
#else
    gimbal_sysid_debug.yaw_offset_rad = 0.0f;
    gimbal_sysid_debug.pitch_offset_rad = 0.0f;
#endif
}
#endif

void GimbalSysId_Init(void)
{
#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
    memset(&sysid_state, 0, sizeof(sysid_state));
    memset((void *)&gimbal_sysid_debug, 0, sizeof(gimbal_sysid_debug));
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS
    sysid_state.current_offset_index = 4u;
    sysid_state.rng_state = GIMBAL_SYSID_PRBS_SEED;
#endif
#else
    memset((void *)&gimbal_sysid_debug, 0, sizeof(gimbal_sysid_debug));
#endif
}

uint8_t GimbalSysId_Update(const Gimbal_Upload_Data_s *feedback, Gimbal_Ctrl_Cmd_s *cmd)
{
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS
    uint32_t now_tick_ms;
    uint32_t elapsed_ms;

    if ((feedback == NULL) || (cmd == NULL) || !feedback->imu_online)
    {
        GimbalSysId_Init();
        return 0u;
    }

    now_tick_ms = HAL_GetTick();
    if (!sysid_state.started)
    {
        sysid_state.started = 1u;
        sysid_state.start_tick_ms = now_tick_ms;
        sysid_state.stage_start_tick_ms = now_tick_ms;
        sysid_state.stage_hold_ms = 0u;
        sysid_state.seq_index = 0u;
        sysid_state.current_offset_index = 4u;
        sysid_state.current_offset_rad = 0.0f;
        sysid_state.rng_state = GIMBAL_SYSID_PRBS_SEED;
        sysid_state.base_yaw_rad = feedback->gimbal_imu_data.YawTotalAngle;
        sysid_state.base_pitch_rad = feedback->gimbal_imu_data.Pitch;
    }

    elapsed_ms = now_tick_ms - sysid_state.start_tick_ms;
    cmd->gimbal_mode = GIMBAL_IMU_MODE;
    cmd->pitch = sysid_state.base_pitch_rad;

    if (elapsed_ms < GIMBAL_SYSID_BASELINE_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_BASELINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_BASELINE_MS + GIMBAL_SYSID_PRBS_MS))
    {
        if ((sysid_state.stage_hold_ms == 0u) ||
            ((now_tick_ms - sysid_state.stage_start_tick_ms) >= sysid_state.stage_hold_ms))
        {
            sysid_state.current_offset_index = GimbalSysId_NextOffsetIndex();
            sysid_state.current_offset_rad = yaw_prbs_offsets_rad[sysid_state.current_offset_index];
            sysid_state.stage_hold_ms = GimbalSysId_NextHoldMs();
            sysid_state.stage_start_tick_ms = now_tick_ms;
            sysid_state.seq_index++;
        }

        cmd->yaw = sysid_state.base_yaw_rad + sysid_state.current_offset_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_PRBS, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < GIMBAL_SYSID_TOTAL_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_RETURN, elapsed_ms);
        return 1u;
    }

    sysid_state.current_offset_rad = 0.0f;
    cmd->yaw = sysid_state.base_yaw_rad;
    GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_DONE, elapsed_ms);
    return 1u;
#elif GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_STEP
    uint32_t now_tick_ms;
    uint32_t elapsed_ms;
    uint32_t step_ms;
    uint32_t stage_elapsed_ms;
    uint8_t stage_index;
    const uint8_t stage_count = (uint8_t)(sizeof(yaw_step_offsets_rad) / sizeof(yaw_step_offsets_rad[0]));

    if ((feedback == NULL) || (cmd == NULL) || !feedback->imu_online)
    {
        GimbalSysId_Init();
        return 0u;
    }

    now_tick_ms = HAL_GetTick();
    if (!sysid_state.started)
    {
        sysid_state.started = 1u;
        sysid_state.start_tick_ms = now_tick_ms;
        sysid_state.stage_start_tick_ms = now_tick_ms;
        sysid_state.stage_hold_ms = GIMBAL_SYSID_YAW_STEP_STAGE_MS;
        sysid_state.seq_index = 0u;
        sysid_state.current_offset_index = 0xFFu;
        sysid_state.current_offset_rad = 0.0f;
        sysid_state.base_yaw_rad = feedback->gimbal_imu_data.YawTotalAngle;
        sysid_state.base_pitch_rad = feedback->gimbal_imu_data.Pitch;
    }

    elapsed_ms = now_tick_ms - sysid_state.start_tick_ms;
    step_ms = (uint32_t)stage_count * GIMBAL_SYSID_YAW_STEP_STAGE_MS;
    cmd->gimbal_mode = GIMBAL_IMU_MODE;
    cmd->pitch = sysid_state.base_pitch_rad;

    if (elapsed_ms < GIMBAL_SYSID_YAW_STEP_BASELINE_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_BASELINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_YAW_STEP_BASELINE_MS + step_ms))
    {
        stage_elapsed_ms = elapsed_ms - GIMBAL_SYSID_YAW_STEP_BASELINE_MS;
        stage_index = (uint8_t)(stage_elapsed_ms / GIMBAL_SYSID_YAW_STEP_STAGE_MS);
        if (stage_index >= stage_count)
        {
            stage_index = (uint8_t)(stage_count - 1u);
        }

        if (stage_index != sysid_state.current_offset_index)
        {
            sysid_state.current_offset_index = stage_index;
            sysid_state.current_offset_rad = yaw_step_offsets_rad[stage_index];
            sysid_state.stage_start_tick_ms = now_tick_ms;
            sysid_state.seq_index++;
        }

        cmd->yaw = sysid_state.base_yaw_rad + sysid_state.current_offset_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_STEP, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_YAW_STEP_BASELINE_MS + step_ms + GIMBAL_SYSID_YAW_STEP_RETURN_MS))
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_RETURN, elapsed_ms);
        return 1u;
    }

    sysid_state.current_offset_rad = 0.0f;
    cmd->yaw = sysid_state.base_yaw_rad;
    GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_DONE, elapsed_ms);
    return 1u;
#elif (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PERF_STEP) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_STEP)
    uint32_t now_tick_ms;
    uint32_t elapsed_ms;
    uint32_t step_ms;
    uint32_t stage_elapsed_ms;
    uint8_t stage_index;
    const uint8_t stage_count = (uint8_t)(sizeof(perf_step_offsets_rad) / sizeof(perf_step_offsets_rad[0]));

    if ((feedback == NULL) || (cmd == NULL) || !feedback->imu_online)
    {
        GimbalSysId_Init();
        return 0u;
    }

    now_tick_ms = HAL_GetTick();
    if (!sysid_state.started)
    {
        sysid_state.started = 1u;
        sysid_state.start_tick_ms = now_tick_ms;
        sysid_state.stage_start_tick_ms = now_tick_ms;
        sysid_state.stage_hold_ms = GIMBAL_SYSID_PERF_STEP_STAGE_MS;
        sysid_state.seq_index = 0u;
        sysid_state.current_offset_index = 0xFFu;
        sysid_state.current_offset_rad = 0.0f;
        sysid_state.base_yaw_rad = feedback->gimbal_imu_data.YawTotalAngle;
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_STEP
        sysid_state.base_pitch_rad = GimbalSysId_ClampF(feedback->gimbal_imu_data.Pitch,
                                                        GIMBAL_PITCH_MIN_RAD,
                                                        GIMBAL_PITCH_MAX_RAD);
#else
        sysid_state.base_pitch_rad = feedback->gimbal_imu_data.Pitch;
#endif
    }

    elapsed_ms = now_tick_ms - sysid_state.start_tick_ms;
    step_ms = (uint32_t)stage_count * GIMBAL_SYSID_PERF_STEP_STAGE_MS;
    cmd->gimbal_mode = GIMBAL_IMU_MODE;

    if (elapsed_ms < GIMBAL_SYSID_PERF_STEP_BASELINE_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_BASELINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_PERF_STEP_BASELINE_MS + step_ms))
    {
        stage_elapsed_ms = elapsed_ms - GIMBAL_SYSID_PERF_STEP_BASELINE_MS;
        stage_index = (uint8_t)(stage_elapsed_ms / GIMBAL_SYSID_PERF_STEP_STAGE_MS);
        if (stage_index >= stage_count)
        {
            stage_index = (uint8_t)(stage_count - 1u);
        }

        if (stage_index != sysid_state.current_offset_index)
        {
            sysid_state.current_offset_index = stage_index;
            sysid_state.current_offset_rad = perf_step_offsets_rad[stage_index];
            sysid_state.stage_start_tick_ms = now_tick_ms;
            sysid_state.seq_index++;
        }

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PERF_STEP
        cmd->yaw = sysid_state.base_yaw_rad + sysid_state.current_offset_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
#else
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = GimbalSysId_ClampF(sysid_state.base_pitch_rad + sysid_state.current_offset_rad,
                                        GIMBAL_PITCH_MIN_RAD,
                                        GIMBAL_PITCH_MAX_RAD);
#endif
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_STEP, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_PERF_STEP_BASELINE_MS + step_ms + GIMBAL_SYSID_PERF_STEP_RETURN_MS))
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_RETURN, elapsed_ms);
        return 1u;
    }

    sysid_state.current_offset_rad = 0.0f;
    cmd->yaw = sysid_state.base_yaw_rad;
    cmd->pitch = sysid_state.base_pitch_rad;
    GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_DONE, elapsed_ms);
    return 1u;
#elif (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PERF_SINE) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_SINE)
    uint32_t now_tick_ms;
    uint32_t elapsed_ms;
    uint32_t sine_ms;
    uint32_t stage_elapsed_ms;
    uint8_t stage_index;
    float stage_t_s;
    const uint8_t stage_count = (uint8_t)(sizeof(perf_sine_cases) / sizeof(perf_sine_cases[0]));

    if ((feedback == NULL) || (cmd == NULL) || !feedback->imu_online)
    {
        GimbalSysId_Init();
        return 0u;
    }

    now_tick_ms = HAL_GetTick();
    if (!sysid_state.started)
    {
        sysid_state.started = 1u;
        sysid_state.start_tick_ms = now_tick_ms;
        sysid_state.stage_start_tick_ms = now_tick_ms;
        sysid_state.stage_hold_ms = GIMBAL_SYSID_PERF_SINE_STAGE_MS;
        sysid_state.seq_index = 0u;
        sysid_state.current_offset_index = 0xFFu;
        sysid_state.current_offset_rad = 0.0f;
        sysid_state.base_yaw_rad = feedback->gimbal_imu_data.YawTotalAngle;
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_SINE
        sysid_state.base_pitch_rad = GimbalSysId_ClampF(feedback->gimbal_imu_data.Pitch,
                                                        GIMBAL_PITCH_MIN_RAD,
                                                        GIMBAL_PITCH_MAX_RAD);
#else
        sysid_state.base_pitch_rad = feedback->gimbal_imu_data.Pitch;
#endif
    }

    elapsed_ms = now_tick_ms - sysid_state.start_tick_ms;
    sine_ms = (uint32_t)stage_count * GIMBAL_SYSID_PERF_SINE_STAGE_MS;
    cmd->gimbal_mode = GIMBAL_IMU_MODE;

    if (elapsed_ms < GIMBAL_SYSID_PERF_SINE_BASELINE_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_BASELINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_PERF_SINE_BASELINE_MS + sine_ms))
    {
        stage_elapsed_ms = elapsed_ms - GIMBAL_SYSID_PERF_SINE_BASELINE_MS;
        stage_index = (uint8_t)(stage_elapsed_ms / GIMBAL_SYSID_PERF_SINE_STAGE_MS);
        if (stage_index >= stage_count)
        {
            stage_index = (uint8_t)(stage_count - 1u);
        }

        if (stage_index != sysid_state.current_offset_index)
        {
            sysid_state.current_offset_index = stage_index;
            sysid_state.stage_start_tick_ms = now_tick_ms;
            sysid_state.seq_index++;
        }

        stage_t_s = 0.001f * (float)(now_tick_ms - sysid_state.stage_start_tick_ms);
        sysid_state.current_offset_rad =
            perf_sine_cases[stage_index].amplitude_rad *
            sinf(GIMBAL_SYSID_TWO_PI * stage_t_s / perf_sine_cases[stage_index].period_s);

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PERF_SINE
        cmd->yaw = sysid_state.base_yaw_rad + sysid_state.current_offset_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
#else
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = GimbalSysId_ClampF(sysid_state.base_pitch_rad + sysid_state.current_offset_rad,
                                        GIMBAL_PITCH_MIN_RAD,
                                        GIMBAL_PITCH_MAX_RAD);
#endif
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_SINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_PERF_SINE_BASELINE_MS + sine_ms + GIMBAL_SYSID_PERF_SINE_RETURN_MS))
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_RETURN, elapsed_ms);
        return 1u;
    }

    sysid_state.current_offset_rad = 0.0f;
    cmd->yaw = sysid_state.base_yaw_rad;
    cmd->pitch = sysid_state.base_pitch_rad;
    GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_DONE, elapsed_ms);
    return 1u;
#elif GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF
    uint32_t now_tick_ms;
    uint32_t elapsed_ms;
    uint32_t pitch_ff_ms;
    uint32_t stage_elapsed_ms;
    uint8_t stage_index;
    const uint8_t stage_count = (uint8_t)(sizeof(pitch_ff_offsets_rad) / sizeof(pitch_ff_offsets_rad[0]));

    if ((feedback == NULL) || (cmd == NULL) || !feedback->imu_online)
    {
        GimbalSysId_Init();
        return 0u;
    }

    now_tick_ms = HAL_GetTick();
    if (!sysid_state.started)
    {
        sysid_state.started = 1u;
        sysid_state.start_tick_ms = now_tick_ms;
        sysid_state.stage_start_tick_ms = now_tick_ms;
        sysid_state.stage_hold_ms = GIMBAL_SYSID_PITCH_FF_STAGE_MS;
        sysid_state.seq_index = 0u;
        sysid_state.current_offset_index = 0xFFu;
        sysid_state.current_offset_rad = 0.0f;
        sysid_state.base_yaw_rad = feedback->gimbal_imu_data.YawTotalAngle;
        sysid_state.base_pitch_rad = GimbalSysId_ClampF(feedback->gimbal_imu_data.Pitch,
                                                        GIMBAL_PITCH_MIN_RAD,
                                                        GIMBAL_PITCH_MAX_RAD);
    }

    elapsed_ms = now_tick_ms - sysid_state.start_tick_ms;
    pitch_ff_ms = (uint32_t)stage_count * GIMBAL_SYSID_PITCH_FF_STAGE_MS;
    cmd->gimbal_mode = GIMBAL_IMU_MODE;
    cmd->yaw = sysid_state.base_yaw_rad;

    if (elapsed_ms < GIMBAL_SYSID_BASELINE_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_BASELINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_BASELINE_MS + pitch_ff_ms))
    {
        stage_elapsed_ms = elapsed_ms - GIMBAL_SYSID_BASELINE_MS;
        stage_index = (uint8_t)(stage_elapsed_ms / GIMBAL_SYSID_PITCH_FF_STAGE_MS);
        if (stage_index >= stage_count)
        {
            stage_index = (uint8_t)(stage_count - 1u);
        }

        if (stage_index != sysid_state.current_offset_index)
        {
            sysid_state.current_offset_index = stage_index;
            sysid_state.current_offset_rad = pitch_ff_offsets_rad[stage_index];
            sysid_state.stage_start_tick_ms = now_tick_ms;
            sysid_state.seq_index++;
            GimbalResetPitchControlState();
        }

        cmd->pitch = GimbalSysId_ClampF(sysid_state.base_pitch_rad + sysid_state.current_offset_rad,
                                        GIMBAL_PITCH_MIN_RAD,
                                        GIMBAL_PITCH_MAX_RAD);
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_STEP, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_BASELINE_MS + pitch_ff_ms + GIMBAL_SYSID_RETURN_MS))
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_RETURN, elapsed_ms);
        return 1u;
    }

    sysid_state.current_offset_rad = 0.0f;
    cmd->pitch = sysid_state.base_pitch_rad;
    GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_DONE, elapsed_ms);
    return 1u;
#elif GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_HYST
    uint32_t now_tick_ms;
    uint32_t elapsed_ms;
    uint32_t pitch_hyst_ms;
    uint32_t stage_elapsed_ms;
    uint8_t stage_index;
    const uint8_t stage_count = (uint8_t)(sizeof(pitch_hyst_offsets_rad) / sizeof(pitch_hyst_offsets_rad[0]));

    if ((feedback == NULL) || (cmd == NULL) || !feedback->imu_online)
    {
        GimbalSysId_Init();
        return 0u;
    }

    now_tick_ms = HAL_GetTick();
    if (!sysid_state.started)
    {
        sysid_state.started = 1u;
        sysid_state.start_tick_ms = now_tick_ms;
        sysid_state.stage_start_tick_ms = now_tick_ms;
        sysid_state.stage_hold_ms = GIMBAL_SYSID_PITCH_HYST_STAGE_MS;
        sysid_state.seq_index = 0u;
        sysid_state.current_offset_index = 0xFFu;
        sysid_state.current_offset_rad = 0.0f;
        sysid_state.base_yaw_rad = feedback->gimbal_imu_data.YawTotalAngle;
        sysid_state.base_pitch_rad = GimbalSysId_ClampF(feedback->gimbal_imu_data.Pitch,
                                                        GIMBAL_PITCH_MIN_RAD,
                                                        GIMBAL_PITCH_MAX_RAD);
    }

    elapsed_ms = now_tick_ms - sysid_state.start_tick_ms;
    pitch_hyst_ms = (uint32_t)stage_count * GIMBAL_SYSID_PITCH_HYST_STAGE_MS;
    cmd->gimbal_mode = GIMBAL_IMU_MODE;
    cmd->yaw = sysid_state.base_yaw_rad;

    if (elapsed_ms < GIMBAL_SYSID_BASELINE_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_BASELINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_BASELINE_MS + pitch_hyst_ms))
    {
        stage_elapsed_ms = elapsed_ms - GIMBAL_SYSID_BASELINE_MS;
        stage_index = (uint8_t)(stage_elapsed_ms / GIMBAL_SYSID_PITCH_HYST_STAGE_MS);
        if (stage_index >= stage_count)
        {
            stage_index = (uint8_t)(stage_count - 1u);
        }

        if (stage_index != sysid_state.current_offset_index)
        {
            sysid_state.current_offset_index = stage_index;
            sysid_state.current_offset_rad = pitch_hyst_offsets_rad[stage_index];
            sysid_state.stage_start_tick_ms = now_tick_ms;
            sysid_state.seq_index++;
            GimbalResetPitchControlState();
        }

        cmd->pitch = GimbalSysId_ClampF(sysid_state.base_pitch_rad + sysid_state.current_offset_rad,
                                        GIMBAL_PITCH_MIN_RAD,
                                        GIMBAL_PITCH_MAX_RAD);
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_STEP, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_BASELINE_MS + pitch_hyst_ms + GIMBAL_SYSID_RETURN_MS))
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_RETURN, elapsed_ms);
        return 1u;
    }

    sysid_state.current_offset_rad = 0.0f;
    cmd->pitch = sysid_state.base_pitch_rad;
    GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_DONE, elapsed_ms);
    return 1u;
#else
    (void)feedback;
    (void)cmd;
    return 0u;
#endif
}
