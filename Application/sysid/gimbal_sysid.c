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
#define GIMBAL_SYSID_FAST_MULTISINE_BASELINE_MS 3000u
#define GIMBAL_SYSID_FAST_MULTISINE_MS          45000u
#define GIMBAL_SYSID_FAST_MULTISINE_RETURN_MS   5000u
#define GIMBAL_SYSID_PITCH_FF_SWEEP_BASELINE_MS 3000u
#define GIMBAL_SYSID_PITCH_FF_SWEEP_RETURN_MS   4000u
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

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PRBS
static const float pitch_prbs_offsets_rad[] = {
    -0.17453293f, // -10 deg
    -0.12217305f, // -7 deg
    -0.06981317f, // -4 deg
    0.0f,
    0.06981317f,  // +4 deg
    0.12217305f,  // +7 deg
    0.17453293f,  // +10 deg
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

#if (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_FAST_MULTISINE) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FAST_MULTISINE)
typedef struct
{
    float amplitude_rad;
    float frequency_hz;
    float phase_rad;
} GimbalSysIdMultiSineComponent_s;

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_FAST_MULTISINE
static const GimbalSysIdMultiSineComponent_s fast_multisine_components[] = {
    {0.03490659f, 0.20f, 0.00f}, // 2.0 deg
    {0.03054326f, 0.40f, 1.70f}, // 1.75 deg
    {0.02617994f, 0.70f, 3.10f}, // 1.5 deg
    {0.02268928f, 1.00f, 4.20f}, // 1.3 deg
    {0.01919862f, 1.50f, 0.80f}, // 1.1 deg
    {0.01570796f, 2.20f, 2.40f}, // 0.9 deg
    {0.01221730f, 3.30f, 5.10f}, // 0.7 deg
    {0.00959931f, 5.00f, 1.10f}, // 0.55 deg
    {0.00698132f, 7.00f, 3.70f}, // 0.4 deg
    {0.00523599f, 10.00f, 5.50f}, // 0.3 deg
};
#else
static const GimbalSysIdMultiSineComponent_s fast_multisine_components[] = {
    {0.02617994f, 0.20f, 0.00f}, // 1.5 deg
    {0.02268928f, 0.40f, 1.70f}, // 1.3 deg
    {0.01919862f, 0.70f, 3.10f}, // 1.1 deg
    {0.01570796f, 1.00f, 4.20f}, // 0.9 deg
    {0.01308997f, 1.50f, 0.80f}, // 0.75 deg
    {0.01047198f, 2.20f, 2.40f}, // 0.6 deg
    {0.00785398f, 3.00f, 5.10f}, // 0.45 deg
    {0.00610865f, 4.50f, 1.10f}, // 0.35 deg
    {0.00436332f, 6.00f, 3.70f}, // 0.25 deg
};
#endif
#endif

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF_SWEEP
typedef struct
{
    float start_offset_rad;
    float end_offset_rad;
    float speed_rad_s;
} GimbalSysIdSweepSegment_s;

static const GimbalSysIdSweepSegment_s pitch_ff_sweep_segments[] = {
    {0.0f, -0.43633231f, 0.06981317f},          // 0 -> -25 deg, 4 deg/s
    {-0.43633231f, 0.43633231f, 0.03490659f},   // -25 -> +25 deg, 2 deg/s
    {0.43633231f, -0.43633231f, 0.03490659f},   // +25 -> -25 deg, 2 deg/s
    {-0.43633231f, 0.43633231f, 0.06981317f},   // -25 -> +25 deg, 4 deg/s
    {0.43633231f, -0.43633231f, 0.06981317f},   // +25 -> -25 deg, 4 deg/s
    {-0.43633231f, 0.43633231f, 0.10471976f},   // -25 -> +25 deg, 6 deg/s
    {0.43633231f, -0.43633231f, 0.10471976f},   // +25 -> -25 deg, 6 deg/s
    {-0.43633231f, 0.0f, 0.06981317f},          // -25 -> 0 deg, 4 deg/s
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

#if (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PRBS)
static uint8_t GimbalSysId_PrbsOffsetCount(void)
{
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS
    return (uint8_t)(sizeof(yaw_prbs_offsets_rad) / sizeof(yaw_prbs_offsets_rad[0]));
#else
    return (uint8_t)(sizeof(pitch_prbs_offsets_rad) / sizeof(pitch_prbs_offsets_rad[0]));
#endif
}

static float GimbalSysId_GetPrbsOffset(uint8_t index)
{
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS
    return yaw_prbs_offsets_rad[index];
#else
    return pitch_prbs_offsets_rad[index];
#endif
}

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
    const uint8_t count = GimbalSysId_PrbsOffsetCount();
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
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_SINE) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PRBS) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FAST_MULTISINE) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF_SWEEP)
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

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF_SWEEP
static uint8_t GimbalSysId_PitchSweepSegmentCount(void)
{
    return (uint8_t)(sizeof(pitch_ff_sweep_segments) / sizeof(pitch_ff_sweep_segments[0]));
}

static uint32_t GimbalSysId_PitchSweepSegmentMs(uint8_t index)
{
    float distance_rad;
    float speed_rad_s;

    if (index >= GimbalSysId_PitchSweepSegmentCount())
    {
        return 0u;
    }

    distance_rad = fabsf(pitch_ff_sweep_segments[index].end_offset_rad -
                         pitch_ff_sweep_segments[index].start_offset_rad);
    speed_rad_s = pitch_ff_sweep_segments[index].speed_rad_s;
    if ((distance_rad <= 0.0f) || (speed_rad_s <= 0.0f))
    {
        return 0u;
    }
    return (uint32_t)((distance_rad / speed_rad_s) * 1000.0f + 0.5f);
}

static uint32_t GimbalSysId_PitchSweepTotalMs(void)
{
    uint32_t total_ms = 0u;
    uint8_t i;

    for (i = 0u; i < GimbalSysId_PitchSweepSegmentCount(); i++)
    {
        total_ms += GimbalSysId_PitchSweepSegmentMs(i);
    }
    return total_ms;
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
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PERF_SINE) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_FAST_MULTISINE)
    gimbal_sysid_debug.yaw_offset_rad = sysid_state.current_offset_rad;
    gimbal_sysid_debug.pitch_offset_rad = 0.0f;
#elif (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_HYST) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_STEP) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PERF_SINE) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PRBS) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FAST_MULTISINE) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF_SWEEP)
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
#if (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PRBS)
    sysid_state.current_offset_index = (uint8_t)(GimbalSysId_PrbsOffsetCount() / 2u);
    sysid_state.rng_state = GIMBAL_SYSID_PRBS_SEED;
#endif
#else
    memset((void *)&gimbal_sysid_debug, 0, sizeof(gimbal_sysid_debug));
#endif
}

uint8_t GimbalSysId_Update(const Gimbal_Upload_Data_s *feedback, Gimbal_Ctrl_Cmd_s *cmd)
{
#if (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS) || \
    (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PRBS)
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
        sysid_state.current_offset_index = (uint8_t)(GimbalSysId_PrbsOffsetCount() / 2u);
        sysid_state.current_offset_rad = 0.0f;
        sysid_state.rng_state = GIMBAL_SYSID_PRBS_SEED;
        sysid_state.base_yaw_rad = feedback->gimbal_imu_data.YawTotalAngle;
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_PRBS
        sysid_state.base_pitch_rad = GimbalSysId_ClampF(feedback->gimbal_imu_data.Pitch,
                                                        GIMBAL_PITCH_MIN_RAD,
                                                        GIMBAL_PITCH_MAX_RAD);
#else
        sysid_state.base_pitch_rad = feedback->gimbal_imu_data.Pitch;
#endif
    }

    elapsed_ms = now_tick_ms - sysid_state.start_tick_ms;
    cmd->gimbal_mode = GIMBAL_IMU_MODE;

    if (elapsed_ms < GIMBAL_SYSID_BASELINE_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_BASELINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_BASELINE_MS + GIMBAL_SYSID_PRBS_MS))
    {
        if ((sysid_state.stage_hold_ms == 0u) ||
            ((now_tick_ms - sysid_state.stage_start_tick_ms) >= sysid_state.stage_hold_ms))
        {
            sysid_state.current_offset_index = GimbalSysId_NextOffsetIndex();
            sysid_state.current_offset_rad = GimbalSysId_GetPrbsOffset(sysid_state.current_offset_index);
            sysid_state.stage_hold_ms = GimbalSysId_NextHoldMs();
            sysid_state.stage_start_tick_ms = now_tick_ms;
            sysid_state.seq_index++;
        }

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_PRBS
        cmd->yaw = sysid_state.base_yaw_rad + sysid_state.current_offset_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
#else
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = GimbalSysId_ClampF(sysid_state.base_pitch_rad + sysid_state.current_offset_rad,
                                        GIMBAL_PITCH_MIN_RAD,
                                        GIMBAL_PITCH_MAX_RAD);
#endif
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_PRBS, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < GIMBAL_SYSID_TOTAL_MS)
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
#elif (GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_FAST_MULTISINE) || \
      (GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FAST_MULTISINE)
    uint32_t now_tick_ms;
    uint32_t elapsed_ms;
    float t_s;
    float offset_rad = 0.0f;
    size_t i;

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
        sysid_state.stage_hold_ms = GIMBAL_SYSID_FAST_MULTISINE_MS;
        sysid_state.seq_index = 0u;
        sysid_state.current_offset_index = 0u;
        sysid_state.current_offset_rad = 0.0f;
        sysid_state.base_yaw_rad = feedback->gimbal_imu_data.YawTotalAngle;
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FAST_MULTISINE
        sysid_state.base_pitch_rad = GimbalSysId_ClampF(feedback->gimbal_imu_data.Pitch,
                                                        GIMBAL_PITCH_MIN_RAD,
                                                        GIMBAL_PITCH_MAX_RAD);
#else
        sysid_state.base_pitch_rad = feedback->gimbal_imu_data.Pitch;
#endif
    }

    elapsed_ms = now_tick_ms - sysid_state.start_tick_ms;
    cmd->gimbal_mode = GIMBAL_IMU_MODE;

    if (elapsed_ms < GIMBAL_SYSID_FAST_MULTISINE_BASELINE_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_BASELINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_FAST_MULTISINE_BASELINE_MS + GIMBAL_SYSID_FAST_MULTISINE_MS))
    {
        t_s = 0.001f * (float)(elapsed_ms - GIMBAL_SYSID_FAST_MULTISINE_BASELINE_MS);
        for (i = 0u; i < (sizeof(fast_multisine_components) / sizeof(fast_multisine_components[0])); i++)
        {
            offset_rad += fast_multisine_components[i].amplitude_rad *
                          sinf(GIMBAL_SYSID_TWO_PI * fast_multisine_components[i].frequency_hz * t_s +
                               fast_multisine_components[i].phase_rad);
        }
        sysid_state.current_offset_rad = offset_rad;
        sysid_state.seq_index = (uint16_t)((elapsed_ms - GIMBAL_SYSID_FAST_MULTISINE_BASELINE_MS) / 100u);

#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_YAW_FAST_MULTISINE
        cmd->yaw = sysid_state.base_yaw_rad + sysid_state.current_offset_rad;
        cmd->pitch = sysid_state.base_pitch_rad;
#else
        cmd->yaw = sysid_state.base_yaw_rad;
        cmd->pitch = GimbalSysId_ClampF(sysid_state.base_pitch_rad + sysid_state.current_offset_rad,
                                        GIMBAL_PITCH_MIN_RAD,
                                        GIMBAL_PITCH_MAX_RAD);
#endif
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_MULTISINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_FAST_MULTISINE_BASELINE_MS +
                      GIMBAL_SYSID_FAST_MULTISINE_MS +
                      GIMBAL_SYSID_FAST_MULTISINE_RETURN_MS))
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
#elif GIMBAL_SYSID_MODE == GIMBAL_SYSID_PITCH_FF_SWEEP
    uint32_t now_tick_ms;
    uint32_t elapsed_ms;
    uint32_t sweep_elapsed_ms;
    uint32_t segment_start_ms = 0u;
    uint32_t segment_ms = 0u;
    uint32_t sweep_total_ms;
    uint8_t segment_index;
    uint8_t segment_count;
    float segment_ratio;
    float offset_delta;

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
        sysid_state.current_offset_index = 0xFFu;
        sysid_state.current_offset_rad = 0.0f;
        sysid_state.base_yaw_rad = feedback->gimbal_imu_data.YawTotalAngle;
        sysid_state.base_pitch_rad = GimbalSysId_ClampF(feedback->gimbal_imu_data.Pitch,
                                                        GIMBAL_PITCH_MIN_RAD,
                                                        GIMBAL_PITCH_MAX_RAD);
    }

    elapsed_ms = now_tick_ms - sysid_state.start_tick_ms;
    sweep_total_ms = GimbalSysId_PitchSweepTotalMs();
    cmd->gimbal_mode = GIMBAL_IMU_MODE;
    cmd->yaw = sysid_state.base_yaw_rad;

    if (elapsed_ms < GIMBAL_SYSID_PITCH_FF_SWEEP_BASELINE_MS)
    {
        sysid_state.current_offset_rad = 0.0f;
        cmd->pitch = sysid_state.base_pitch_rad;
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_BASELINE, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_PITCH_FF_SWEEP_BASELINE_MS + sweep_total_ms))
    {
        sweep_elapsed_ms = elapsed_ms - GIMBAL_SYSID_PITCH_FF_SWEEP_BASELINE_MS;
        segment_count = GimbalSysId_PitchSweepSegmentCount();
        for (segment_index = 0u; segment_index < segment_count; segment_index++)
        {
            segment_ms = GimbalSysId_PitchSweepSegmentMs(segment_index);
            if (sweep_elapsed_ms < (segment_start_ms + segment_ms))
            {
                break;
            }
            segment_start_ms += segment_ms;
        }
        if (segment_index >= segment_count)
        {
            segment_index = (uint8_t)(segment_count - 1u);
            segment_ms = GimbalSysId_PitchSweepSegmentMs(segment_index);
            segment_start_ms = sweep_total_ms - segment_ms;
        }

        if (segment_index != sysid_state.current_offset_index)
        {
            sysid_state.current_offset_index = segment_index;
            sysid_state.stage_start_tick_ms = now_tick_ms;
            sysid_state.seq_index++;
        }

        if (segment_ms == 0u)
        {
            segment_ratio = 1.0f;
        }
        else
        {
            segment_ratio = (float)(sweep_elapsed_ms - segment_start_ms) / (float)segment_ms;
            segment_ratio = GimbalSysId_ClampF(segment_ratio, 0.0f, 1.0f);
        }
        offset_delta = pitch_ff_sweep_segments[segment_index].end_offset_rad -
                       pitch_ff_sweep_segments[segment_index].start_offset_rad;
        sysid_state.current_offset_rad = pitch_ff_sweep_segments[segment_index].start_offset_rad +
                                         offset_delta * segment_ratio;
        cmd->pitch = GimbalSysId_ClampF(sysid_state.base_pitch_rad + sysid_state.current_offset_rad,
                                        GIMBAL_PITCH_MIN_RAD,
                                        GIMBAL_PITCH_MAX_RAD);
        GimbalSysId_DebugUpdate(GIMBAL_SYSID_PHASE_SWEEP, elapsed_ms);
        return 1u;
    }

    if (elapsed_ms < (GIMBAL_SYSID_PITCH_FF_SWEEP_BASELINE_MS +
                      sweep_total_ms +
                      GIMBAL_SYSID_PITCH_FF_SWEEP_RETURN_MS))
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
