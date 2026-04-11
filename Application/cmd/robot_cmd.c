#include "robot_cmd.h"

#include <string.h>

#include "stm32f4xx_hal.h"
#include "message_center.h"
#include "robot_def.h"
#include "vision_comm.h"

static Publisher_t *gimbal_cmd_pub;
static Subscriber_t *gimbal_feed_sub;

static Robot_Status_e robot_state;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
static Gimbal_Upload_Data_s gimbal_fetch_data;

#ifndef GIMBAL_AUTO_YAW_STEP_TEST_ENABLE
#define GIMBAL_AUTO_YAW_STEP_TEST_ENABLE 0
#endif

#ifndef GIMBAL_YAW_ONLY_TEST_ENABLE
#define GIMBAL_YAW_ONLY_TEST_ENABLE 0
#endif

#ifndef GIMBAL_AUTO_PITCH_STEP_TEST_ENABLE
#define GIMBAL_AUTO_PITCH_STEP_TEST_ENABLE 0
#endif

#ifndef GIMBAL_PITCH_ONLY_TEST_ENABLE
#define GIMBAL_PITCH_ONLY_TEST_ENABLE 0
#endif

#ifndef GIMBAL_AUTO_PITCH_TEST_BASE_RAD
#define GIMBAL_AUTO_PITCH_TEST_BASE_RAD 0.0f
#endif

#if GIMBAL_AUTO_YAW_STEP_TEST_ENABLE
typedef struct
{
    uint8_t started;
    uint8_t index;
    uint32_t stage_start_tick;
    float base_yaw_rad;
    float base_pitch_rad;
} GimbalAutoYawStepTest_t;

typedef struct
{
    float yaw_offset_rad;
    uint32_t hold_ms;
} GimbalAutoYawStepStage_t;

static GimbalAutoYawStepTest_t g_auto_yaw_test;
static const GimbalAutoYawStepStage_t g_auto_yaw_test_stages[] = {
    {0.0f, 8000u},
    {0.34906585f, 3000u},
    {0.0f, 3000u},
    {-0.34906585f, 3000u},
    {0.0f, 3000u},
    {0.69813170f, 3000u},
    {0.0f, 3000u},
    {-0.69813170f, 3000u},
    {0.0f, 4000u},
};
#endif

#if GIMBAL_AUTO_PITCH_STEP_TEST_ENABLE
typedef struct
{
    uint8_t started;
    uint8_t index;
    uint32_t stage_start_tick;
    float base_yaw_rad;
    float base_pitch_rad;
} GimbalAutoPitchStepTest_t;

typedef struct
{
    float pitch_offset_rad;
    uint32_t hold_ms;
} GimbalAutoPitchStepStage_t;

static GimbalAutoPitchStepTest_t g_auto_pitch_test;
static const GimbalAutoPitchStepStage_t g_auto_pitch_test_stages[] = {
    {0.0f, 8000u},
    {0.17453293f, 3000u},
    {0.0f, 3000u},
    {-0.17453293f, 3000u},
    {0.0f, 3000u},
    {0.34906585f, 3000u},
    {0.0f, 3000u},
    {-0.34906585f, 3000u},
    {0.0f, 4000u},
};
#endif

static float ClampF(float value, float min_value, float max_value)
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

void RobotCMDInit(void)
{
    VisionComm_Init();

    // robot_cmd 模块: 发布 gimbal_cmd,订阅 gimbal_feed
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

    robot_state = ROBOT_READY;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    gimbal_cmd_send.yaw = 0.0f;
    gimbal_cmd_send.pitch = 0.0f;

#if GIMBAL_AUTO_YAW_STEP_TEST_ENABLE
    memset(&g_auto_yaw_test, 0, sizeof(g_auto_yaw_test));
#endif
#if GIMBAL_AUTO_PITCH_STEP_TEST_ENABLE
    memset(&g_auto_pitch_test, 0, sizeof(g_auto_pitch_test));
#endif
}

void RobotCMDTask(void)
{
    VisionCmd_t vision_cmd;
    ControlStatus_t control_status = {0};
    uint8_t vision_online;
    uint8_t target_tracked;
    uint8_t gimbal_ready;

    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    VisionComm_Task();
    vision_online = VisionComm_GetVisionCmd(&vision_cmd);
    target_tracked = (uint8_t)(vision_online && (vision_cmd.track_state != 0u));
#if GIMBAL_AUTO_YAW_STEP_TEST_ENABLE
    gimbal_ready = gimbal_fetch_data.imu_online;
#elif GIMBAL_AUTO_PITCH_STEP_TEST_ENABLE
    gimbal_ready = gimbal_fetch_data.imu_online;
#elif GIMBAL_YAW_ONLY_TEST_ENABLE
    gimbal_ready = (uint8_t)(gimbal_fetch_data.imu_online &&
                             gimbal_fetch_data.yaw_motor_online);
#elif GIMBAL_PITCH_ONLY_TEST_ENABLE
    gimbal_ready = (uint8_t)(gimbal_fetch_data.imu_online &&
                             gimbal_fetch_data.pitch_motor_online);
#else
    gimbal_ready = (uint8_t)(gimbal_fetch_data.imu_online &&
                             gimbal_fetch_data.yaw_motor_online &&
                             gimbal_fetch_data.pitch_motor_online);
#endif

    if ((robot_state == ROBOT_STOP) || !gimbal_ready)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd_send.yaw = 0.0f;
        gimbal_cmd_send.pitch = 0.0f;
#if GIMBAL_AUTO_YAW_STEP_TEST_ENABLE
        g_auto_yaw_test.started = 0u;
        g_auto_yaw_test.index = 0u;
#endif
#if GIMBAL_AUTO_PITCH_STEP_TEST_ENABLE
        g_auto_pitch_test.started = 0u;
        g_auto_pitch_test.index = 0u;
#endif
    }
#if GIMBAL_AUTO_YAW_STEP_TEST_ENABLE
    else
    {
        const GimbalAutoYawStepStage_t *stage;
        uint32_t now_tick = HAL_GetTick();

        if (!g_auto_yaw_test.started)
        {
            g_auto_yaw_test.started = 1u;
            g_auto_yaw_test.index = 0u;
            g_auto_yaw_test.stage_start_tick = now_tick;
            g_auto_yaw_test.base_yaw_rad = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            g_auto_yaw_test.base_pitch_rad = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch,
                                                    GIMBAL_PITCH_MIN_RAD,
                                                    GIMBAL_PITCH_MAX_RAD);
        }

        if (g_auto_yaw_test.index < (sizeof(g_auto_yaw_test_stages) / sizeof(g_auto_yaw_test_stages[0])))
        {
            stage = &g_auto_yaw_test_stages[g_auto_yaw_test.index];
            if ((now_tick - g_auto_yaw_test.stage_start_tick) >= stage->hold_ms)
            {
                g_auto_yaw_test.stage_start_tick = now_tick;
                if (g_auto_yaw_test.index + 1u < (sizeof(g_auto_yaw_test_stages) / sizeof(g_auto_yaw_test_stages[0])))
                {
                    g_auto_yaw_test.index++;
                }
            }
        }

        stage = &g_auto_yaw_test_stages[g_auto_yaw_test.index];
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
        gimbal_cmd_send.yaw = g_auto_yaw_test.base_yaw_rad + stage->yaw_offset_rad;
        gimbal_cmd_send.pitch = g_auto_yaw_test.base_pitch_rad;
    }
#elif GIMBAL_AUTO_PITCH_STEP_TEST_ENABLE
    else
    {
        const GimbalAutoPitchStepStage_t *stage;
        uint32_t now_tick = HAL_GetTick();

        if (!g_auto_pitch_test.started)
        {
            g_auto_pitch_test.started = 1u;
            g_auto_pitch_test.index = 0u;
            g_auto_pitch_test.stage_start_tick = now_tick;
            g_auto_pitch_test.base_yaw_rad = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            g_auto_pitch_test.base_pitch_rad = ClampF(GIMBAL_AUTO_PITCH_TEST_BASE_RAD,
                                                      GIMBAL_PITCH_MIN_RAD,
                                                      GIMBAL_PITCH_MAX_RAD);
        }

        if (g_auto_pitch_test.index < (sizeof(g_auto_pitch_test_stages) / sizeof(g_auto_pitch_test_stages[0])))
        {
            stage = &g_auto_pitch_test_stages[g_auto_pitch_test.index];
            if ((now_tick - g_auto_pitch_test.stage_start_tick) >= stage->hold_ms)
            {
                g_auto_pitch_test.stage_start_tick = now_tick;
                if (g_auto_pitch_test.index + 1u < (sizeof(g_auto_pitch_test_stages) / sizeof(g_auto_pitch_test_stages[0])))
                {
                    g_auto_pitch_test.index++;
                }
            }
        }

        stage = &g_auto_pitch_test_stages[g_auto_pitch_test.index];
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
        gimbal_cmd_send.yaw = g_auto_pitch_test.base_yaw_rad;
        gimbal_cmd_send.pitch = ClampF(g_auto_pitch_test.base_pitch_rad + stage->pitch_offset_rad,
                                       GIMBAL_PITCH_MIN_RAD,
                                       GIMBAL_PITCH_MAX_RAD);
    }
#else
    else if (target_tracked)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle +
                              0.01f * (float)vision_cmd.yaw_0p01rad;
        gimbal_cmd_send.pitch = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch +
                                       0.01f * (float)vision_cmd.pitch_0p01rad,
                                       GIMBAL_PITCH_MIN_RAD,
                                       GIMBAL_PITCH_MAX_RAD);
    }
    else
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch,
                                       GIMBAL_PITCH_MIN_RAD,
                                       GIMBAL_PITCH_MAX_RAD);
    }
#endif

    control_status.enemy_color = 0u;
    control_status.bullet_speed_0p01mps = 0u;
    control_status.vision_mode = (uint8_t)gimbal_cmd_send.gimbal_mode;
    control_status.fire_permission = (uint8_t)(gimbal_ready && target_tracked);
    VisionComm_UpdateControlStatus(&control_status);

    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd_send);
}
