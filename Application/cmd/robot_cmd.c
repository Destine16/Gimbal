#include "robot_cmd.h"

#include <math.h>
#include <string.h>

#include "gimbal.h"
#include "gm6020.h"
#include "message_center.h"
#include "robot_def.h"
#include "vision_comm.h"
#include "gimbal_sysid.h"
#include "sysid_rtt.h"

static Publisher_t *gimbal_cmd_pub;
static Subscriber_t *gimbal_feed_sub;

static Robot_Status_e robot_state;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
static Gimbal_Upload_Data_s gimbal_fetch_data;
static uint8_t vision_target_valid;
static float vision_target_yaw_rad;
static float vision_target_pitch_rad;
volatile RobotCmdDebug_s robot_cmd_debug;

typedef enum
{
    SENTRY_STATE_SCAN = 0,
    SENTRY_STATE_TRACK,
    SENTRY_STATE_TARGET_LOST,
    SENTRY_STATE_STALL_RECOVERY,
} SentryState_e;

typedef enum
{
    SENTRY_STALL_AXIS_NONE = 0,
    SENTRY_STALL_AXIS_YAW,
    SENTRY_STALL_AXIS_PITCH,
} SentryStallAxis_e;

static SentryState_e sentry_state;
static SentryStallAxis_e stall_axis;
static uint32_t last_robot_cmd_tick_ms;
static uint32_t last_target_valid_tick_ms;
static uint32_t target_lost_enter_tick_ms;
static uint32_t stall_recovery_end_tick_ms;
static uint32_t yaw_stall_start_tick_ms;
static uint32_t pitch_stall_start_tick_ms;
static float scan_yaw_center_rad;
static float scan_pitch_center_rad;
static float scan_yaw_target_rad;
static float scan_pitch_target_rad;
static float scan_yaw_dir;
static float scan_pitch_dir;
static float recovery_yaw_target_rad;
static float recovery_pitch_target_rad;

#ifndef GIMBAL_YAW_ONLY_TEST_ENABLE
#define GIMBAL_YAW_ONLY_TEST_ENABLE 0
#endif

#ifndef GIMBAL_PITCH_ONLY_TEST_ENABLE
#define GIMBAL_PITCH_ONLY_TEST_ENABLE 0
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

static float AbsF(float value)
{
    return fabsf(value);
}

static float SignF(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

static float LimitStep(float current, float min_value, float max_value, float *dir)
{
    if (current > max_value)
    {
        current = max_value;
        *dir = -1.0f;
    }
    else if (current < min_value)
    {
        current = min_value;
        *dir = 1.0f;
    }
    return current;
}

static void SentryResetStallTimers(void)
{
    yaw_stall_start_tick_ms = 0u;
    pitch_stall_start_tick_ms = 0u;
}

static void SentryEnterScan(float current_yaw, float current_pitch)
{
    sentry_state = SENTRY_STATE_SCAN;
    scan_yaw_center_rad = current_yaw;
    scan_pitch_center_rad = ClampF(current_pitch,
                                   GIMBAL_PITCH_MIN_RAD + SENTRY_SCAN_PITCH_RANGE_RAD,
                                   GIMBAL_PITCH_MAX_RAD - SENTRY_SCAN_PITCH_RANGE_RAD);
    scan_yaw_target_rad = current_yaw;
    scan_pitch_target_rad = ClampF(current_pitch,
                                   GIMBAL_PITCH_MIN_RAD,
                                   GIMBAL_PITCH_MAX_RAD);
    scan_yaw_dir = 1.0f;
    scan_pitch_dir = 1.0f;
    vision_target_valid = 0u;
}

static void SentryUpdateScan(float dt_s)
{
    const float yaw_min = scan_yaw_center_rad - SENTRY_SCAN_YAW_RANGE_RAD;
    const float yaw_max = scan_yaw_center_rad + SENTRY_SCAN_YAW_RANGE_RAD;
    const float pitch_min = ClampF(scan_pitch_center_rad - SENTRY_SCAN_PITCH_RANGE_RAD,
                                   GIMBAL_PITCH_MIN_RAD,
                                   GIMBAL_PITCH_MAX_RAD);
    const float pitch_max = ClampF(scan_pitch_center_rad + SENTRY_SCAN_PITCH_RANGE_RAD,
                                   GIMBAL_PITCH_MIN_RAD,
                                   GIMBAL_PITCH_MAX_RAD);

    scan_yaw_target_rad += scan_yaw_dir * SENTRY_SCAN_YAW_SPEED_RAD_S * dt_s;
    scan_pitch_target_rad += scan_pitch_dir * SENTRY_SCAN_PITCH_SPEED_RAD_S * dt_s;
    scan_yaw_target_rad = LimitStep(scan_yaw_target_rad, yaw_min, yaw_max, &scan_yaw_dir);
    scan_pitch_target_rad = LimitStep(scan_pitch_target_rad, pitch_min, pitch_max, &scan_pitch_dir);
}

static uint8_t SentryAxisStallDetected(const GM6020_ControlSnapshot_s *snapshot,
                                       uint32_t now_tick,
                                       uint32_t *stall_start_tick)
{
    float output_abs;
    float speed_abs;
    float error_abs;

    if ((snapshot == NULL) || (stall_start_tick == NULL) ||
        !snapshot->valid || !snapshot->enabled || !snapshot->online)
    {
        if (stall_start_tick != NULL)
        {
            *stall_start_tick = 0u;
        }
        return 0u;
    }

    output_abs = AbsF((float)snapshot->output_cmd);
    speed_abs = AbsF(snapshot->speed_feedback_rad_s);
    error_abs = AbsF(snapshot->angle_ref_rad - snapshot->angle_feedback_rad);
    if ((output_abs >= SENTRY_STALL_OUTPUT_THRESHOLD_RAW) &&
        (speed_abs <= SENTRY_STALL_SPEED_THRESHOLD_RAD_S) &&
        (error_abs >= SENTRY_STALL_ANGLE_ERROR_MIN_RAD))
    {
        if (*stall_start_tick == 0u)
        {
            *stall_start_tick = now_tick;
            return 0u;
        }
        return (uint8_t)((now_tick - *stall_start_tick) >= SENTRY_STALL_DETECT_MS);
    }

    *stall_start_tick = 0u;
    return 0u;
}

static uint8_t SentryCheckStall(uint32_t now_tick,
                                float current_yaw,
                                float current_pitch)
{
    GM6020_ControlSnapshot_s yaw_snapshot;
    GM6020_ControlSnapshot_s pitch_snapshot;
    float axis_error;

    if (sentry_state == SENTRY_STATE_STALL_RECOVERY)
    {
        return 0u;
    }

    if (GM6020_GetControlSnapshot(GIMBAL_YAW_MOTOR_ID, &yaw_snapshot) &&
        SentryAxisStallDetected(&yaw_snapshot, now_tick, &yaw_stall_start_tick_ms))
    {
        axis_error = yaw_snapshot.angle_ref_rad - yaw_snapshot.angle_feedback_rad;
        if (AbsF(axis_error) < 0.001f)
        {
            axis_error = (float)yaw_snapshot.output_cmd;
        }
        recovery_yaw_target_rad = current_yaw - SignF(axis_error) * SENTRY_STALL_YAW_BACKOFF_RAD;
        recovery_pitch_target_rad = ClampF(current_pitch, GIMBAL_PITCH_MIN_RAD, GIMBAL_PITCH_MAX_RAD);
        stall_axis = SENTRY_STALL_AXIS_YAW;
        return 1u;
    }

    if (GM6020_GetControlSnapshot(GIMBAL_PITCH_MOTOR_ID, &pitch_snapshot) &&
        SentryAxisStallDetected(&pitch_snapshot, now_tick, &pitch_stall_start_tick_ms))
    {
        axis_error = pitch_snapshot.angle_ref_rad - pitch_snapshot.angle_feedback_rad;
        if (AbsF(axis_error) < 0.001f)
        {
            axis_error = (float)pitch_snapshot.output_cmd;
        }
        recovery_yaw_target_rad = current_yaw;
        recovery_pitch_target_rad = ClampF(current_pitch - SignF(axis_error) * SENTRY_STALL_PITCH_BACKOFF_RAD,
                                           GIMBAL_PITCH_MIN_RAD,
                                           GIMBAL_PITCH_MAX_RAD);
        stall_axis = SENTRY_STALL_AXIS_PITCH;
        return 1u;
    }

    stall_axis = SENTRY_STALL_AXIS_NONE;
    return 0u;
}

static void SentryEnterStallRecovery(uint32_t now_tick)
{
    sentry_state = SENTRY_STATE_STALL_RECOVERY;
    stall_recovery_end_tick_ms = now_tick + SENTRY_STALL_RECOVERY_MS;
    vision_target_valid = 0u;
    SentryResetStallTimers();
    if (stall_axis == SENTRY_STALL_AXIS_YAW)
    {
        GimbalResetYawControlState();
    }
    else if (stall_axis == SENTRY_STALL_AXIS_PITCH)
    {
        GimbalResetPitchControlState();
    }
}

void RobotCMDInit(void)
{
    VisionComm_Init();
    GimbalSysId_Init();
    SysIdRtt_Init();

    // robot_cmd 模块: 发布 gimbal_cmd,订阅 gimbal_feed
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

    robot_state = ROBOT_READY;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    gimbal_cmd_send.yaw = 0.0f;
    gimbal_cmd_send.pitch = 0.0f;
    vision_target_valid = 0u;
    vision_target_yaw_rad = 0.0f;
    vision_target_pitch_rad = 0.0f;
    sentry_state = SENTRY_STATE_SCAN;
    stall_axis = SENTRY_STALL_AXIS_NONE;
    last_robot_cmd_tick_ms = HAL_GetTick();
    last_target_valid_tick_ms = 0u;
    target_lost_enter_tick_ms = 0u;
    stall_recovery_end_tick_ms = 0u;
    SentryResetStallTimers();
    scan_yaw_center_rad = 0.0f;
    scan_pitch_center_rad = 0.0f;
    scan_yaw_target_rad = 0.0f;
    scan_pitch_target_rad = 0.0f;
    scan_yaw_dir = 1.0f;
    scan_pitch_dir = 1.0f;
    recovery_yaw_target_rad = 0.0f;
    recovery_pitch_target_rad = 0.0f;
    memset((void *)&robot_cmd_debug, 0, sizeof(robot_cmd_debug));
}

void RobotCMDTask(void)
{
    VisionCmd_t vision_cmd;
    uint8_t vision_cmd_ready = 0u;
    VisionStatus_t vision_status;
    uint8_t gimbal_ready;
    uint32_t now_tick = HAL_GetTick();
    float dt_s;
    float current_yaw_rad;
    float current_pitch_rad;
    uint8_t stall_detected = 0u;

    if (last_robot_cmd_tick_ms == 0u)
    {
        dt_s = 0.005f;
    }
    else
    {
        dt_s = 0.001f * (float)(now_tick - last_robot_cmd_tick_ms);
        if ((dt_s <= 0.0f) || (dt_s > 0.05f))
        {
            dt_s = 0.005f;
        }
    }
    last_robot_cmd_tick_ms = now_tick;

    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    current_yaw_rad = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    current_pitch_rad = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch,
                               GIMBAL_PITCH_MIN_RAD,
                               GIMBAL_PITCH_MAX_RAD);
    memset(&vision_status, 0, sizeof(vision_status));
    vision_status.yaw_actual_1e4rad = (int32_t)(current_yaw_rad * 10000.0f);
    vision_status.pitch_actual_1e4rad = (int32_t)(current_pitch_rad * 10000.0f);
    VisionComm_UpdateStatus(&vision_status);
    VisionComm_Task();
    SysIdRtt_Task();
#if GIMBAL_YAW_ONLY_TEST_ENABLE
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
        SentryEnterScan(current_yaw_rad, current_pitch_rad);
        stall_axis = SENTRY_STALL_AXIS_NONE;
        stall_recovery_end_tick_ms = 0u;
        SentryResetStallTimers();
    }
#if GIMBAL_SYSID_MODE != GIMBAL_SYSID_NONE
    else
    {
        if (!GimbalSysId_Update(&gimbal_fetch_data, &gimbal_cmd_send))
        {
            gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            gimbal_cmd_send.pitch = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch,
                                           GIMBAL_PITCH_MIN_RAD,
                                           GIMBAL_PITCH_MAX_RAD);
        }
    }
#else
    else
    {
        if (sentry_state != SENTRY_STATE_STALL_RECOVERY)
        {
            vision_cmd_ready = VisionComm_GetVisionCmd(&vision_cmd);
        }
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
        if (vision_cmd_ready)
        {
            if (vision_cmd.target_valid)
            {
                vision_target_yaw_rad = current_yaw_rad +
                                        0.0001f * (float)vision_cmd.delta_yaw_1e4rad;
                vision_target_pitch_rad = ClampF(current_pitch_rad +
                                                 0.0001f * (float)vision_cmd.delta_pitch_1e4rad,
                                                 GIMBAL_PITCH_MIN_RAD,
                                                 GIMBAL_PITCH_MAX_RAD);
                vision_target_valid = 1u;
                last_target_valid_tick_ms = now_tick;
                sentry_state = SENTRY_STATE_TRACK;
                stall_axis = SENTRY_STALL_AXIS_NONE;
            }
            else
            {
                vision_target_valid = 0u;
                if (sentry_state == SENTRY_STATE_TRACK)
                {
                    sentry_state = SENTRY_STATE_TARGET_LOST;
                    target_lost_enter_tick_ms = now_tick;
                }
            }
        }

        if ((sentry_state == SENTRY_STATE_TRACK) &&
            ((now_tick - last_target_valid_tick_ms) > SENTRY_TARGET_LOST_HOLD_MS))
        {
            sentry_state = SENTRY_STATE_TARGET_LOST;
            target_lost_enter_tick_ms = now_tick;
        }

        if ((sentry_state == SENTRY_STATE_TARGET_LOST) &&
            ((now_tick - target_lost_enter_tick_ms) > SENTRY_TARGET_LOST_TO_SCAN_MS))
        {
            SentryEnterScan(current_yaw_rad, current_pitch_rad);
        }

        stall_detected = SentryCheckStall(now_tick, current_yaw_rad, current_pitch_rad);
        if (stall_detected)
        {
            SentryEnterStallRecovery(now_tick);
        }

        if (sentry_state == SENTRY_STATE_STALL_RECOVERY)
        {
            if ((int32_t)(now_tick - stall_recovery_end_tick_ms) >= 0)
            {
                stall_axis = SENTRY_STALL_AXIS_NONE;
                SentryEnterScan(current_yaw_rad, current_pitch_rad);
            }
            else
            {
                gimbal_cmd_send.yaw = recovery_yaw_target_rad;
                gimbal_cmd_send.pitch = recovery_pitch_target_rad;
            }
        }

        if (sentry_state == SENTRY_STATE_TRACK)
        {
            gimbal_cmd_send.yaw = vision_target_yaw_rad;
            gimbal_cmd_send.pitch = vision_target_pitch_rad;
        }
        else if (sentry_state == SENTRY_STATE_TARGET_LOST)
        {
            gimbal_cmd_send.yaw = vision_target_yaw_rad;
            gimbal_cmd_send.pitch = vision_target_pitch_rad;
        }
        else if (sentry_state == SENTRY_STATE_SCAN)
        {
            SentryUpdateScan(dt_s);
            gimbal_cmd_send.yaw = scan_yaw_target_rad;
            gimbal_cmd_send.pitch = scan_pitch_target_rad;
        }
    }
#endif

    robot_cmd_debug.robot_state = (uint8_t)robot_state;
    robot_cmd_debug.gimbal_ready = gimbal_ready;
    robot_cmd_debug.gimbal_mode = (uint8_t)gimbal_cmd_send.gimbal_mode;
    robot_cmd_debug.vision_target_valid = vision_target_valid;
    robot_cmd_debug.vision_cmd_ready = vision_cmd_ready;
    robot_cmd_debug.vision_cmd_target_valid = vision_cmd_ready ? vision_cmd.target_valid : 0u;
    robot_cmd_debug.sentry_state = (uint8_t)sentry_state;
    robot_cmd_debug.stall_axis = (uint8_t)stall_axis;
    robot_cmd_debug.imu_online = gimbal_fetch_data.imu_online;
    robot_cmd_debug.yaw_motor_online = gimbal_fetch_data.yaw_motor_online;
    robot_cmd_debug.pitch_motor_online = gimbal_fetch_data.pitch_motor_online;
    robot_cmd_debug.stall_detected = stall_detected;
    robot_cmd_debug.last_target_valid_tick_ms = last_target_valid_tick_ms;
    robot_cmd_debug.stall_recovery_end_tick_ms = stall_recovery_end_tick_ms;
    robot_cmd_debug.vision_target_yaw_rad = vision_target_yaw_rad;
    robot_cmd_debug.vision_target_pitch_rad = vision_target_pitch_rad;
    robot_cmd_debug.scan_yaw_target_rad = scan_yaw_target_rad;
    robot_cmd_debug.scan_pitch_target_rad = scan_pitch_target_rad;
    robot_cmd_debug.recovery_yaw_target_rad = recovery_yaw_target_rad;
    robot_cmd_debug.recovery_pitch_target_rad = recovery_pitch_target_rad;
    robot_cmd_debug.cmd_yaw_rad = gimbal_cmd_send.yaw;
    robot_cmd_debug.cmd_pitch_rad = gimbal_cmd_send.pitch;

    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd_send);
}
