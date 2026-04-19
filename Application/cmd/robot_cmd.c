#include "robot_cmd.h"

#include <string.h>

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
}

void RobotCMDTask(void)
{
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_NONE
    VisionCmd_t vision_cmd;
    uint8_t vision_cmd_ready;
#endif
    VisionStatus_t vision_status;
    uint8_t gimbal_ready;

    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    memset(&vision_status, 0, sizeof(vision_status));
    vision_status.yaw_actual_1e4rad = (int32_t)(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle * 10000.0f);
    vision_status.pitch_actual_1e4rad = (int32_t)(gimbal_fetch_data.gimbal_imu_data.Pitch * 10000.0f);
    VisionComm_UpdateStatus(&vision_status);
    VisionComm_Task();
    SysIdRtt_Task();
#if GIMBAL_SYSID_MODE == GIMBAL_SYSID_NONE
    vision_cmd_ready = VisionComm_GetVisionCmd(&vision_cmd);
#endif
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
        vision_target_valid = 0u;
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
#if VISION_CONTROL_MODE == VISION_CONTROL_EVENT_TARGET
    else
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
        if (vision_cmd_ready)
        {
            vision_target_yaw_rad = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle +
                                    0.0001f * (float)vision_cmd.delta_yaw_1e4rad;
            vision_target_pitch_rad = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch +
                                             0.0001f * (float)vision_cmd.delta_pitch_1e4rad,
                                             GIMBAL_PITCH_MIN_RAD,
                                             GIMBAL_PITCH_MAX_RAD);
            vision_target_valid = 1u;
        }

        if (vision_target_valid)
        {
            gimbal_cmd_send.yaw = vision_target_yaw_rad;
            gimbal_cmd_send.pitch = vision_target_pitch_rad;
        }
        else
        {
            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            gimbal_cmd_send.pitch = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch,
                                           GIMBAL_PITCH_MIN_RAD,
                                           GIMBAL_PITCH_MAX_RAD);
        }
    }
#else
    else if (vision_cmd_ready)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle +
                              0.0001f * (float)vision_cmd.delta_yaw_1e4rad;
        gimbal_cmd_send.pitch = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch +
                                       0.0001f * (float)vision_cmd.delta_pitch_1e4rad,
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
#endif

    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd_send);
}
