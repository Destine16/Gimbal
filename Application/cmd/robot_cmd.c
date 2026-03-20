#include "robot_cmd.h"

#include "message_center.h"
#include "robot_def.h"
#include "vision_comm.h"

static Publisher_t *gimbal_cmd_pub;
static Subscriber_t *gimbal_feed_sub;

static Robot_Status_e robot_state;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
static Gimbal_Upload_Data_s gimbal_fetch_data;

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

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

    robot_state = ROBOT_READY;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    gimbal_cmd_send.yaw = 0.0f;
    gimbal_cmd_send.pitch = 0.0f;
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
    gimbal_ready = (uint8_t)(gimbal_fetch_data.imu_online &&
                             gimbal_fetch_data.yaw_motor_online &&
                             gimbal_fetch_data.pitch_motor_online);

    if ((robot_state == ROBOT_STOP) || !gimbal_ready)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd_send.yaw = 0.0f;
        gimbal_cmd_send.pitch = 0.0f;
    }
    else if (target_tracked)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle +
                              0.01f * (float)vision_cmd.yaw_0p01deg;
        gimbal_cmd_send.pitch = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch +
                                       0.01f * (float)vision_cmd.pitch_0p01deg,
                                       GIMBAL_PITCH_MIN_DEG,
                                       GIMBAL_PITCH_MAX_DEG);
    }
    else
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = ClampF(gimbal_fetch_data.gimbal_imu_data.Pitch,
                                       GIMBAL_PITCH_MIN_DEG,
                                       GIMBAL_PITCH_MAX_DEG);
    }

    control_status.enemy_color = 0u;
    control_status.bullet_speed_0p01mps = 0u;
    control_status.vision_mode = (uint8_t)gimbal_cmd_send.gimbal_mode;
    control_status.fire_permission = (uint8_t)(gimbal_ready && target_tracked);
    VisionComm_UpdateControlStatus(&control_status);

    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd_send);
}
