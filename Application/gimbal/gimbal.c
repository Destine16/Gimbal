#include "gimbal.h"

#include <string.h>

#include "ins_task.h"
#include "message_center.h"
#include "gm6020.h"
#include "gimbal_params.h"
#include "robot_def.h"

static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;

static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static const INS_t *gimbal_ins;
static GM6020_Instance *yaw_motor;
static GM6020_Instance *pitch_motor;
static float yaw_angle_feedback_deg;
static float yaw_speed_feedback;
static float pitch_angle_feedback_deg;
static float pitch_speed_feedback;

static float ClampMotorTarget(float target, const GimbalMotorParam_s *param)
{
    if ((param == NULL) || !param->angle_limit_enable)
    {
        return target;
    }
    if (target > param->max_angle_deg)
    {
        return param->max_angle_deg;
    }
    if (target < param->min_angle_deg)
    {
        return param->min_angle_deg;
    }
    return target;
}

void GimbalInit(void)
{
    memset(&gimbal_cmd_recv, 0, sizeof(gimbal_cmd_recv));
    memset(&gimbal_feedback_data, 0, sizeof(gimbal_feedback_data));

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_ins = INS_GetData();

    yaw_motor = GM6020_Init(&(GM6020_Init_Config_s){
        .can_handle = &hcan1,
        .motor_id = GimbalYawParam.motor_id,
        .angle_feedback_ptr = &yaw_angle_feedback_deg,
        .speed_feedback_ptr = &yaw_speed_feedback,
        .current_feedback_sign = GimbalYawParam.current_feedback_sign,
        .output_sign = GimbalYawParam.output_sign,
        .angle_pid_config = GimbalYawParam.angle_pid,
        .speed_pid_config = GimbalYawParam.speed_pid,
        .current_pid_config = GimbalYawParam.current_pid,
    });

    pitch_motor = GM6020_Init(&(GM6020_Init_Config_s){
        .can_handle = &hcan1,
        .motor_id = GimbalPitchParam.motor_id,
        .angle_feedback_ptr = &pitch_angle_feedback_deg,
        .speed_feedback_ptr = &pitch_speed_feedback,
        .current_feedback_sign = GimbalPitchParam.current_feedback_sign,
        .output_sign = GimbalPitchParam.output_sign,
        .angle_pid_config = GimbalPitchParam.angle_pid,
        .speed_pid_config = GimbalPitchParam.speed_pid,
        .current_pid_config = GimbalPitchParam.current_pid,
    });
}

void GimbalTask(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint8_t imu_online = 0u;

    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    gimbal_ins = INS_GetData();
    yaw_angle_feedback_deg = GimbalYawParam.angle_feedback_sign * gimbal_ins->YawTotalAngle;
    yaw_speed_feedback = GimbalYawParam.speed_feedback_sign * gimbal_ins->Gyro[2];
    pitch_angle_feedback_deg = GimbalPitchParam.angle_feedback_sign * gimbal_ins->Pitch;
    pitch_speed_feedback = GimbalPitchParam.speed_feedback_sign * gimbal_ins->Gyro[0];

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:
        GM6020_Stop(yaw_motor);
        GM6020_Stop(pitch_motor);
        break;

    case GIMBAL_IMU_MODE:
        GM6020_Enable(yaw_motor);
        GM6020_Enable(pitch_motor);
        GM6020_SetAngleRef(yaw_motor, ClampMotorTarget(gimbal_cmd_recv.yaw, &GimbalYawParam));
        GM6020_SetAngleRef(pitch_motor, ClampMotorTarget(gimbal_cmd_recv.pitch, &GimbalPitchParam));
        break;

    default:
        break;
    }

    if (gimbal_ins != NULL)
    {
        imu_online = INS_IsOnline();
        memcpy(gimbal_feedback_data.gimbal_imu_data.q, gimbal_ins->q, sizeof(gimbal_feedback_data.gimbal_imu_data.q));
        memcpy(gimbal_feedback_data.gimbal_imu_data.Gyro, gimbal_ins->Gyro, sizeof(gimbal_feedback_data.gimbal_imu_data.Gyro));
        memcpy(gimbal_feedback_data.gimbal_imu_data.Accel, gimbal_ins->Accel, sizeof(gimbal_feedback_data.gimbal_imu_data.Accel));
        gimbal_feedback_data.gimbal_imu_data.Roll = gimbal_ins->Roll;
        gimbal_feedback_data.gimbal_imu_data.Pitch = gimbal_ins->Pitch;
        gimbal_feedback_data.gimbal_imu_data.Yaw = gimbal_ins->Yaw;
        gimbal_feedback_data.gimbal_imu_data.YawTotalAngle = gimbal_ins->YawTotalAngle;
        gimbal_feedback_data.imu_online = imu_online;
    }
    else
    {
        gimbal_feedback_data.imu_online = 0u;
    }
    if (yaw_motor != NULL)
    {
        gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.ecd;
        gimbal_feedback_data.yaw_motor_online = GM6020_IsOnline(yaw_motor, now_tick);
    }
    else
    {
        gimbal_feedback_data.yaw_motor_online = 0u;
    }
    if (pitch_motor != NULL)
    {
        gimbal_feedback_data.pitch_motor_online = GM6020_IsOnline(pitch_motor, now_tick);
    }
    else
    {
        gimbal_feedback_data.pitch_motor_online = 0u;
    }

    PubPushMessage(gimbal_pub, &gimbal_feedback_data);
}
