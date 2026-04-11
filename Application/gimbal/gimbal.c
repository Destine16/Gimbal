#include "gimbal.h"

#include <string.h>

#include "ins_task.h"
#include "message_center.h"
#include "gm6020.h"
#include "gimbal_params.h"
#include "robot_def.h"

#ifndef GIMBAL_YAW_ONLY_TEST_ENABLE
#define GIMBAL_YAW_ONLY_TEST_ENABLE 0
#endif

#ifndef GIMBAL_PITCH_ONLY_TEST_ENABLE
#define GIMBAL_PITCH_ONLY_TEST_ENABLE 0
#endif

static Publisher_t *gimbal_pub;   // 发布云台反馈数据到 gimbal_feed
static Subscriber_t *gimbal_sub;  // 订阅 robot_cmd 发来的 gimbal_cmd

static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;      // 最近一次收到的云台控制命令
static Gimbal_Upload_Data_s gimbal_feedback_data; // 周期上报给上层/视觉的云台状态
static const INS_t *gimbal_ins;                // 当前 INS 姿态数据只读指针
static GM6020_Instance *yaw_motor;             // yaw 轴电机实例
static GM6020_Instance *pitch_motor;           // pitch 轴电机实例
static float yaw_angle_feedback_rad;           // 映射到 yaw 电机角度环的反馈角,单位 rad
static float yaw_speed_feedback;               // 映射到 yaw 电机速度环的反馈速度,单位 rad/s
static float pitch_angle_feedback_rad;         // 映射到 pitch 电机角度环的反馈角,单位 rad
static float pitch_speed_feedback;             // 映射到 pitch 电机速度环的反馈速度,单位 rad/s

static float ClampMotorTarget(float target, const GimbalMotorParam_s *param)
{
    if ((param == NULL) || !param->angle_limit_enable)
    {
        return target;
    }
    if (target > param->max_angle_rad)
    {
        return param->max_angle_rad;
    }
    if (target < param->min_angle_rad)
    {
        return param->min_angle_rad;
    }
    return target;
}

void GimbalInit(void)
{
    memset(&gimbal_cmd_recv, 0, sizeof(gimbal_cmd_recv));
    memset(&gimbal_feedback_data, 0, sizeof(gimbal_feedback_data));

    // gimbal 模块: 发布 gimbal_feed,订阅 gimbal_cmd
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_ins = INS_GetData();

    // 这里使用“复合字面量 + 指定初始化”现场构造一个匿名 GM6020_Init_Config_s 配置对象,
    // 再通过取地址传给 GM6020_Init(); 这种写法适合只在初始化时使用一次的配置结构体
    yaw_motor = GM6020_Init(&(GM6020_Init_Config_s){
        .can_handle = &hcan2,
        .motor_id = GimbalYawParam.motor_id,
        .angle_feedback_ptr = &yaw_angle_feedback_rad,
        .speed_feedback_ptr = &yaw_speed_feedback,
        .current_feedback_sign = GimbalYawParam.current_feedback_sign,
        .output_sign = GimbalYawParam.output_sign,
        .max_output_raw = GimbalYawParam.max_output_raw,
        .output_ff_sin_raw = GimbalYawParam.output_ff_sin_raw,
        .output_ff_offset_raw = GimbalYawParam.output_ff_offset_raw,
        .angle_pid_config = GimbalYawParam.angle_pid,
        .speed_pid_config = GimbalYawParam.speed_pid,
        .current_pid_config = GimbalYawParam.current_pid,
    });

    pitch_motor = GM6020_Init(&(GM6020_Init_Config_s){
        .can_handle = &hcan2,
        .motor_id = GimbalPitchParam.motor_id,
        .angle_feedback_ptr = &pitch_angle_feedback_rad,
        .speed_feedback_ptr = &pitch_speed_feedback,
        .current_feedback_sign = GimbalPitchParam.current_feedback_sign,
        .output_sign = GimbalPitchParam.output_sign,
        .max_output_raw = GimbalPitchParam.max_output_raw,
        .output_ff_sin_raw = GimbalPitchParam.output_ff_sin_raw,
        .output_ff_offset_raw = GimbalPitchParam.output_ff_offset_raw,
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
    yaw_angle_feedback_rad = GimbalYawParam.angle_feedback_sign * gimbal_ins->YawTotalAngle;
    yaw_speed_feedback = GimbalYawParam.speed_feedback_sign * gimbal_ins->Gyro[2];
    pitch_angle_feedback_rad = GimbalPitchParam.angle_feedback_sign * gimbal_ins->Roll;
    pitch_speed_feedback = GimbalPitchParam.speed_feedback_sign * gimbal_ins->Gyro[0];

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:
        GM6020_Stop(yaw_motor);
        GM6020_Stop(pitch_motor);
        break;

    case GIMBAL_IMU_MODE:
#if GIMBAL_PITCH_ONLY_TEST_ENABLE
        GM6020_Stop(yaw_motor);
        GM6020_Enable(pitch_motor);
        GM6020_SetAngleRef(pitch_motor, ClampMotorTarget(gimbal_cmd_recv.pitch, &GimbalPitchParam));
#else
        GM6020_Enable(yaw_motor);
#if GIMBAL_YAW_ONLY_TEST_ENABLE
        GM6020_Stop(pitch_motor);
#else
        GM6020_Enable(pitch_motor);
#endif
        GM6020_SetAngleRef(yaw_motor, ClampMotorTarget(gimbal_cmd_recv.yaw, &GimbalYawParam));
#if !GIMBAL_YAW_ONLY_TEST_ENABLE
        GM6020_SetAngleRef(pitch_motor, ClampMotorTarget(gimbal_cmd_recv.pitch, &GimbalPitchParam));
#endif
#endif
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
        gimbal_feedback_data.gimbal_imu_data.Pitch = pitch_angle_feedback_rad;
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
