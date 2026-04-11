#ifndef GM6020_H
#define GM6020_H

#include "can.h"
#include "controller.h"
#include "daemon.h"
#include "general_def.h"
#include "stdint.h"

#define GM6020_MAX_NUM 8u
// GM6020 CAN 协议和特性参数规格限幅,不要用调参值覆盖这些硬边界。
#define GM6020_ENCODER_RAW_RANGE          8192.0f
#define GM6020_VOLTAGE_CMD_MAX_RAW        25000.0f
#define GM6020_VOLTAGE_CMD_MAX_I16        25000
#define GM6020_TORQUE_CURRENT_CMD_MAX_RAW 16384.0f
#define GM6020_RATED_TORQUE_SPEED_MAX_RAD_S (132.0f * RPM_2_RAD_PER_SEC)
#define GM6020_NO_LOAD_SPEED_MAX_RAD_S       (320.0f * RPM_2_RAD_PER_SEC)
#define GM6020_ECD_TO_RAD (PI2 / GM6020_ENCODER_RAW_RANGE)

typedef struct
{
    uint16_t ecd;                  // 当前单圈编码器值
    uint16_t last_ecd;             // 上一帧单圈编码器值
    float angle_single_round_rad;  // 单圈角度,单位 rad
    float total_angle_rad;         // 累计总角度,调试器里常看的角度反馈量
    int32_t total_round;           // 已跨过的整圈数
    float speed_rad_s;             // 当前转速,单位 rad/s,调试器里常看的速度反馈量
    int16_t real_current;          // 电机反馈电流,调试器里常看的电流反馈量
    uint8_t temperature;           // 电机温度反馈
} GM6020_Measure_s;

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint8_t motor_id;
    float *angle_feedback_ptr;
    float *speed_feedback_ptr;
    float current_feedback_sign;
    float output_sign;
    float max_output_raw;
    float output_ff_sin_raw;
    float output_ff_offset_raw;
    PID_Init_Config_s angle_pid_config;
    PID_Init_Config_s speed_pid_config;
    PID_Init_Config_s current_pid_config;
} GM6020_Init_Config_s;

typedef struct
{
    GM6020_Measure_s measure;

    CAN_HandleTypeDef *can_handle;
    uint8_t motor_id;
    uint8_t tx_group;
    uint8_t tx_index;
    uint8_t enabled;
    uint32_t last_rx_tick;
    DaemonInstance *daemon;

    float *angle_feedback_ptr;
    float *speed_feedback_ptr;
    float current_feedback_sign;
    float output_sign;
    float max_output_raw;
    float output_ff_sin_raw;
    float output_ff_offset_raw;

    PIDInstance angle_pid;   // 运行时角度环 PID 实例; 在线调参应改这一份,而不是参数模板
    PIDInstance speed_pid;   // 运行时速度环 PID 实例; 调试器里可看 Err/Pout/Iout/Dout/Output
    PIDInstance current_pid; // 运行时电流环 PID 实例

    float angle_ref_rad; // 当前目标角度,调试器里常看的设定值
    int16_t output_cmd;  // 本轮最终发给电机的输出命令
} GM6020_Instance;

void GM6020_CAN_Init(CAN_HandleTypeDef *hcan);
GM6020_Instance *GM6020_Init(const GM6020_Init_Config_s *config);
void GM6020_SetAngleRef(GM6020_Instance *motor, float angle_rad);
void GM6020_Enable(GM6020_Instance *motor);
void GM6020_Stop(GM6020_Instance *motor);
uint8_t GM6020_IsOnline(const GM6020_Instance *motor, uint32_t now_tick);
void GM6020_ControlAll(void);
void GM6020_RxFifo0Callback(CAN_HandleTypeDef *hcan);

#endif
