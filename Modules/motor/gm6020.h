#ifndef GM6020_H
#define GM6020_H

#include "can.h"
#include "controller.h"
#include "daemon.h"
#include "stdint.h"

#define GM6020_MAX_NUM 8u
#define GM6020_ECD_TO_DEG (360.0f / 8192.0f)

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;
    float angle_single_round_deg;
    float total_angle_deg;
    int32_t total_round;
    float speed_deg_s;
    int16_t real_current;
    uint8_t temperature;
} GM6020_Measure_s;

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint8_t motor_id;
    float *angle_feedback_ptr;
    float *speed_feedback_ptr;
    float current_feedback_sign;
    float output_sign;
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

    PIDInstance angle_pid;
    PIDInstance speed_pid;
    PIDInstance current_pid;

    float angle_ref_deg;
    int16_t output_cmd;
} GM6020_Instance;

void GM6020_CAN_Init(CAN_HandleTypeDef *hcan);
GM6020_Instance *GM6020_Init(const GM6020_Init_Config_s *config);
void GM6020_SetAngleRef(GM6020_Instance *motor, float angle_deg);
void GM6020_Enable(GM6020_Instance *motor);
void GM6020_Stop(GM6020_Instance *motor);
uint8_t GM6020_IsOnline(const GM6020_Instance *motor, uint32_t now_tick);
void GM6020_ControlAll(void);
void GM6020_RxFifo0Callback(CAN_HandleTypeDef *hcan);

#endif
