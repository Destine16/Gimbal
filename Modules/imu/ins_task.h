#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "ins_types.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1

typedef struct
{
    float q[4]; // 四元数估计值

    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU量测值
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    uint32_t update_tick;
    uint8_t init;
} INS_t;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

/**
 * @brief 初始化惯导解算系统
 *
 */
const INS_t *INS_Init(void);
const INS_t *INS_GetData(void);
uint8_t INS_IsOnline(void);

/**
 * @brief 此函数放入实时系统中,以1kHz频率运行
 *        p.s. osDelay(1);
 *
 */
void INS_Task(void);

/**
 * @brief 机体系到惯性系的变换函数
 *
 * @param vecBF body frame
 * @param vecEF earth frame
 * @param q
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

/**
 * @brief 惯性系转换到机体系
 *
 * @param vecEF
 * @param vecBF
 * @param q
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif
