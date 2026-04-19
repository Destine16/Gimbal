#include "ins_task.h"
#include <string.h>
#include "QuaternionEKF.h"
#include "spi.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "daemon.h"
#include "general_def.h"
#include "robot_def.h"

static INS_t INS = {
    .q = {1.0f, 0.0f, 0.0f, 0.0f},
};
static IMU_Param_t IMU_Param;
static DaemonInstance *ins_daemon_instance; // INS 更新在线监测

// 预留派生量所需的机体系基向量; 当前工程主线暂未使用,先注释保留
#if 0
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
#endif

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0;

static void BMI088_ToGimbalFrame(const float bmi088_vec[3], float gimbal_vec[3]);
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);
static float Dot3(const float a[3], const float b[3]);
static void Cross3(const float a[3], const float b[3], float out[3]);
static void Normalize3(float v[3]);

// 将 INS 离线超时(毫秒)换算成 daemon 任务需要递减的计数值
static uint16_t INS_DaemonReloadCount(void)
{
    // 这里不能直接写成 timeout / period,因为整数除法会向下取整:
    // 例如 timeout=21ms、period=10ms 时,直接相除会得到 2,实际只等 20ms 就判离线,比配置更早
    // 因此这里用 (a + b - 1) / b 做向上取整,保证实际允许离线时间不少于配置值
    uint32_t count = (GIMBAL_IMU_OFFLINE_TIMEOUT_MS + DAEMON_TASK_PERIOD_MS - 1u) / DAEMON_TASK_PERIOD_MS;
    // 至少返回 1,避免出现“注册后永远无法上线”的 0 计数
    return (uint16_t)((count == 0u) ? 1u : count);
}

// 初始化姿态四元数的定义:
// 1. 导航系 n 取“重力方向为 +Zn”,即静止时理想重力单位向量记为 (0,0,1)
// 2. INS.q / init_q4 表示“机体系 b -> 导航系 n”的旋转四元数,即对任意机体系向量 v_b 有:
//      v_n = q * v_b * q^{-1}
//    后续 BodyFrameToEarthFrame() / EarthFrameToBodyFrame() 都按这个方向约定使用
// 3. 启动时仅凭加速度计可确定重力方向,因此只能确定 b 系相对 n 系的 Roll / Pitch;
//    绝对航向无法由重力唯一确定,所以这里默认初始 Yaw = 0
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系 n 中定义的单位重力方向
    float axis_rot[3] = {0};           // 将机体系下观测到的重力方向旋到导航系目标重力方向所需的旋转轴
    // 连续读取并平均加速度,得到启动瞬间机体系下的重力方向估计
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);
        float accel_gimbal[3];
        BMI088_ToGimbalFrame(BMI088.Accel, accel_gimbal);
        acc_init[X] += accel_gimbal[X];
        acc_init[Y] += accel_gimbal[Y];
        acc_init[Z] += accel_gimbal[Z];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    // 归一化后,acc_init 表示机体系 b 中观测到的单位重力方向
    Normalize3(acc_init);
    // 下面构造一个“把 acc_init 旋到 gravity_norm”的轴角 (axis_rot, angle),再转成四元数;
    // 按轴角转四元数公式 q = [cos(angle/2), axis_rot * sin(angle/2)] 得到的就是 b -> n 初始姿态
    float angle = acosf(Dot3(acc_init, gravity_norm)); // 两个重力方向之间的夹角
    Cross3(acc_init, gravity_norm, axis_rot);          // 由叉积得到从 acc_init 旋到 gravity_norm 的旋转轴
    Normalize3(axis_rot);                              // 轴角转四元数前先将旋转轴归一化
    // 轴角 -> 四元数,作为 EKF 初始姿态
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 3; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f);
}

const INS_t *INS_Init(void)
{
    BMI088_Error_t init_error;
    float init_quaternion[4] = {0};

    if (INS.init)
        return &INS;

    INS.init_attempt_count++;
    BMI088_AsyncDisable();

    init_error = BMI088Init(&hspi1);
    INS.init_error = init_error;
    if (init_error != BMI088_NO_ERROR)
    {
        return NULL;
    }

    // IMU 安装/比例修正参数: 当前保持单位比例和零安装角,等效于不做额外修正
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    InitQuaternion(init_quaternion);
    // 基于 static.csv 的静态噪声估计,这里先采用更稳的工程初值:
    // Q1/Q2/R 贴近静态数据量级,同时给出保守的 lambda/lpf 便于后续继续动态整定。
    IMU_QuaternionEKF_Init(init_quaternion, 1e-7f, 1e-7f, 1e-5f, 0.9996f, 0.0085f);
#if !IMU_ONLY_BRINGUP_ENABLE
    BMI088_AsyncEnable();
#endif
    if (ins_daemon_instance == NULL)
    {
        // 这里同样是“复合字面量 + 指定初始化”:
        // 现场构造一个匿名 Daemon_Init_Config_s 配置对象,再把它的地址传给 DaemonRegister()
        // 仅在第一次 INS_Init() 时注册一次 daemon:
        // init_count = 0 表示初始先视为离线,后续只有 INS_Task() 正常更新并喂狗后才会上线
        ins_daemon_instance = DaemonRegister(&(Daemon_Init_Config_s){
            .reload_count = INS_DaemonReloadCount(), // INS 允许丢失更新的最大计数
            .init_count = 0u,                        // 注册后默认离线,等待首次有效更新
            .callback = NULL,                        // 当前不额外绑定 INS 离线回调
            .owner_id = &INS,                        // 将该 daemon 实例与 INS 模块关联
        });
    }

    // noise of accel is relatively big and of high freq,thus lpf is used
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
    INS.init = 1;
    INS.init_error = BMI088_NO_ERROR;
    return &INS;
}

const INS_t *INS_GetData(void)
{
    return &INS;
}

// 只有 INS 已完成初始化,且其 daemon 计数未超时,才认为当前 IMU 在线
uint8_t INS_IsOnline(void)
{
    return (uint8_t)(INS.init && DaemonIsOnline(ins_daemon_instance));
}

/* 注意以1kHz的频率运行此任务 */
void INS_Task(void)
{
#if !IMU_ONLY_BRINGUP_ENABLE
    static uint32_t last_bmi088_seq = 0; // 上一次已消费的 BMI088 样本序号,用于避免重复处理旧样本
    uint32_t sample_seq = 0;             // 与 sample 对应的样本序号
#endif
    IMU_Data_t sample;

    if (!INS.init)
        return;

#if IMU_ONLY_BRINGUP_ENABLE
    // IMU-only bring-up 目标是尽快稳定采到 BMI088 数据并输出 RTT 日志。
    // 这里直接走阻塞式 BMI088_Read(),绕开 EXTI/DMA 依赖,避免因为中断线或 DMA 状态机问题导致“有初始化、没样本”。
    BMI088_Read(&BMI088);
    sample = BMI088;
    dt = DWT_GetDeltaT(&INS_DWT_Count);
#else
    // EXTI data-ready 正常时这一步基本没有额外成本;
    // 若自制板的 BMI088 中断线没接好,这里的轮询兜底仍会推动 DMA 采样继续运行。
    BMI088_AsyncPoll();

    // 这里按 || 从左到右求值:
    // 1. 先执行 BMI088_FetchData(&sample, &sample_seq),函数内部会顺手把当前样本序号写入 sample_seq
    // 2. 若左边已经为真(还没有有效样本),则右边短路不再判断
    // 3. 只有左边为假时,才继续比较 sample_seq 是否仍等于上一次已消费的序号
    if (!BMI088_FetchData(&sample, &sample_seq) || sample_seq == last_bmi088_seq)
        return;
    // 只在真正消费到一帧新样本后再更新时间步长,这样 dt 才表示“上一次有效样本到这一次有效样本”的实际间隔
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    // 记录本轮已消费到的最新样本序号,避免下一次重复处理同一帧数据
    last_bmi088_seq = sample_seq;
#endif

    BMI088_ToGimbalFrame(sample.Accel, INS.Accel);
    BMI088_ToGimbalFrame(sample.Gyro, INS.Gyro);

    // 用于修正安装误差; 当前参数为单位变换
    IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

    // 预留扩展: 当前工程未启用这两个角度量,这里只保留计算入口注释
    // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]);
    // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]);

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 预留派生量: 当前工程上层未消费机体系基向量和去重力后的运动加速度,先注释保留
#if 0
    const float gravity[3] = {0, 0, 9.81f}; // 导航系 n 中定义的重力向量
    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
    {
        INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
#endif

    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
    INS.update_tick = HAL_GetTick();
    // 成功形成一帧新的姿态结果后喂狗
    DaemonReload(ins_daemon_instance);
}

static void BMI088_ToGimbalFrame(const float bmi088_vec[3], float gimbal_vec[3])
{
    // ACE 主控安装: 相机前方 = BMI088 -X, 上方 = BMI088 +Z.
    // 常见机器人右手坐标: +X 为前方, +Y 为左方, +Z 为上方.
    // 在这个坐标中 +Yaw 为从上往下看逆时针; 业务 +Pitch 的抬头约定由 robot_def.h 的符号层处理.
    gimbal_vec[X] = -bmi088_vec[X];
    gimbal_vec[Y] = -bmi088_vec[Y];
    gimbal_vec[Z] = bmi088_vec[Z];
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @note           预留坐标变换工具函数; 当前工程主线暂未使用,先保留
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @note           预留坐标变换工具函数; 当前工程主线暂未使用,先保留
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 * @note  当前函数在 INS_Task() 中实际会被调用; 只是当前参数设置为单位比例和零安装角,效果上等效于不修正
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw);
        cosPitch = arm_cos_f32(param->Pitch);
        cosRoll = arm_cos_f32(param->Roll);
        sinYaw = arm_sin_f32(param->Yaw);
        sinPitch = arm_sin_f32(param->Pitch);
        sinRoll = arm_sin_f32(param->Roll);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

// 三维向量点积; 若两向量已归一化,结果即为夹角余弦
static float Dot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// 三维向量叉积,结果方向满足右手定则
static void Cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// 将三维向量归一化为单位向量; 过小时不处理以避免除零
static void Normalize3(float v[3])
{
    float norm = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (norm > 1e-6f)
    {
        v[0] /= norm;
        v[1] /= norm;
        v[2] /= norm;
    }
}
