# IMU 姿态解算说明

## 作用

本工程使用 BMI088 数据和四元数 EKF 来估计云台姿态，再把姿态结果作为云台控制环的反馈来源。

## 状态与量测

EKF 状态是 6 维：

```text
x = [q0, q1, q2, q3, bx, by]^T
```

- `q0..q3`：四元数
- `bx, by`：陀螺仪零偏估计

量测是 3 维：

```text
z = 归一化后的加速度方向
```

也就是说，在 EKF 更新阶段，加速度计不是按“原始模长量测”来用，而是按“重力方向观测”来用。

## 处理链路

IMU 主线如下：

```text
BMI088_Read()
-> INS_Task()
-> IMU_QuaternionEKF_Update()
-> 输出 q / yaw / pitch / roll / yaw_total
```

主要代码位置：

- BMI088 驱动：
  - `Modules/imu/BMI088driver.c`
- INS 任务：
  - `Modules/imu/ins_task.c`
- Quaternion EKF：
  - `Modules/algorithm/QuaternionEKF.c`

## 初始化过程

启动时依次完成：

1. 初始化 BMI088
2. 在线标定陀螺仪零偏
3. 根据静止重力模长修正加速度比例
4. 由静止加速度方向计算初始四元数
5. 用该四元数初始化 EKF

对应代码位置：

- `INS_Init()`
- `InitQuaternion()`
- `IMU_QuaternionEKF_Init()`

## 运行时更新

每轮更新大致做：

1. 读取 BMI088 陀螺仪和加速度
2. 如有需要，做安装误差修正
3. 更新 Quaternion EKF
4. 由四元数反解 `Roll / Pitch / Yaw`
5. 计算 `YawTotalAngle`
6. 可选地减去重力，得到运动加速度

## 在云台控制中的作用

当前云台应用把 IMU 输出作为外环反馈：

- yaw 角度反馈：`YawTotalAngle`
- pitch 角度反馈：`Pitch`
- yaw 角速度反馈：`Gyro[2]`
- pitch 角速度反馈：`Gyro[0]`

对应代码位置：

- `Application/gimbal/gimbal.c`

## 主反馈来源

外环主反馈使用 IMU 姿态，电机编码器不直接作为外环主反馈。

主要考虑如下：

- 姿态量本身在物理意义上更直接
- yaw / pitch 的角速度反馈可以直接来自陀螺仪

电机反馈仍然需要保留，用于：

- 电机在线检测
- 最内层电流环
- 编码器相关辅助量

## 约束

- EKF 只估计 `bx` 和 `by`
- `bz` 当前不参与主动估计
- 纯重力观测对 yaw 的约束有限
