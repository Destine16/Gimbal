# 云台工程

本项目是一个基于 `STM32F405 + BMI088 + GM6020 + USB CDC` 的双轴云台控制工程。

当前工程围绕双轴云台主线组织，核心流程如下：

```text
系统初始化 / 主任务
-> 指令解释与模式决策
-> 云台应用层
-> IMU + 电机 + 视觉通信
```

## 主要功能

- 双轴云台控制：`yaw / pitch`
- BMI088 姿态解算
- Quaternion EKF 四元数滤波
- IMU 角度反馈位置环
- IMU 陀螺仪反馈速度环
- 三环控制链：
  - 位置环 -> 速度环 -> 电流环 -> 最终输出
- USB CDC 视觉通信
- CRC16/MODBUS 协议校验
- IMU 与电机基础在线保护

## 目录结构

```text
Application/
  robot.c                系统入口
  cmd/robot_cmd.c        指令解释与模式决策
  gimbal/gimbal.c        云台应用层
  gimbal/gimbal_params.c 云台参数占位

Modules/
  algorithm/             PID、CRC16、QuaternionEKF、kalman_filter
  imu/                   BMI088 驱动与 INS 任务
  motor/                 GM6020 驱动与三环控制
  vision/                USB CDC 协议解析与状态回传
  message_center/        轻量消息中心
  bsp/                   DWT 计时工具
```

## 控制策略

- 视觉侧下发的是增量角，而不是绝对角
  - `yaw_target = current_yaw + yaw_delta`
  - `pitch_target = current_pitch + pitch_delta`
- yaw 角度反馈来自 IMU 的 `YawTotalAngle`
- pitch 角度反馈来自 IMU 的 `Pitch`
- yaw 角速度反馈来自 IMU 的 `Gyro[2]`
- pitch 角速度反馈来自 IMU 的 `Gyro[0]`
- pitch 目标角在进入电机控制前会经过软件限位

## 编译方式

本工程使用 CMake。

```bash
cmake -S . -B build/Debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug -j
```

生成文件位于：

```text
build/Debug/Gimbal.elf
```

## 当前硬件配置

- MCU：STM32F405
- IMU：BMI088，SPI1
- 电机：2 个 GM6020，CAN1
- 视觉链路：USB CDC

## 上板前需要确认的参数

以下文件中仍包含与机构和接线相关的占位参数：

- `Application/robot_def.h`
- `Application/gimbal/gimbal_params.c`

上板前至少要确认：

- 电机 ID
- yaw / pitch 各环反馈正负号
- 电流反馈正负号
- 最终输出正负号
- pitch 软限位
- 三环 PID 参数

## 文档索引

- [视觉 USB CDC 通信协议](docs/protocol/vision_usb_cdc_protocol.md)
- [IMU 姿态解算说明](docs/control/imu_attitude_estimation.md)
- [Quaternion EKF 数学说明](docs/control/quaternion_ekf_math.md)
- [量测门控与 bias 可观测性](docs/control/measurement_gating_and_bias.md)
