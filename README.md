# 云台工程

本项目是一个基于 `STM32F405 + BMI088 + GM6020 + USB CDC` 的双轴云台控制工程。

当前主线目标是让云台通过 IMU 反馈闭环控制 yaw / pitch，并通过 USB CDC 接收视觉侧发送的 `delta_yaw / delta_pitch` 增量命令。

## 当前状态

- MCU：`STM32F405`
- IMU：`BMI088`
- 电机：`GM6020 * 2`
- CAN：`CAN2`
- yaw 电机 ID：`2`
- pitch 电机 ID：`4`
- 视觉链路：`USB CDC`
- 控制模式：IMU 角度反馈 + IMU 角速度反馈
- 系统辨识：支持 RTT 采集 yaw PRBS、yaw step、pitch 前馈、pitch 滞回数据

## 坐标与方向约定

云台业务坐标定义：

- `+yaw`：从上往下看逆时针
- `+pitch`：相机抬头

反馈来源：

- yaw 角度反馈：`INS.YawTotalAngle`
- yaw 速度反馈：`INS.Gyro[2]`
- pitch 角度反馈：`INS.Roll` 映射后的 pitch 轴角度
- pitch 速度反馈：`INS.Gyro[0]`

pitch 软件限位：

```text
min = -0.73303829 rad 约 -42 deg
max = +0.73303829 rad 约 +42 deg
```

方向、电机 ID、限位集中在：

```text
Application/robot_def.h
Application/gimbal/gimbal_params.c
```

## 控制链路

电机控制链路：

```text
角度环 P -> 速度环 PI -> 电流/电压环 PI -> GM6020 voltage output
```

当前 yaw / pitch PID 参数：

```text
angle Kp = 16.4788
speed Kp = 1318.3296
speed Ki = 486.4756
current Kp = 0.8
current Ki = 100.0
```

当前安全限幅：

```text
speed_ref_max = 2.0 rad/s
current_ref_max = 3000 raw
voltage_cmd_max = 5000 raw
```

pitch 前馈模型：

```text
output_ff = A * sin(theta) + C + H * motion_sign
A = -1115.8459
C = -97.4462
H = -714.2191
```

其中 `motion_sign` 由 pitch 角度环输出的 `speed_ref` 决定，带 `0.10 rad/s` 死区，避免静止噪声反复切换方向。

## 视觉通信

视觉侧发送固定 8 字节二进制帧：

```text
A5 5A | delta_yaw int16 | delta_pitch int16 | CRC16/MODBUS
```

单位：

```text
delta_yaw   = int16_t, 0.0001 rad
delta_pitch = int16_t, 0.0001 rad
```

当前默认控制模式是事件目标模式：

```text
VISION_CONTROL_MODE = 1
```

含义：

- 每收到一帧合法视觉命令，只消费一次 delta
- 固件把 delta 转成新的绝对目标
- 新目标持续保持到下一帧合法命令到来
- 适合视觉侧低频发送，不要求连续高速发送

协议细节见：

```text
docs/protocol/vision_usb_cdc_protocol.md
```

## 目录结构

```text
Application/
  robot.c                         系统初始化入口
  robot_task.c                    FreeRTOS 任务调度入口
  cmd/robot_cmd.c                 模式决策、视觉命令解释、系统辨识接入
  gimbal/gimbal.c                 云台应用层
  gimbal/gimbal_params.c          云台 PID、限幅、前馈参数
  sysid/                          系统辨识实验模式与 RTT 遥测帧

Modules/
  algorithm/                      PID、CRC16、QuaternionEKF、Kalman Filter
  bsp/                            DWT 计时工具
  debug/                          RTT 系统辨识输出
  imu/                            BMI088 驱动与 INS 任务
  message_center/                 轻量消息中心
  motor/                          GM6020 驱动、CAN、三环控制
  vision/                         USB CDC 协议解析与状态回传

host_tools/
  vision_jog_gui.py               USB CDC 云台键盘/GUI 测试工具
  vision_step_sender.py           USB CDC 阶跃命令发送工具
  gimbal_sysid_rtt_capture.py     SEGGER RTT 系统辨识采集工具
  analyze_*_sysid.py              系统辨识分析脚本
  optimize_yaw_pid_model.py       yaw PID 模型优化脚本
```

## 编译

推荐使用 CMake preset：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

生成文件：

```text
build/Debug/Gimbal.elf
```

普通云台固件应确认：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=0
cmake --build --preset Debug
```

## 烧录

使用 J-Link 烧录 ELF：

```bash
JLinkExe -device STM32F405RG -if SWD -speed 4000 -autoconnect 1
```

进入 J-Link Commander 后执行：

```text
r
h
loadfile build/Debug/Gimbal.elf
r
g
q
```

## 主机工具

安装依赖：

```bash
.venv-host/bin/python -m pip install -r host_tools/requirements.txt
```

启动 USB CDC GUI：

```bash
.venv-host/bin/python host_tools/vision_jog_gui.py
```

如果同时插了 J-Link 和主控 USB CDC，建议使用 `/dev/serial/by-id/...STMicroelectronics...` 这类稳定设备名，不要盲目使用 `/dev/ttyACM0`。

## 系统辨识模式

系统辨识模式通过 CMake 参数切换：

```text
GIMBAL_SYSID_MODE=0  关闭系统辨识，普通云台固件
GIMBAL_SYSID_MODE=1  yaw PRBS 辨识
GIMBAL_SYSID_MODE=2  pitch 重力前馈辨识
GIMBAL_SYSID_MODE=3  yaw 小阶跃验证
GIMBAL_SYSID_MODE=4  pitch 滞回/摩擦辨识
```

示例：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=4
cmake --build --preset Debug
```

RTT 采集示例：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 123 --kill-conflicts \
  --output data/sysid/pitch_hyst_rtt_$(date +%Y%m%d_%H%M%S).csv
```

实验结束后务必切回普通模式：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=0
cmake --build --preset Debug
```

## 实验文档

- [yaw PRBS 与 PID 优化](docs/control/yaw_prbs_sysid_experiment.md)
- [pitch 重力前馈辨识](docs/control/pitch_feedforward_sysid_experiment.md)
- [pitch 滞回/摩擦辨识](docs/control/pitch_hysteresis_sysid_experiment.md)
- [视觉 USB CDC 通信协议](docs/protocol/vision_usb_cdc_protocol.md)
- [云台坐标系与方向约定](docs/control/gimbal_coordinate_and_direction.md)
- [IMU 姿态解算说明](docs/control/imu_attitude_estimation.md)
- [Quaternion EKF 数学说明](docs/control/quaternion_ekf_math.md)
- [量测门控与 bias 可观测性](docs/control/measurement_gating_and_bias.md)

## 上板检查清单

上板前至少确认：

- `GIMBAL_SYSID_MODE=0`
- yaw 电机 ID 为 `2`
- pitch 电机 ID 为 `4`
- CAN 使用 `CAN2`
- pitch 限位没有超过机械限位
- IMU 方向和云台业务坐标一致
- USB CDC 设备名选中的是主控板，不是 J-Link
- pitch 前馈和滞回补偿打开后没有明显跳变

