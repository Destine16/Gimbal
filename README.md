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
- 系统辨识：支持 RTT 采集 yaw / pitch PRBS、fast multisine、阶跃、正弦和 pitch 静态前馈图谱数据

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
yaw:
  angle Kp = 48.0000
  speed Kp = 3600.0000
  speed Ki = 600.0000

pitch:
  angle Kp = 46.0000
  speed Kp = 3100.0000
  speed Ki = 300.0000
  gyro LPF alpha = 0.50

current loop:
  Kp = 0.8
  Ki = 100.0
```

当前安全限幅：

```text
yaw_speed_ref_max = 4.8 rad/s
pitch_speed_ref_max = 3.4 rad/s
current_ref_max = 3800 raw
voltage_cmd_max = 5000 raw
```

pitch 前馈模型：

```text
output_ff = A * sin(theta) + C
A = -1115.8459
C = -97.4462
```

已删除试验效果不好的 speed feedforward 和简单 hysteresis feedforward 开关。方向相关摩擦/线缆项只在静态图谱中作为诊断量，不直接进入当前固件输出。

当前 pitch 参数的性能验证数据和曲线见：

```text
data/sysid/pitch_fast_multisine_oldff_kp46_sp3100_ki300_20260522_182257.csv
data/sysid/pitch_perf_step_oldff_kp46_sp3100_ki300_20260522_182431.csv
data/sysid/pitch_perf_sine_oldff_kp46_sp3100_ki300_20260522_182558.csv
data/sysid/analysis/pitch_fast_multisine_oldff_kp46_sp3100_ki300_20260522_182257_pitch_fast_multisine_fast/
data/sysid/analysis/pitch_perf_step_oldff_kp46_sp3100_ki300_20260522_182431_pitch_perf_step/
data/sysid/analysis/pitch_perf_sine_oldff_kp46_sp3100_ki300_20260522_182558_pitch_perf_sine/
```

本轮 pitch 调参保留 `46 / 3100 / 300` 作为当前默认。试验过的 D、参考速度前馈、更高 angle Kp 和更高 speed Kp 均未保留：它们会引入抖动、输出饱和，或破坏 `+/-3 deg` 小角度静态精度。

本轮详细调参记录见：

```text
docs/control/pitch_tuning_summary_20260522.md
```

## 视觉通信

视觉侧发送固定 10 字节二进制帧：

```text
A5 5A | seq uint8 | target_valid uint8 | delta_yaw int16 | delta_pitch int16 | CRC16/MODBUS
```

单位：

```text
delta_yaw   = int16_t, 0.0001 rad
delta_pitch = int16_t, 0.0001 rad
```

`target_valid = 1` 表示当前有目标；`target_valid = 0` 表示当前无目标，此时视觉侧应把 delta 填 0。

电控侧回传固定 13 字节状态帧：

```text
5A A5 | seq_echo uint8 | yaw_actual int32 | pitch_actual int32 | CRC16/MODBUS
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

哨兵模式行为：

- `target_valid = 0`：当前默认保持当前位置；扫描功能保留在代码中，但默认关闭
- `target_valid = 1`：电控停止扫描，使用视觉 delta 跟踪装甲板
- 目标短暂丢失时先保持上一目标，超过延时后回到无目标保持状态
- 检测到堵转时清对应轴 PID，并向反方向回退一小段后恢复无目标状态

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
  debug/                          RTT 系统辨识输出、普通视觉链路调试输出
  imu/                            BMI088 驱动与 INS 任务
  message_center/                 轻量消息中心
  motor/                          GM6020 驱动、CAN、三环控制
  vision/                         USB CDC 协议解析与状态回传

host_tools/
  vision_jog_gui.py               USB CDC 云台键盘/GUI 测试工具
  vision_step_sender.py           USB CDC 阶跃命令发送工具
  gimbal_sysid_rtt_capture.py     SEGGER RTT 系统辨识采集工具
  vision_debug_rtt_capture.py     SEGGER RTT 普通视觉链路调试采集工具
  analyze_*_sysid.py              系统辨识分析脚本
  optimize_yaw_pid_model.py       yaw PID 模型优化脚本
  analyze_fast_sysid.py           yaw / pitch 快响应辨识评分脚本
  optimize_fast_pid_model.py      yaw / pitch 快响应 PID 模型优化脚本
  compare_fast_pid_results.py     快响应 PID 对比报告生成脚本
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
cmake --preset Debug -DGIMBAL_SYSID_MODE=0 -DVISION_DEBUG_RTT_ENABLE=OFF
cmake --build --preset Debug
```

普通视觉链路 RTT 调试固件：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=0 -DVISION_CONTROL_MODE=1 -DVISION_DEBUG_RTT_ENABLE=ON
cmake --build --preset Debug
```

打开 `VISION_DEBUG_RTT_ENABLE` 后，固件上电运行时会自动向 RTT up-buffer 2 输出 `vision_dbg` 二进制调试帧。不开启时不会输出普通视觉调试日志。

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
GIMBAL_SYSID_MODE=3  yaw 小阶跃验证
GIMBAL_SYSID_MODE=5  yaw 阶跃性能测试
GIMBAL_SYSID_MODE=6  pitch 阶跃性能测试
GIMBAL_SYSID_MODE=7  yaw 正弦跟踪性能测试
GIMBAL_SYSID_MODE=8  pitch 正弦跟踪性能测试
GIMBAL_SYSID_MODE=9  pitch PRBS 辨识
GIMBAL_SYSID_MODE=10 yaw fast multisine 快响应辨识
GIMBAL_SYSID_MODE=11 pitch fast multisine 快响应辨识
GIMBAL_SYSID_MODE=13 pitch 上下行静态前馈图谱辨识
```

示例：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=11
cmake --build --preset Debug
```

快响应辨识推荐使用 preset：

```bash
cmake --preset YawFastMultisineSysid
cmake --build --preset YawFastMultisineSysid

cmake --preset PitchFastMultisineSysid
cmake --build --preset PitchFastMultisineSysid
```

采集 yaw fast multisine：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 55 --kill-conflicts \
  --elf build/YawFastMultisineSysid/Gimbal.elf \
  --output data/sysid/yaw_fast_multisine_$(date +%Y%m%d_%H%M%S).csv
```

采集 pitch fast multisine：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 55 --kill-conflicts \
  --elf build/PitchFastMultisineSysid/Gimbal.elf \
  --output data/sysid/pitch_fast_multisine_$(date +%Y%m%d_%H%M%S).csv
```

采集 pitch 带相机负载的上下行静态前馈图谱：

```bash
cmake --preset PitchStaticFfMapSysid
cmake --build --preset PitchStaticFfMapSysid

.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 125 --kill-conflicts \
  --elf build/PitchStaticFfMapSysid/Gimbal.elf \
  --output data/sysid/pitch_static_ff_map_$(date +%Y%m%d_%H%M%S).csv

.venv-host/bin/python host_tools/analyze_pitch_static_ff_map.py \
  data/sysid/pitch_static_ff_map_YYYYMMDD_HHMMSS.csv
```

这个实验用于判断 pitch 轴是否主要受重力、线缆/摩擦回差限制，并拟合 `A*sin(theta)+B*cos(theta)+C+H*direction` 作为诊断模型。其中 `+/-38deg` 只是让 `+/-35deg` 目标能从两个方向到达的锚点，默认不会参与最终拟合。当前固件默认只回填兼容的 `A*sin(theta)+C` 静态前馈。

分析快响应数据：

```bash
.venv-host/bin/python host_tools/analyze_fast_sysid.py data/sysid/yaw_fast_multisine_YYYYMMDD_HHMMSS.csv
```

基于模型优化 yaw PID：

```bash
.venv-host/bin/python host_tools/optimize_fast_pid_model.py data/sysid/yaw_fast_multisine_YYYYMMDD_HHMMSS.csv \
  --current-angle-kp 48 --current-speed-kp 3600 --current-speed-ki 600 --speed-ref-limit 4.8
```

基于模型优化 pitch PID：

```bash
.venv-host/bin/python host_tools/optimize_fast_pid_model.py data/sysid/pitch_fast_multisine_YYYYMMDD_HHMMSS.csv \
  --current-angle-kp 28 --current-speed-kp 2800 --current-speed-ki 420 --speed-ref-limit 3.6
```

生成对比报告：

```bash
.venv-host/bin/python host_tools/compare_fast_pid_results.py \
  --baseline-analysis data/sysid/analysis/BASELINE_DIR/yaw_fast_multisine_fast_result.json \
  --optimization data/sysid/analysis/OPT_DIR/yaw_fast_multisine_fast_pid_result.json \
  --output data/sysid/analysis/yaw_fast_pid_compare.md
```

普通视觉链路 RTT 调试采集示例：

```bash
.venv-host/bin/python host_tools/vision_debug_rtt_capture.py --duration 60 --kill-conflicts
```

该脚本默认不复位目标板，适合在视觉联调过程中直接附着采集。输出 CSV 默认保存在 `data/debug/`。

实验结束后务必切回普通模式：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=0 -DVISION_DEBUG_RTT_ENABLE=OFF
cmake --build --preset Debug
```

## 实验文档

- [yaw PRBS 与 PID 优化](docs/control/yaw_prbs_sysid_experiment.md)
- [yaw / pitch 快响应系统辨识与 PID 优化](docs/control/fast_pid_sysid_workflow.md)
- [pitch 静态前馈图谱实验](docs/control/pitch_static_ff_map_experiment.md)
- [pitch 调参小结](docs/control/pitch_tuning_summary_20260522.md)
- [云台性能测试报告](Documents/gimbal_performance_report.md)
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
- pitch 静态重力前馈已开启
