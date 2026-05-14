# 云台项目报告书

# 基础功能展示

## 1.1 整体展示

本项目实现了基于 `STM32F405 + BMI088 + GM6020` 的双轴云台控制。云台通过 IMU 解算出的姿态角和角速度进行闭环控制，yaw 与 pitch 均不依赖 GM6020 编码器作为主反馈。

当前控制链路为：

```text
视觉 delta yaw / delta pitch -> 电控生成绝对目标角 -> 角度环 P -> 速度环 PI -> 电流/电压环 PI -> GM6020 电压输出
```

## 1.2 哨兵/自瞄切换

视觉与电控通过 USB CDC 通信。视觉侧发送 `target_valid` 和角度增量：

```text
target_valid = 1：当前识别到装甲板，电控进入自瞄跟踪
target_valid = 0：当前未识别到装甲板，电控进入无目标状态
```

当前工程保留了哨兵扫描状态机：无目标时可执行 yaw / pitch 扫描，识别到目标后切换为视觉跟踪。

## 1.3 软件限位

pitch 轴软件限位：

```text
pitch_min = -0.73303829 rad ≈ -42 deg
pitch_max = +0.73303829 rad ≈ +42 deg
```

| 限幅项目 | 当前值 |
|---|---:|
| yaw 速度参考限幅 | 4.8 rad/s |
| pitch 速度参考限幅 | 3.6 rad/s |
| 速度环输出限幅 | 3800 raw |
| 电压输出限幅 | 5000 raw |
| pitch 软件角度限位 | ±42 deg |

# 代码介绍

## 2.1 代码架构

| 模块 | 作用 |
|---|---|
| `Application/robot.c` | 系统初始化入口 |
| `Application/robot_task.c` | FreeRTOS 任务调度入口 |
| `Application/cmd/robot_cmd.c` | 模式决策、视觉命令解释、系统辨识接入 |
| `Application/gimbal/gimbal.c` | 云台应用层控制 |
| `Application/gimbal/gimbal_params.c` | 云台 PID、限幅、前馈参数 |
| `Application/sysid/` | 系统辨识实验模式与 RTT 遥测 |
| `Modules/imu/` | BMI088 驱动与 INS 姿态解算 |
| `Modules/motor/` | GM6020 驱动、CAN、三环控制 |

## 2.2 控制逻辑

云台上电时会记录当前 yaw / pitch 作为启动零点。视觉侧发送的 `delta_yaw` 和 `delta_pitch` 会被电控消费一次，并转换为新的绝对目标角。

| 字段 | 含义 |
|---|---|
| `seq` | 帧序号 |
| `target_valid` | 是否识别到目标 |
| `delta_yaw` | yaw 增量，单位 0.0001 rad |
| `delta_pitch` | pitch 增量，单位 0.0001 rad |
| `CRC16` | CRC16/MODBUS 校验 |

## 2.3 控制参数

| 轴 | 角度环 Kp | 速度环 Kp | 速度环 Ki | 速度反馈滤波 | 备注 |
|---|---:|---:|---:|---|---|
| yaw | 48.0000 | 3600.0000 | 600.0000 | 无额外低通 | 无重力前馈，偏向快速响应 |
| pitch | 28.0000 | 2800.0000 | 420.0000 | gyro LPF alpha = 0.50 | 重力前馈开启，速度/摩擦动态前馈关闭 |

pitch 重力前馈模型：

```text
output_ff = A * sin(theta) + C
A = -1115.8459
C = -97.4462
```

# 云台性能展示

数据来源：`data/perf`  
采集方式：J-Link RTT  
数据更新时间：2026-05-13

## 3.1 阶跃响应指标

### yaw

| 阶跃幅度 | 上升时间 tr | 峰值时间 tp | 调节时间 ts | 超调量 | 稳态误差 |
|---:|---:|---:|---:|---:|---:|
| +3 deg | 0.0143 | 0.1590 | 0.0490 | 1.60% | 0.1073 |
| +5 deg | 0.0103 | 0.0700 | 0.1700 | 23.76% | 0.0713 |
| +10 deg | 0.0149 | 0.0990 | 0.2390 | 52.76% | 0.1107 |
| +20 deg | 0.0382 | 0.1390 | N/A | 12.85% | 0.3785 |

### pitch

| 阶跃幅度 | 上升时间 tr | 峰值时间 tp | 调节时间 ts | 超调量 | 稳态误差 |
|---:|---:|---:|---:|---:|---:|
| +3 deg | N/A | 2.1490 | N/A | 0.00% | 0.2975 |
| -3 deg | 0.2026 | 2.3000 | 0.1200 | 4.67% | 0.0346 |
| -10 deg | 0.0825 | 0.1800 | 0.2450 | 8.39% | 0.2077 |
| +10 deg | 0.5504 | 1.9000 | 2.0190 | 0.00% | 0.2829 |

## 3.2 跟踪性能

测试模型：

```text
A * sin(2*pi*t/T)
```

### yaw

| 测试项 | MAE | RMSE | Max Error |
|---|---:|---:|---:|
| A = 5 deg, T = 1 s | 0.5730 | 0.6229 | 0.9497 |
| A = 20 deg, T = 2 s | 1.0735 | 1.1912 | 3.5195 |

### pitch

| 测试项 | MAE | RMSE | Max Error |
|---|---:|---:|---:|
| A = 5 deg, T = 1 s | 1.0959 | 1.2022 | 2.0185 |
| A = 20 deg, T = 2 s | 1.9514 | 2.1314 | 3.4727 |

## 3.3 汇总评分

| 项目 | 分数 |
|---|---:|
| yaw 阶跃 | 18.8265 |
| yaw 正弦跟踪 | 6.6853 |
| pitch 阶跃 | 13.0405 |
| pitch 正弦跟踪 | 11.8934 |
| 总分 | 50.4457 |

## 3.4 简要结论

- 当前参数目标偏向“响应更快”，因此 yaw 允许一定超调。
- yaw 正弦跟踪性能明显优于 pitch，主要因为 yaw 无重力项与线缆拖拽影响。
- pitch 本轮优化后阶跃响应明显变快，`-10 deg` 和 `+10 deg` 均已能计算出上升时间。
- pitch 正弦跟踪相比更保守参数略有变差，这是“更快响应”和“平滑跟踪误差”之间的取舍。
- 当前 pitch 工程折中为：`angle Kp = 28`，`speed Kp = 2800`，`speed Ki = 420`，`gyro LPF alpha = 0.50`，只保留重力前馈，关闭速度/摩擦动态前馈。

# 性能曲线图汇总

当前 MCP 应用上传图片时返回 `403`，说明缺少图片上传权限。因此本文件先更新文字和表格数据，曲线图片需后续手动替换。

最新曲线位于：

| 曲线 | 本地路径 |
|---|---|
| yaw +3 deg 阶跃 | `data/perf/analysis/yaw_perf_step/step_p3deg.png` |
| yaw +5 deg 阶跃 | `data/perf/analysis/yaw_perf_step/step_p5deg.png` |
| yaw +10 deg 阶跃 | `data/perf/analysis/yaw_perf_step/step_p10deg.png` |
| yaw +20 deg 阶跃 | `data/perf/analysis/yaw_perf_step/step_p20deg.png` |
| pitch +3 deg 阶跃 | `data/perf/analysis/pitch_perf_step/step_p3deg.png` |
| pitch -3 deg 阶跃 | `data/perf/analysis/pitch_perf_step/step_m3deg.png` |
| pitch -10 deg 阶跃 | `data/perf/analysis/pitch_perf_step/step_m10deg.png` |
| pitch +10 deg 阶跃 | `data/perf/analysis/pitch_perf_step/step_p10deg.png` |
| yaw A=5deg T=1s 正弦 | `data/perf/analysis/yaw_perf_sine/sine_A5deg_T1s.png` |
| yaw A=20deg T=2s 正弦 | `data/perf/analysis/yaw_perf_sine/sine_A20deg_T2s.png` |
| pitch A=5deg T=1s 正弦 | `data/perf/analysis/pitch_perf_sine/sine_A5deg_T1s.png` |
| pitch A=20deg T=2s 正弦 | `data/perf/analysis/pitch_perf_sine/sine_A20deg_T2s.png` |
