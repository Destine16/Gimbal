# Yaw PRBS 系统辨识实验

## 目的

本实验用于重新整定 yaw 轴 PID。实验采用固件内部生成的多级 PRBS 角度目标，主机只负责采集遥测数据，避免 GUI 或 USB 下行时序影响实验输入。

## 实验模式

通过 CMake 开关启用：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=1
cmake --build --preset Debug
```

`GIMBAL_SYSID_MODE=1` 表示 yaw PRBS；`GIMBAL_SYSID_MODE=3` 表示 yaw 小阶跃验证。pitch 当前推荐使用 `GIMBAL_SYSID_MODE=11` 做 fast multisine，或 `GIMBAL_SYSID_MODE=13` 做静态前馈图谱。

关闭系统辨识模式：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=0
cmake --build --preset Debug
```

## Yaw PRBS 设计

实验总时长约 75 秒：

- `0 ~ 5s`：保持 yaw 启动零点，记录基线
- `5 ~ 65s`：执行 yaw 多级 PRBS
- `65 ~ 75s`：回到 yaw 启动零点，观察回零和积分残留

PRBS 使用的 yaw 目标 offset：

```text
-50deg
-35deg
-22deg
-12deg
0deg
+12deg
+22deg
+35deg
+50deg
```

这些值是相对启动零点的目标 offset，不是连续累加 delta：

```text
yaw_ref = yaw_startup_zero + yaw_offset
```

平台保持时间随机取：

```text
0.25s
0.35s
0.50s
0.80s
```

比例约为：

```text
0.25s: 25%
0.35s: 30%
0.50s: 30%
0.80s: 15%
```

## 固件输出

系统辨识模式下，固件通过 SEGGER RTT 输出 `A6 6A` 开头的 100 字节遥测帧。

遥测内容包括：

- `angle_ref_rad`
- `angle_feedback_rad`
- `speed_ref_rad_s`
- `speed_feedback_rad_s`
- `current_ref_raw`
- `current_feedback_raw`
- `voltage_ref_raw`
- `output_ff_raw`
- `output_cmd`
- `real_current`
- 三环 PID 的 `Pout/Iout/Dout/Output`

## RTT 采集

编译并烧录系统辨识固件：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=1
cmake --build --preset Debug
```

RTT 遥测使用 up-buffer 1，名称为 `sysid`。主机侧只运行一个采集脚本，不要同时打开 `JLinkRTTLogger`、`JLinkRTTClient`、Ozone 或 J-Link GDB Server。

运行：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 80 \
  --output data/sysid/yaw_prbs_rtt_$(date +%Y%m%d_%H%M%S).csv
```

如果提示有 J-Link 冲突进程，先关闭这些工具，或让脚本自动关闭：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 80 --kill-conflicts \
  --output data/sysid/yaw_prbs_rtt_$(date +%Y%m%d_%H%M%S).csv
```

默认输出到：

```text
data/sysid/yaw_prbs_rtt_YYYYMMDD_HHMMSS.csv
```

RTT 采集脚本会优先从 `build/Debug/Gimbal.elf` 读取 `_SEGGER_RTT` 地址，并用这个地址直接附着控制块，避免单纯自动扫描时偶发找不到 RTT 控制块。

## 主机依赖

主机侧工具依赖已经整理在：

```text
host_tools/requirements.txt
```

如果需要重新安装：

```bash
.venv-host/bin/python -m pip install -r host_tools/requirements.txt
```

## 实验前提

- 云台上电前手动摆正 yaw / pitch。
- yaw 周围不要有线缆缠绕风险。
- RTT 采集期间不要再运行其他 J-Link 连接工具，避免多个进程抢同一个 J-Link。
- pitch 会保持启动时角度，不参与本轮辨识。

## 数据分析

采集完成后运行：

```bash
.venv-host/bin/python host_tools/analyze_yaw_prbs_sysid.py \
  data/sysid/yaw_prbs_rtt_YYYYMMDD_HHMMSS.csv
```

默认输出目录：

```text
data/sysid/analysis/yaw_prbs_rtt_YYYYMMDD_HHMMSS_yaw_prbs/
```

输出文件包括：

- `yaw_prbs_result.json`：完整分析结果和 PID 候选值
- `yaw_prbs_step_metrics.csv`：每个 PRBS 阶跃段的超调、上升时间、稳态误差、饱和比例
- `yaw_prbs_frequency_response.csv`：闭环频响和相干度
- `yaw_prbs_time_series.png`：目标角、实际角、误差、速度、输出时域曲线
- `yaw_prbs_frequency_response.png`：闭环幅频、相频、相干度
- `yaw_prbs_step_metrics.png`：每个阶跃段的性能指标
- `yaw_prbs_arx_validation.png`：闭环 ARX 模型仿真对比
- `yaw_prbs_summary.png`：核心指标和 PID 候选值摘要

## 关键指标含义

脚本会计算：

- `tracking_rmse_deg`：整个 PRBS 阶段的 yaw 跟踪 RMS 误差，越小越好。
- `tracking_max_abs_deg`：最大绝对跟踪误差，用来发现明显失控或大滞后。
- `output_saturation_ratio`：输出接近限幅的比例，过高说明数据已经被限幅非线性污染。
- `median_overshoot_ratio`：多段阶跃的中位数超调比例。
- `median_rise_time_s`：多段阶跃的中位数 10% 到 90% 上升时间。
- `median_settling_time_s`：多段阶跃的中位数稳定时间。
- `angle_bandwidth_hz`：`yaw_ref -> yaw_actual` 的经验闭环带宽。
- `speed_bandwidth_hz`：`speed_ref -> speed_actual` 的经验速度内环闭环带宽。
- `mean_coherence_0p05_3hz`：频响估计可信度，越接近 1 越好。
- `ARX simulation R2`：闭环模型仿真拟合度，低于约 `0.70` 时不要相信自动 PID 候选。

## PID 候选值逻辑

脚本给出的 PID 不是直接写入固件的最终答案，而是下一轮实验候选值。

计算依据：

- 先从 PRBS 数据估计 `yaw_ref -> yaw_actual` 的闭环带宽。
- 再从 `speed_ref -> speed_actual` 估计速度内环带宽。
- yaw 角度环目标带宽默认不超过速度内环带宽的 `30%`，避免外环逼近内环导致振荡。
- 根据多段阶跃的超调、稳态误差和输出饱和比例，对 `angle Kp / speed Kp / speed Ki` 做受限调整。

判断原则：

- 如果速度内环带宽不够，优先考虑提高 `speed Kp`。
- 如果稳态误差偏大且输出没有饱和，才考虑提高 `speed Ki`。
- 如果超调或饱和明显，避免继续提高 `angle Kp`。
- 如果 ARX 拟合度、相干度很差，说明这组数据不适合直接算 PID，需要重新采集。

当前脚本只给出这三个参数的候选值：

- yaw angle `Kp`
- yaw speed `Kp`
- yaw speed `Ki`

第一轮不优先改 current 环，除非数据表明 current 环自身已经成为主要限制。

## 小阶跃验证

烧录新 yaw PID 后，建议先运行小阶跃验证模式，而不是直接做完整 PRBS。

编译：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=3
cmake --build --preset Debug
```

实验序列：

```text
0deg -> +10deg -> 0deg -> -10deg -> 0deg
```

时间安排：

- `0 ~ 3s`：保持启动 yaw 零点
- 后续每个平台保持 `2s`
- 最后回零保持 `3s`

总时长约 `16s`。pitch 全程保持启动时角度。

采集：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 20 \
  --output data/sysid/yaw_step_rtt_$(date +%Y%m%d_%H%M%S).csv
```

分析：

```bash
.venv-host/bin/python host_tools/analyze_yaw_step_sysid.py \
  data/sysid/yaw_step_rtt_YYYYMMDD_HHMMSS.csv
```

重点看：

- `output_saturation` 是否接近 `0%`
- `median_overshoot` 是否明显小于 `15%`
- 是否有持续振荡
- 回到 `0deg` 后稳态误差是否可以接受

如果小阶跃验证正常，再切回 `GIMBAL_SYSID_MODE=1` 做完整 yaw PRBS 复测。

## 模型优化脚本

如果需要比规则候选更接近“科学计算”，使用模型优化脚本：

```bash
.venv-host/bin/python host_tools/optimize_yaw_pid_model.py \
  data/sysid/yaw_prbs_rtt_YYYYMMDD_HHMMSS.csv \
  --current-angle-kp 12.0 \
  --current-speed-kp 600.0 \
  --current-speed-ki 1600.0
```

这个脚本的计算流程是：

- 从 PRBS 数据辨识 `output_cmd -> speed_feedback` 的 ARX 离散模型。
- 用训练集拟合模型，用验证集检查模型泛化能力。
- 用 `current_ref_raw -> output_cmd` 的经验增益近似当前环固定后的输出比例。
- 在辨识模型上重建 `angle PID -> speed PID -> output_cmd -> yaw speed -> yaw angle` 闭环仿真。
- 定义代价函数，包含跟踪误差、最大误差、输出饱和、输出 RMS 和输出变化率。
- 使用 `scipy.optimize.differential_evolution` 加 `L-BFGS-B` 搜索 `angle Kp / speed Kp / speed Ki`。

默认输出目录：

```text
data/sysid/analysis/yaw_prbs_rtt_YYYYMMDD_HHMMSS_yaw_model_pid/
```

输出文件包括：

- `yaw_model_pid_result.json`：模型、优化目标、训练集/验证集指标和推荐 PID
- `yaw_model_plant_validation.png`：被控对象模型验证图
- `yaw_model_pid_validation.png`：当前 PID 与优化 PID 的模型仿真对比
- `yaw_model_pid_summary.png`：优化摘要

使用判断：

- `plant validation R2` 建议至少大于 `0.65`。
- `current_ref_to_output_cmd r2` 越高越好；如果低于 `0.65`，说明速度环参数优化可信度不足。
- 如果验证集优化代价没有低于当前 PID，不要采用该组参数。
- 该脚本仍然是模型内最优，最终还要重新烧录实测验证。
