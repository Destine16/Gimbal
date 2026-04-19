# Pitch 滞回/摩擦系统辨识实验

## 目的

本实验用于判断 pitch 轴同一角度从不同方向接近时，保持输出是否存在稳定差异。

它和 pitch 重力前馈实验不同：

- 重力前馈实验拟合 `A * sin(theta) + C`。
- 滞回实验拟合 `A * sin(theta) + C + H * approach_sign`。
- `H` 表示方向相关的摩擦/线缆/机构滞回补偿项。

如果 `H` 稳定且显著，后续可以在固件里加 pitch hysteresis feedforward。如果 `H` 每次变化很大，优先检查线缆走线和机械回差。

## 实验模式

编译 pitch 滞回系统辨识固件：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=4
cmake --build --preset Debug
```

关闭系统辨识模式：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=0
cmake --build --preset Debug
```

## 角度序列

实验总时长约 `115s`：

- `0 ~ 5s`：保持启动 pitch 零点，记录基线
- `5 ~ 105s`：执行 pitch 滞回角度序列
- `105 ~ 115s`：回到启动 pitch 零点

pitch 目标 offset 如下，单位为相对启动零点的角度：

```text
0deg
+10deg
+20deg
+30deg
+38deg
+30deg
+20deg
+10deg
0deg
-10deg
-20deg
-30deg
-38deg
-30deg
-20deg
-10deg
0deg
+10deg
+20deg
+30deg
+38deg
+30deg
+20deg
+10deg
0deg
```

每个平台保持 `4s`。分析脚本默认只取每个平台最后 `1s`。

## approach_sign 定义

分析脚本会根据相邻平台目标自动推导：

- `approach_sign = +1`：目标角从小变大，表示从低角度接近当前角度。
- `approach_sign = -1`：目标角从大变小，表示从高角度接近当前角度。
- `approach_sign = 0`：第一段或没有角度变化，不参与滞回拟合。

同一个目标角如果 `+1` 和 `-1` 的保持输出差异很大，就说明存在方向相关滞回。

## 固件行为

- yaw 固定在启动瞬间的 yaw 零点。
- pitch 按上面的滞回角度序列运动。
- pitch 目标经过 `GIMBAL_PITCH_MIN_RAD / GIMBAL_PITCH_MAX_RAD` 软件限位。
- 每次切换平台时会清空 pitch 三环 PID 状态，降低上一个平台积分残留对保持输出的污染。
- RTT 遥测帧仍是 `A6 6A` 开头的 100 字节二进制帧。
- `mode = 4` 表示当前数据来自 pitch 滞回实验。
- `phase = 5` 表示当前处于角度平台阶段。

## 采集方式

运行：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 123 \
  --output data/sysid/pitch_hyst_rtt_$(date +%Y%m%d_%H%M%S).csv
```

如果提示 J-Link 冲突：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 123 --kill-conflicts \
  --output data/sysid/pitch_hyst_rtt_$(date +%Y%m%d_%H%M%S).csv
```

## 分析方式

运行：

```bash
.venv-host/bin/python host_tools/analyze_pitch_hysteresis_sysid.py \
  data/sysid/pitch_hyst_rtt_YYYYMMDD_HHMMSS.csv
```

默认输出目录：

```text
data/sysid/analysis/pitch_hyst_rtt_YYYYMMDD_HHMMSS_pitch_hysteresis/
```

输出文件：

- `pitch_hysteresis_result.json`：完整拟合结果、`H` 候选值、显著性和告警。
- `pitch_hysteresis_stage_summary.csv`：每个平台的稳定段、接近方向、保持输出和残差。
- `pitch_hysteresis_pair_summary.csv`：同一目标角不同接近方向的 spread。
- `pitch_hysteresis_time_series.png`：目标角、实际角、速度和输出时域曲线。
- `pitch_hysteresis_fit.png`：按接近方向区分的拟合曲线。
- `pitch_hysteresis_pair_spread.png`：同角度上下行 spread。

## 判断标准

重点看：

- `H` 的绝对值是否足够大。
- `H` 的 `p_value` 是否小于 `0.05`。
- 加入 `H` 后 `RMSE` 是否明显下降。
- `pitch_hysteresis_pair_summary.csv` 里不同角度的 `hysteresis_half_raw` 是否大致一致。

如果 `H` 大致稳定，可以考虑在固件中加入：

```text
output_ff = A * sin(theta) + C + H * last_pitch_motion_sign
```

如果 `hysteresis_half_raw` 随角度变化很大，或者每次实验差异很大，不建议先写入固件，应优先检查线缆和机械结构。

## 当前辨识结果

`pitch_hyst_rtt_20260419_131053.csv` 的结果：

```text
u_hold = A * sin(theta) + C + H * approach_sign
A = -1115.8459
C = -97.4462
H = -714.2191
R2 = 0.97028
RMSE = 154.716 raw
```

对照结果：

- 当前重力前馈单独使用：`RMSE ~= 788 raw`
- 固定重力只加 `H`：`RMSE ~= 477 raw`
- 重新拟合 `A + C + H`：`RMSE ~= 155 raw`

因此当前固件采用 `A + C + H` 模型。`H` 的方向由角度环输出的 `speed_ref` 决定，只有明显运动时才更新，静止保持时沿用最近一次运动方向。
