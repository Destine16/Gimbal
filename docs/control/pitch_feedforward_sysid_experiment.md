# Pitch 前馈系统辨识实验

## 目的

本实验用于重新估计 pitch 轴重力前馈参数。实验采用固件内部生成的静态 pitch 角度阶梯，主机只通过 SEGGER RTT 采集遥测数据。

## 实验模式

编译系统辨识固件：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=2
cmake --build --preset Debug
```

关闭系统辨识模式：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=0
cmake --build --preset Debug
```

## 角度阶梯

实验总时长约 100 秒：

- `0 ~ 5s`：保持启动 pitch 零点，记录基线
- `5 ~ 90s`：执行 pitch 静态角度阶梯
- `90 ~ 100s`：回到启动 pitch 零点

pitch 目标 offset 如下，单位为相对启动零点的角度：

```text
0deg
+10deg
0deg
-10deg
0deg
+20deg
0deg
-20deg
0deg
+30deg
0deg
-30deg
0deg
+38deg
0deg
-38deg
0deg
```

每个平台保持 `5s`。后续拟合时默认只取每个平台最后 `1s`，避开换向后的过渡过程。

## 固件行为

- yaw 固定在启动瞬间的 yaw 零点。
- pitch 按上面的静态角度表运动。
- pitch 目标会经过 `GIMBAL_PITCH_MIN_RAD / GIMBAL_PITCH_MAX_RAD` 软件限位。
- 每次切换平台时会清空 pitch 三环 PID 状态，降低上一个平台积分残留对保持输出的污染。
- RTT 遥测帧仍是 `A6 6A` 开头的 100 字节二进制帧。
- `mode = 2` 表示当前数据来自 pitch 前馈实验。
- `phase = 5` 表示当前处于静态角度阶梯阶段。

## 采集方式

运行：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 108 \
  --output data/sysid/pitch_ff_rtt_$(date +%Y%m%d_%H%M%S).csv
```

如果提示 J-Link 冲突，先关闭 Ozone、JLinkRTTLogger、JLinkRTTClient、JLinkGDBServer，或使用：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 108 --kill-conflicts \
  --output data/sysid/pitch_ff_rtt_$(date +%Y%m%d_%H%M%S).csv
```

## 主机依赖

主机侧采集、分析、绘图依赖已经整理在：

```text
host_tools/requirements.txt
```

如果需要重新安装：

```bash
.venv-host/bin/python -m pip install -r host_tools/requirements.txt
```

## 拟合思路

采集完成后运行：

```bash
.venv-host/bin/python host_tools/analyze_pitch_ff_sysid.py \
  data/sysid/pitch_ff_rtt_YYYYMMDD_HHMMSS.csv
```

默认输出目录：

```text
data/sysid/analysis/pitch_ff_rtt_YYYYMMDD_HHMMSS_pitch_ff/
```

输出文件包括：

- `pitch_ff_result.json`：完整拟合结果、置信区间、拟合指标和固件宏建议值
- `pitch_ff_stage_summary.csv`：每个 pitch 平台段的稳定性、残差、是否被剔除
- `pitch_ff_target_summary.csv`：按目标角度分组平均后的拟合点和同角度重复经过的输出差异
- `pitch_ff_time_series.png`：目标角、实际角、速度、输出时域曲线
- `pitch_ff_fit.png`：`u_hold` 对 pitch 角度的拟合曲线和拟合指标
- `pitch_ff_residuals.png`：每个平台段的残差、速度和饱和诊断
- `pitch_ff_target_residuals.png`：按目标角度统计的残差和同角度输出 spread

每个平台稳定段估计保持输出：

```text
u_hold = output_ff_raw + output_sign * voltage_ref_raw
```

当前 pitch 的 `output_sign = -1`，所以本工程里等价于：

```text
u_hold = output_ff_raw - voltage_ref_raw
```

脚本默认先剔除不可靠平台，再按目标角度做一次平均，然后用这些目标平均点拟合，避免多个 `0deg` 锚点把拟合权重拉偏。

第一轮拟合完整模型：

```text
u_hold = A * sin(theta) + B * cos(theta) + C
```

判断方式：

- 如果 `B` 很小，继续使用当前 `sin + offset` 前馈结构。
- 如果 `B` 不小，说明前馈相位不只是一项 `sin(theta)`，应把代码扩展为 `sin + cos + offset`。
- 如果正向经过和反向经过同一角度差异明显，说明摩擦或线缆滞后较大，先不要用这部分差异当重力前馈。

## 自动筛选

脚本默认只使用每个平台最后 `1s` 数据，并自动剔除不可靠平台段。

默认剔除条件：

- 稳定段样本数少于 `20`
- 稳定段平均角速度大于 `0.05 rad/s`
- 稳定段平均绝对跟踪误差大于 `0.035 rad`
- 输出接近限幅的比例超过 `2%`

这些阈值可以通过命令行参数修改，例如：

```bash
.venv-host/bin/python host_tools/analyze_pitch_ff_sysid.py \
  data/sysid/pitch_ff_rtt_YYYYMMDD_HHMMSS.csv \
  --tail 1.2 \
  --max-stable-speed 0.04 \
  --max-tracking-error 0.03
```

## 拟合指标

重点看这些指标：

- `R2`：拟合度，越接近 `1` 越好。
- `RMSE`：拟合残差均方根，单位是 raw 输出值，越小越好。
- `LOOCV_RMSE`：留一交叉验证误差；如果明显大于 `RMSE`，说明某些平台点可能是异常点。
- `ci95_low / ci95_high`：参数 95% 置信区间；区间很宽说明数据不足或噪声大。
- `residual_reduced_raw`：当前固件兼容模型 `A*sin(theta)+C` 在每个平台的残差。
- `output_saturation_ratio`：平台段是否受输出限幅影响。

更新固件前建议满足：

- 大多数平台段没有被剔除。
- `A*sin(theta)+C` 的 `R2` 足够高。
- `LOOCV_RMSE` 没有明显劣化。
- 残差图没有明显单侧偏差。
- `cos(theta)` 项不显著，或者即使显著也暂时接受当前固件的一阶近似。

## 实验前提

- 上电前手动把云台摆到水平零点附近。
- pitch 运动范围内不要有线缆拉扯。
- 实验期间不要运行其他 J-Link 工具。
- 如果 pitch 接近限位或出现异常振荡，立即断电。
