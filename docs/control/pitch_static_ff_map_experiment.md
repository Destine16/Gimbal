# Pitch 静态前馈图谱实验

目标：在相机安装在 pitch 轴上的真实负载状态下，测出不同角度、不同到达方向所需的保持输出，用于判断重力前馈和摩擦/线缆回差是否是 pitch 响应慢、抖动或不对称的主要原因。

## 实验模式

使用 `GIMBAL_SYSID_MODE=13`，对应 CMake preset：

```bash
cmake --preset PitchStaticFfMapSysid
cmake --build --preset PitchStaticFfMapSysid
```

固件会固定 yaw，只让 pitch 按上下行序列逐点保持。主要目标点：

```text
-35, -25, -15, -8, 0, +8, +15, +25, +35 deg
```

序列中还包含 `+/-38 deg` 锚点，只用于让 `+/-35 deg` 可以从两个方向到达。分析脚本默认不把超过 `36 deg` 的锚点纳入拟合。

每个点保持 `3.5 s`，脚本默认只取每段最后 `0.9 s` 的稳定数据。

## 采集

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 125 --kill-conflicts \
  --elf build/PitchStaticFfMapSysid/Gimbal.elf \
  --output data/sysid/pitch_static_ff_map_$(date +%Y%m%d_%H%M%S).csv
```

如果现场发现明显撞限位、线缆拉扯或持续抖动，应直接断电，不要继续采集。

## 分析

```bash
.venv-host/bin/python host_tools/analyze_pitch_static_ff_map.py \
  data/sysid/pitch_static_ff_map_YYYYMMDD_HHMMSS.csv
```

输出目录默认在：

```text
data/sysid/analysis/pitch_static_ff_map_YYYYMMDD_HHMMSS_pitch_static_ff_map/
```

关键输出：

```text
pitch_static_ff_map_stage_summary.csv
pitch_static_ff_map_target_summary.csv
pitch_static_ff_map_result.json
pitch_static_ff_map_macros.md
pitch_static_ff_map_fit.png
pitch_static_ff_map_direction_spread.png
```

## 拟合模型

脚本会拟合三组模型：

```text
u = A*sin(theta) + C
u = A*sin(theta) + B*cos(theta) + C
u = A*sin(theta) + B*cos(theta) + C + H*direction
```

其中：

- `theta` 是 pitch 实际角度。
- `direction=+1` 表示从低角度往高角度到达。
- `direction=-1` 表示从高角度往低角度到达。
- `u` 是估计出的保持所需前馈输出。

如果第三组模型明显降低 RMSE，说明 pitch 存在方向相关摩擦/线缆回差，继续单纯调 PID 收益会有限。

注意：当前固件默认只回填 `A*sin(theta)+C` 形式的静态前馈。`B*cos(theta)` 和 `H*direction` 在分析结果中只作为诊断量；如果它们很显著，应先新增并单独验证对应固件逻辑，不要直接把诊断系数当成可用宏。

## 回填参数

分析后优先看 `pitch_static_ff_map_macros.md`。脚本会给出当前固件兼容的保守回填值：

```c
#define GIMBAL_PITCH_OUTPUT_FF_SIN_RAW
#define GIMBAL_PITCH_OUTPUT_FF_OFFSET_RAW
```

方向相关 spread 只用于判断线缆/摩擦问题是否存在。简单 hysteresis 前馈开关已经删除，因为它不是“到达方向保持补偿”，之前测试也没有作为可保留方案。

回填后不要直接认为完成优化，还需要重新跑 pitch 阶跃和正弦性能测试确认。
