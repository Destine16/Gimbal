# Yaw / Pitch 快响应系统辨识与 PID 优化

## 目的

这套流程用于在当前可运行 PID 的基础上进一步提高云台响应速度。它不是盲目试参数，而是按下面顺序处理：

1. 固件生成可重复的目标信号。
2. RTT 采集目标、反馈、速度、输出和三环 PID 内部量。
3. 脚本计算快响应指标和频响指标。
4. 脚本拟合 `output_cmd -> speed` 模型。
5. 在模型上搜索新的 `angle Kp / speed Kp / speed Ki / speed_ref_max`。
6. 上板复测候选参数，用对比报告判断是否采用。

## 新增系统辨识模式

```text
GIMBAL_SYSID_MODE=9   pitch PRBS 辨识
GIMBAL_SYSID_MODE=10  yaw fast multisine 快响应辨识
GIMBAL_SYSID_MODE=11  pitch fast multisine 快响应辨识
```

推荐优先使用 fast multisine 做快响应整定，因为它比单个阶跃包含更多频率信息，也比大范围 PRBS 更容易控制实验时间和幅度。

## 实验信号

### yaw fast multisine

目标是启动零点附近的多频正弦叠加：

```text
yaw_ref = yaw_startup_zero + sum(A_i * sin(2*pi*f_i*t + phi_i))
```

当前设计覆盖约 `0.2 Hz ~ 10 Hz`，主要用于评估 yaw 快速跟踪、相位滞后和输出饱和。

### pitch fast multisine

目标是启动 pitch 零点附近的多频正弦叠加，并经过 pitch 软件限位裁剪：

```text
pitch_ref = clamp(pitch_startup_zero + sum(A_i * sin(2*pi*f_i*t + phi_i)))
```

当前设计覆盖约 `0.2 Hz ~ 6 Hz`。pitch 受重力、线缆、摩擦和限位影响更明显，因此幅度比 yaw 更保守。

### pitch PRBS

pitch PRBS 用于补充多级阶跃数据，目标 offset 为：

```text
-10deg, -7deg, -4deg, 0deg, +4deg, +7deg, +10deg
```

它适合检查非线性、滞回和不同角度附近的响应差异。

## 编译

```bash
cmake --preset YawFastMultisineSysid
cmake --build --preset YawFastMultisineSysid

cmake --preset PitchFastMultisineSysid
cmake --build --preset PitchFastMultisineSysid

cmake --preset PitchPrbsSysid
cmake --build --preset PitchPrbsSysid
```

烧录对应 ELF 后，固件上电会自动运行对应实验。实验结束后要切回普通模式：

```bash
cmake --preset Debug -DGIMBAL_SYSID_MODE=0 -DVISION_DEBUG_RTT_ENABLE=OFF
cmake --build --preset Debug
```

## RTT 采集

yaw fast multisine：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 55 --kill-conflicts \
  --elf build/YawFastMultisineSysid/Gimbal.elf \
  --output data/sysid/yaw_fast_multisine_$(date +%Y%m%d_%H%M%S).csv
```

pitch fast multisine：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 55 --kill-conflicts \
  --elf build/PitchFastMultisineSysid/Gimbal.elf \
  --output data/sysid/pitch_fast_multisine_$(date +%Y%m%d_%H%M%S).csv
```

pitch PRBS：

```bash
.venv-host/bin/python host_tools/gimbal_sysid_rtt_capture.py --duration 80 --kill-conflicts \
  --elf build/PitchPrbsSysid/Gimbal.elf \
  --output data/sysid/pitch_prbs_$(date +%Y%m%d_%H%M%S).csv
```

采集时不要同时打开 Ozone、JLinkRTTClient、JLinkRTTLogger 或 J-Link GDB Server，避免抢占同一个 J-Link。

## 快响应分析

```bash
.venv-host/bin/python host_tools/analyze_fast_sysid.py data/sysid/yaw_fast_multisine_YYYYMMDD_HHMMSS.csv
```

脚本会自动判断 yaw / pitch 和 PRBS / multisine。必要时可以手动指定：

```bash
.venv-host/bin/python host_tools/analyze_fast_sysid.py data/sysid/pitch_prbs_YYYYMMDD_HHMMSS.csv \
  --axis pitch --mode 9
```

主要输出：

```text
*_fast_result.json
*_fast_time_series.png
*_fast_frequency_response.png
*_fast_step_metrics.png
*_fast_summary.png
```

核心指标：

```text
fast_score                  综合快响应评分，越小越好
tracking_rmse_deg           角度跟踪 RMS 误差，越小越好
tracking_mae_deg            角度跟踪平均绝对误差，越小越好
tracking_max_abs_deg        最大跟踪误差，用来发现大滞后或失控
speed_rmse_rad_s            速度环误差
output_saturation_ratio     输出接近限幅的比例
mean_coherence_0p1_3hz      低频段频响可信度
mean_coherence_3_10hz       高频段频响可信度
phase_deg_at_1hz/3hz/5hz    相位滞后，越接近 0 越好
```

## 模型优化 PID

yaw 当前参数示例：

```bash
.venv-host/bin/python host_tools/optimize_fast_pid_model.py data/sysid/yaw_fast_multisine_YYYYMMDD_HHMMSS.csv \
  --current-angle-kp 48 \
  --current-speed-kp 3600 \
  --current-speed-ki 600 \
  --speed-ref-limit 4.8
```

pitch 当前参数示例：

```bash
.venv-host/bin/python host_tools/optimize_fast_pid_model.py data/sysid/pitch_fast_multisine_YYYYMMDD_HHMMSS.csv \
  --current-angle-kp 28 \
  --current-speed-kp 2800 \
  --current-speed-ki 420 \
  --speed-ref-limit 3.6
```

脚本会输出：

```text
*_fast_pid_result.json
*_fast_pid_plant_validation.png
*_fast_pid_validation.png
*_fast_pid_summary.png
```

需要重点看：

```text
plant.validation_simulation_r2
plant.validation_simulation_rmse
optimization.validation.current
optimization.validation.optimized
```

如果模型验证 `R2` 很低，不要直接采用候选 PID。此时应重新采集数据，或降低实验中输出饱和和机械非线性的影响。

## 生成对比报告

只做 baseline 和优化建议：

```bash
.venv-host/bin/python host_tools/compare_fast_pid_results.py \
  --baseline-analysis data/sysid/analysis/BASELINE_DIR/yaw_fast_multisine_fast_result.json \
  --optimization data/sysid/analysis/OPT_DIR/yaw_fast_multisine_fast_pid_result.json \
  --output data/sysid/analysis/yaw_fast_pid_compare.md
```

如果已经烧录候选 PID 并复测，可以加入 candidate：

```bash
.venv-host/bin/python host_tools/compare_fast_pid_results.py \
  --baseline-analysis data/sysid/analysis/BASELINE_DIR/yaw_fast_multisine_fast_result.json \
  --candidate-analysis data/sysid/analysis/CANDIDATE_DIR/yaw_fast_multisine_fast_result.json \
  --optimization data/sysid/analysis/OPT_DIR/yaw_fast_multisine_fast_pid_result.json \
  --output data/sysid/analysis/yaw_fast_pid_compare.md
```

判断原则：

- `fast_score`、RMSE、MAE、最大误差下降，说明候选参数更好。
- 相位滞后下降，说明快速跟踪能力提升。
- 输出饱和占比明显上升，说明参数可能过激。
- 模型预测变好但复测变差，说明模型外推不可靠，需要重新采集或缩小优化范围。

## 推荐流程

1. 先用当前最优参数烧录 yaw fast multisine，采集 baseline。
2. 运行 `analyze_fast_sysid.py`，确认数据没有严重饱和且相干度可用。
3. 运行 `optimize_fast_pid_model.py`，得到候选参数。
4. 手动把候选参数写入 `Application/gimbal/gimbal_params.c`。
5. 烧录普通模式或同一个 sysid 模式做小范围验证。
6. 再采一轮 fast multisine，运行 `compare_fast_pid_results.py`。
7. yaw 验证通过后，再用同样流程做 pitch。

## 注意事项

- 这套方法是基于实测模型的局部优化，不保证一次得到全局最优。
- 参数候选值可以保留较多小数，但最终是否采用看复测指标，不看小数位数量。
- pitch 更容易受重力前馈、线缆、摩擦和 IMU 噪声影响，不要只按 yaw 结论机械套用。
- 如果输出长期打满，应该先处理限幅和实验幅度，否则模型会被非线性饱和污染。
