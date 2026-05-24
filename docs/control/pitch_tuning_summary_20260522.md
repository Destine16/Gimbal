# Pitch 调参小结 - 2026-05-22

## 当前保留参数

```text
angle Kp = 46
speed Kp = 3100
speed Ki = 300
pitch_speed_ref_max = 3.4 rad/s
gyro LPF alpha = 0.50
```

当前只保留静态重力前馈：

```text
output_ff = -1115.8459 * sin(theta) - 97.4462
```

速度前馈、D/gyro damping、参考速度前馈、静态滞回补偿均未保留；其中 speed feedforward 和简单 hysteresis feedforward 的固件开关、旧实验入口已经删除。

## 当前性能数据

数据文件：

```text
data/sysid/pitch_fast_multisine_current_kp46_sp3100_ki300_20260522_182257.csv
data/sysid/pitch_perf_step_current_kp46_sp3100_ki300_20260522_182431.csv
data/sysid/pitch_perf_sine_current_kp46_sp3100_ki300_20260522_182558.csv
```

关键图像：

```text
data/sysid/analysis/pitch_fast_multisine_current_kp46_sp3100_ki300_20260522_182257_pitch_fast_multisine_fast/pitch_fast_summary.png
data/sysid/analysis/pitch_fast_multisine_current_kp46_sp3100_ki300_20260522_182257_pitch_fast_multisine_fast/pitch_fast_time_series.png
data/sysid/analysis/pitch_perf_sine_current_kp46_sp3100_ki300_20260522_182558_pitch_perf_sine/sine_A5deg_T1s.png
data/sysid/analysis/pitch_perf_sine_current_kp46_sp3100_ki300_20260522_182558_pitch_perf_sine/sine_A20deg_T2s.png
data/sysid/analysis/pitch_perf_step_current_kp46_sp3100_ki300_20260522_182431_pitch_perf_step/step_p3deg.png
data/sysid/analysis/pitch_perf_step_current_kp46_sp3100_ki300_20260522_182431_pitch_perf_step/step_m3deg.png
```

## 验证结果

| 测试 | 指标 |
|---|---:|
| fast multisine RMSE | 0.4378 deg |
| fast multisine max error | 1.6886 deg |
| fast multisine output saturation | 0.00% |
| sine A=5deg T=1s RMSE | 0.6795 deg |
| sine A=5deg T=1s max error | 1.2344 deg |
| sine A=5deg T=1s phase lag | -10.78 deg |
| sine A=20deg T=2s RMSE | 1.1880 deg |
| sine A=20deg T=2s max error | 2.2681 deg |
| step +3deg steady error | about +0.050 deg |
| step -3deg steady error | about -0.048 deg |

## 清理结果

旧参数、候选参数和未保留方案的曲线、CSV 与分析输出已经删除。当前只保留本页列出的验证数据和静态前馈图谱数据。

## 后续方向

如果继续压小 `0.0x deg` 级稳态误差，优先做低速、小误差、慢淡入的静态补偿，并单独验证 `+/-3 deg` step。

如果继续压正弦峰值误差，不能直接靠 D 或简单参考速度前馈。需要先用模型确认相位/幅值补偿，再用 fast multisine 做安全验证。
