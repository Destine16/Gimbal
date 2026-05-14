# 云台性能测试报告

测试对象：当前工程参数  
数据来源：`data/perf`  
采集方式：J-Link RTT，系统辨识固件自动生成目标曲线  

说明：

- `N/A` 表示该指标在本次采样窗口内没有满足计算判据，例如没有进入稳定带，或没有完成 10%-90% 上升过程。
- 阶跃响应的稳定带使用分析脚本默认值：`max(2% * 阶跃幅度, 0.3 deg)`。
- 曲线图链接指向本次覆盖生成的 `data/perf/analysis` 图片。

## 3.1 阶跃响应指标

### yaw

| 阶跃幅度 | 上升时间 tr (s) | 峰值时间 tp (s) | 调节时间 ts (s) | 超调量 (%) | 稳态误差 (deg) | 曲线图 |
|---:|---:|---:|---:|---:|---:|---|
| +3 deg | 0.0143 | 0.1590 | 0.0490 | 1.60 | 0.1073 | [曲线](../data/perf/analysis/yaw_perf_step/step_p3deg.png) |
| +5 deg | 0.0103 | 0.0700 | 0.1700 | 23.76 | 0.0713 | [曲线](../data/perf/analysis/yaw_perf_step/step_p5deg.png) |
| +10 deg | 0.0149 | 0.0990 | 0.2390 | 52.76 | 0.1107 | [曲线](../data/perf/analysis/yaw_perf_step/step_p10deg.png) |
| +20 deg | 0.0382 | 0.1390 | N/A | 12.85 | 0.3785 | [曲线](../data/perf/analysis/yaw_perf_step/step_p20deg.png) |

### pitch

| 阶跃幅度 | 上升时间 tr (s) | 峰值时间 tp (s) | 调节时间 ts (s) | 超调量 (%) | 稳态误差 (deg) | 曲线图 |
|---:|---:|---:|---:|---:|---:|---|
| +3 deg | N/A | 2.1490 | N/A | 0.00 | 0.2975 | [曲线](../data/perf/analysis/pitch_perf_step/step_p3deg.png) |
| -3 deg | 0.2026 | 2.3000 | 0.1200 | 4.67 | 0.0346 | [曲线](../data/perf/analysis/pitch_perf_step/step_m3deg.png) |
| -10 deg | 0.0825 | 0.1800 | 0.2450 | 8.39 | 0.2077 | [曲线](../data/perf/analysis/pitch_perf_step/step_m10deg.png) |
| +10 deg | 0.5504 | 1.9000 | 2.0190 | 0.00 | 0.2829 | [曲线](../data/perf/analysis/pitch_perf_step/step_p10deg.png) |

## 3.2 跟踪性能

模型：

```text
A * sin(2*pi/T * t)
```

### yaw

| 测试条件 | 平均绝对误差 MAE (deg) | 均方根误差 RMSE (deg) | 最大误差 Max Error (deg) | 曲线图 |
|---|---:|---:|---:|---|
| A = 5 deg, T = 1 s | 0.5730 | 0.6229 | 0.9497 | [曲线](../data/perf/analysis/yaw_perf_sine/sine_A5deg_T1s.png) |
| A = 20 deg, T = 2 s | 1.0735 | 1.1912 | 3.5195 | [曲线](../data/perf/analysis/yaw_perf_sine/sine_A20deg_T2s.png) |

### pitch

| 测试条件 | 平均绝对误差 MAE (deg) | 均方根误差 RMSE (deg) | 最大误差 Max Error (deg) | 曲线图 |
|---|---:|---:|---:|---|
| A = 5 deg, T = 1 s | 1.0959 | 1.2022 | 2.0185 | [曲线](../data/perf/analysis/pitch_perf_sine/sine_A5deg_T1s.png) |
| A = 20 deg, T = 2 s | 1.9514 | 2.1314 | 3.4727 | [曲线](../data/perf/analysis/pitch_perf_sine/sine_A20deg_T2s.png) |

## 汇总评分

| 项目 | 分数 |
|---|---:|
| yaw 阶跃 | 18.8265 |
| yaw 正弦跟踪 | 6.6853 |
| pitch 阶跃 | 13.0405 |
| pitch 正弦跟踪 | 11.8934 |
| 总分 | 50.4457 |

## 数据文件

| 项目 | CSV |
|---|---|
| yaw 阶跃 | `data/perf/yaw_perf_step.csv` |
| pitch 阶跃 | `data/perf/pitch_perf_step.csv` |
| yaw 正弦 | `data/perf/yaw_perf_sine.csv` |
| pitch 正弦 | `data/perf/pitch_perf_sine.csv` |
