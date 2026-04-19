# 视觉 USB CDC 通信协议

## 适用范围

这份协议面向当前项目的单视觉、单电控、单云台场景。

- 视觉主机：1 个
- 下位机控制器：1 个
- 双轴云台：1 套
- 传输链路：USB CDC

协议只包含当前云台自瞄所需字段。

## 设计目标

- 视觉向电控发送最小必要控制量
- 当前只保留 `delta_yaw` 和 `delta_pitch`
- 不直接传 `float`
- 帧结构固定长度，方便 STM32 直接解析

## 下行命令帧

视觉主机发给电控的命令帧格式如下：

```text
SOF1 | SOF2 | delta_yaw | delta_pitch | CRC16
```

字段说明：

- `SOF1`：`0xA5`
- `SOF2`：`0x5A`
- `delta_yaw`：`int16_t`，单位 `0.0001 rad`
- `delta_pitch`：`int16_t`，单位 `0.0001 rad`
- `CRC16`：CRC16/MODBUS，低字节在前

固定总帧长：`8 byte`

## 上行状态帧

电控回传给上位机的状态帧格式如下：

```text
SOF1 | SOF2 | yaw_actual | pitch_actual | last_rx_delta_yaw | last_rx_delta_pitch | CRC16
```

字段说明：

- `SOF1`：`0x5A`
- `SOF2`：`0xA5`
- `yaw_actual`：`int32_t`，单位 `0.0001 rad`
- `pitch_actual`：`int32_t`，单位 `0.0001 rad`
- `last_rx_delta_yaw`：`int16_t`，单位 `0.0001 rad`
- `last_rx_delta_pitch`：`int16_t`，单位 `0.0001 rad`
- `CRC16`：CRC16/MODBUS，低字节在前

固定总帧长：`16 byte`

## CRC16 规则

- 类型：`CRC16/MODBUS`
- 初值：`0xFFFF`
- 多项式：`0xA001`
- 计算范围：
  - 从 `SOF1` 开始
  - 到 `delta_pitch` 结束
  - 包含 `SOF1`
  - 包含 `SOF2`
  - 不包含 CRC 自身

## VisionCmd

### 作用

视觉把当前目标对应的 yaw / pitch 增量发送给下位机。

### 数据结构

```c
typedef struct __attribute__((packed)) {
    int16_t delta_yaw_1e4rad;
    int16_t delta_pitch_1e4rad;
} VisionCmd_t;
```

### 字段含义

- `delta_yaw_1e4rad`
  - 当前云台还需要再转多少 yaw
  - 单位：`0.0001 rad`
- `delta_pitch_1e4rad`
  - 当前云台还需要再转多少 pitch
  - 单位：`0.0001 rad`

### 电控侧解释方式

当前固件默认使用事件目标模式：

- 每收到 1 帧合法视觉命令，只消费 1 次 `delta_yaw_1e4rad` 和 `delta_pitch_1e4rad`
- 电控在收到该帧的时刻，把 delta 换算成新的绝对目标
- 新目标会一直保持，直到下一帧合法视觉命令到来
- 因此视觉不需要高频连续发送；识别频率较慢时也可以正常使用

换算关系：

```text
yaw_target   = current_yaw   + delta_yaw
pitch_target = current_pitch + delta_pitch
```

如果视觉没有发送新帧，电控继续保持上一次生成的 `yaw_target` / `pitch_target`，不会反复累加旧 delta。

模式开关：

- `VISION_CONTROL_MODE=1`：事件目标模式，默认值，适配慢速视觉
- `VISION_CONTROL_MODE=0`：连续 delta 模式，适合高频连续发送同一时刻的视觉误差

当前开关位置：

- `Application/robot_def.h`
- `CMakeLists.txt`

对应代码位置：

- `Application/cmd/robot_cmd.c`

## VisionStatus

### 作用

电控向上位机回传当前云台实际角度，用于显示实时曲线和构造目标/实际对比。

### 数据结构

```c
typedef struct __attribute__((packed)) {
    int32_t yaw_actual_1e4rad;
    int32_t pitch_actual_1e4rad;
    int16_t last_rx_delta_yaw_1e4rad;
    int16_t last_rx_delta_pitch_1e4rad;
} VisionStatus_t;
```

### 字段含义

- `yaw_actual_1e4rad`
  - 当前 IMU yaw 反馈角
  - 单位：`0.0001 rad`
- `pitch_actual_1e4rad`
  - 当前 IMU pitch 反馈角
  - 单位：`0.0001 rad`
- `last_rx_delta_yaw_1e4rad`
  - 电控最近一次成功接收到的 yaw 增量命令
  - 单位：`0.0001 rad`
- `last_rx_delta_pitch_1e4rad`
  - 电控最近一次成功接收到的 pitch 增量命令
  - 单位：`0.0001 rad`

## 工程中的落地位置

- 协议解析与组包：
  - `Modules/vision/vision_comm.c`
- USB 接收入口：
  - `USB_DEVICE/App/usbd_cdc_if.c`
- USB 发送入口：
  - `USB_DEVICE/App/usbd_cdc_if.c`
- 指令解释：
  - `Application/cmd/robot_cmd.c`

## 错误处理

- CRC 错误：丢弃整帧
- 事件目标模式下，命令超时只作为在线状态诊断；已经生成的云台目标不会因为视觉暂时没发新帧而清零
- 连续 delta 模式下，命令超时会让最新视觉命令失效

当前超时逻辑在：

- `Modules/vision/vision_comm.c`
