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

## 帧格式

统一格式如下：

```text
SOF1 | SOF2 | delta_yaw | delta_pitch | CRC16
```

字段说明：

- `SOF1`：`0xA5`
- `SOF2`：`0x5A`
- `delta_yaw`：`int16_t`，单位 `0.0001 rad`
- `delta_pitch`：`int16_t`，单位 `0.0001 rad`
- `CRC16`：CRC16/MODBUS，低字节在前

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

固定总帧长为 `8 byte`

### 字段含义

- `delta_yaw_1e4rad`
  - 当前云台还需要再转多少 yaw
  - 单位：`0.0001 rad`
- `delta_pitch_1e4rad`
  - 当前云台还需要再转多少 pitch
  - 单位：`0.0001 rad`

### 电控侧解释方式

当前固件把 `delta_yaw_1e4rad` 和 `delta_pitch_1e4rad` 当作增量角：

```text
yaw_target   = current_yaw   + delta_yaw
pitch_target = current_pitch + delta_pitch
```

对应代码位置：

- `Application/cmd/robot_cmd.c`

## 工程中的落地位置

- 协议解析与组包：
  - `Modules/vision/vision_comm.c`
- USB 接收入口：
  - `USB_DEVICE/App/usbd_cdc_if.c`
- 指令解释：
  - `Application/cmd/robot_cmd.c`

## 错误处理

- CRC 错误：丢弃整帧
- 命令超时：最新视觉命令失效

当前超时逻辑在：

- `Modules/vision/vision_comm.c`
