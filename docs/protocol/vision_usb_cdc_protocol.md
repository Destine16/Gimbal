# 视觉 USB CDC 通信协议

## 适用范围

这份协议面向当前项目的单视觉、单电控、单云台场景。

- 视觉主机：1 个
- 下位机控制器：1 个
- 双轴云台：1 套
- 传输链路：USB CDC

协议只包含当前云台自瞄所需字段。

## 设计目标

- 视觉向电控发送瞄准结果
- 电控向视觉回传最小必要状态
- 不直接传 `float`
- 帧结构带长度和 CRC 校验

## 帧格式

统一格式如下：

```text
SOF1 | SOF2 | TYPE | LEN | PAYLOAD | CRC16
```

字段说明：

- `SOF1`：`0xA5`
- `SOF2`：`0x5A`
- `TYPE`：包类型
- `LEN`：负载长度
- `PAYLOAD`：包内容
- `CRC16`：CRC16/MODBUS，低字节在前

## CRC16 规则

- 类型：`CRC16/MODBUS`
- 初值：`0xFFFF`
- 多项式：`0xA001`
- 计算范围：
  - 从 `TYPE` 开始
  - 到 `PAYLOAD` 末尾结束
  - 不包含 `SOF1`
  - 不包含 `SOF2`
  - 不包含 CRC 自身

## 包类型

- `0x01`：`VisionCmd`
- `0x02`：`ControlStatus`

## VisionCmd

### 作用

视觉把当前目标的跟踪结果发送给下位机。

### 数据结构

```c
typedef struct __attribute__((packed)) {
    int16_t yaw_0p01deg;
    int16_t pitch_0p01deg;
    int16_t yaw_speed_0p01rad;
    int16_t pitch_speed_0p01rad;
    uint16_t distance_mm;
    uint8_t track_state;
    uint8_t fire_cmd;
} VisionCmd_t;
```

`LEN = 12`

### 字段含义

- `yaw_0p01deg`
  - 当前云台还需要再转多少 yaw
  - 单位：`0.01 deg`
- `pitch_0p01deg`
  - 当前云台还需要再转多少 pitch
  - 单位：`0.01 deg`
- `yaw_speed_0p01rad`
  - 可选 yaw 角速度参考
  - 单位：`0.01 rad/s`
- `pitch_speed_0p01rad`
  - 可选 pitch 角速度参考
  - 单位：`0.01 rad/s`
- `distance_mm`
  - 目标距离
  - 单位：`mm`
- `track_state`
  - `0` 表示无目标
  - 非 `0` 表示当前有有效目标
- `fire_cmd`
  - 视觉给出的发射建议

### 电控侧解释方式

当前固件把 `yaw_0p01deg` 和 `pitch_0p01deg` 当作增量角：

```text
yaw_target   = current_yaw   + yaw_delta
pitch_target = current_pitch + pitch_delta
```

对应代码位置：

- `Application/cmd/robot_cmd.c`

## ControlStatus

### 作用

电控向视觉回传最小必要状态。

### 数据结构

```c
typedef struct __attribute__((packed)) {
    uint8_t enemy_color;
    uint16_t bullet_speed_0p01mps;
    uint8_t vision_mode;
    uint8_t fire_permission;
} ControlStatus_t;
```

`LEN = 5`

### 字段含义

- `enemy_color`
  - 当前敌方颜色
- `bullet_speed_0p01mps`
  - 当前弹速
  - 单位：`0.01 m/s`
- `vision_mode`
  - 当前下位机模式
- `fire_permission`
  - 当前是否允许发射

## 工程中的落地位置

- 协议解析与组包：
  - `Modules/vision/vision_comm.c`
- USB 接收入口：
  - `USB_DEVICE/App/usbd_cdc_if.c`
- 指令解释：
  - `Application/cmd/robot_cmd.c`

## 错误处理

- CRC 错误：丢弃整帧
- 长度非法：丢弃整帧
- 命令超时：最新视觉命令失效

当前超时逻辑在：

- `Modules/vision/vision_comm.c`
