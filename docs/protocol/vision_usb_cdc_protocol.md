# 视觉 USB CDC 通信协议

## 适用范围

这份协议面向当前项目的单视觉、单电控、双轴云台场景。

- 视觉主机：1 个
- 下位机控制器：1 个
- 传输链路：USB CDC
- 数据格式：固定长度二进制帧
- 字节序：小端序
- 校验：CRC16/MODBUS

## 视觉到电控

视觉发给电控的是增量角，不是绝对角。

固定 10 字节：

```text
Byte0   Byte1   Byte2   Byte3          Byte4~5      Byte6~7        Byte8~9
SOF1    SOF2    seq     target_valid   delta_yaw    delta_pitch    CRC16
```

字段定义：

- `SOF1 = 0xA5`
- `SOF2 = 0x5A`
- `seq`：`uint8_t`，帧序号，每发一帧递增，`0~255` 循环
- `target_valid`：`uint8_t`，`1` 表示当前有目标，`0` 表示当前无目标
- `delta_yaw`：`int16_t`，单位 `0.0001 rad`
- `delta_pitch`：`int16_t`，单位 `0.0001 rad`
- `CRC16`：`uint16_t`，CRC16/MODBUS，低字节在前

协议结构体示例：

```c
typedef struct __attribute__((packed))
{
    uint8_t sof1;
    uint8_t sof2;
    uint8_t seq;
    uint8_t target_valid;
    int16_t delta_yaw_1e4rad;
    int16_t delta_pitch_1e4rad;
    uint16_t crc16;
} VisionToEcFrame_t;
```

## 电控到视觉

电控周期性回传最近采用的 `seq` 和当前实际角度。

固定 13 字节：

```text
Byte0   Byte1   Byte2      Byte3~6       Byte7~10        Byte11~12
SOF1    SOF2    seq_echo   yaw_actual    pitch_actual    CRC16
```

字段定义：

- `SOF1 = 0x5A`
- `SOF2 = 0xA5`
- `seq_echo`：`uint8_t`，最近一次被电控采用的视觉命令帧序号
- `yaw_actual`：`int32_t`，单位 `0.0001 rad`
- `pitch_actual`：`int32_t`，单位 `0.0001 rad`
- `CRC16`：`uint16_t`，CRC16/MODBUS，低字节在前

协议结构体示例：

```c
typedef struct __attribute__((packed))
{
    uint8_t sof1;
    uint8_t sof2;
    uint8_t seq_echo;
    int32_t yaw_actual_1e4rad;
    int32_t pitch_actual_1e4rad;
    uint16_t crc16;
} EcToVisionFrame_t;
```

## 单位与方向

角度换算：

```text
angle_rad = angle_1e4rad / 10000.0
angle_deg = angle_rad * 57.29577951
```

方向约定：

- `delta_yaw > 0`：从上往下看，云台逆时针
- `delta_pitch > 0`：相机抬头

## 电控解释方式

电控收到合法帧且 `target_valid = 1` 后，按当前实际角度生成目标角：

```text
yaw_target   = current_yaw   + delta_yaw
pitch_target = current_pitch + delta_pitch
```

说明：

- 每个合法新帧只消费一次
- `pitch_target` 会继续经过电控侧软件限位
- 视觉暂时不发新帧时，电控保持已有目标，不会重复叠加旧增量
- `target_valid = 0` 时，视觉侧应把 delta 填 `0`，电控不使用该帧生成新的瞄准目标
- `seq_echo` 只有在命令被电控控制逻辑采用后才更新

## CRC16 规则

参数：

- 类型：`CRC16/MODBUS`
- 初值：`0xFFFF`
- 多项式：`0xA001`
- CRC 低字节在前，高字节在后

视觉到电控 CRC 范围：

```text
A5 5A seq target_valid delta_yaw_low delta_yaw_high delta_pitch_low delta_pitch_high
```

也就是前 8 字节，不包含最后 2 字节 CRC。

电控到视觉 CRC 范围：

```text
5A A5 seq_echo yaw_actual[4] pitch_actual[4]
```

也就是前 11 字节，不包含最后 2 字节 CRC。

参考实现：

```c
#include <stdint.h>
#include <stddef.h>

static uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x0001u)
            {
                crc = (crc >> 1) ^ 0xA001u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}
```

## 自检样例

视觉到电控正数样例：

- `seq = 1`
- `target_valid = 1`
- `delta_yaw_1e4rad = 10`
- `delta_pitch_1e4rad = 8`

完整 10 字节必须为：

```text
A5 5A 01 01 0A 00 08 00 48 40
```

视觉到电控负数样例：

- `seq = 2`
- `target_valid = 1`
- `delta_yaw_1e4rad = -10`
- `delta_pitch_1e4rad = -8`

完整 10 字节必须为：

```text
A5 5A 02 01 F6 FF F8 FF 4C 53
```

视觉到电控无目标样例：

- `seq = 3`
- `target_valid = 0`
- `delta_yaw_1e4rad = 0`
- `delta_pitch_1e4rad = 0`

完整 10 字节必须为：

```text
A5 5A 03 00 00 00 00 00 70 7A
```

电控到视觉正数样例：

- `seq_echo = 1`
- `yaw_actual_1e4rad = 10`
- `pitch_actual_1e4rad = 8`

完整 13 字节必须为：

```text
5A A5 01 0A 00 00 00 08 00 00 00 BF 20
```

## 工程位置

- 协议解析与组包：`Modules/vision/vision_comm.c`
- 协议结构与调试变量：`Modules/vision/vision_comm.h`
- USB CDC 接收入口：`USB_DEVICE/App/usbd_cdc_if.c`
- 视觉命令解释：`Application/cmd/robot_cmd.c`
