# 云台坐标系与方向约定

## 总体原则

本工程把方向问题分成三层处理：

- 业务坐标层：定义上层命令、视觉和控制逻辑看到的 `yaw / pitch` 正方向。
- IMU 安装适配层：把 BMI088 芯片坐标转换到工程使用的机器人坐标。
- 电机方向适配层：把业务轴正方向转换到 GM6020 官方电机正方向和 CAN 原始输出方向。

这样做的目的，是让上层只关心云台动作含义，不需要记电机安装方向或 BMI088 芯片朝向。

## 业务坐标约定

当前工程采用常见机器人右手坐标：

```text
+X = 相机前方
+Y = 云台左方
+Z = 云台上方
```

角度方向约定：

```text
yaw   = 绕 +Z 轴，从上往下看逆时针
pitch = 绕 +Y 轴，业务上定义为相机抬头
roll  = 绕 +X 轴
```

上层命令和视觉协议只需要遵守业务约定：

```text
yaw   表示向左/逆时针方向修正
pitch 表示相机抬头方向修正
```

## BMI088 安装映射

当前 ACE 主控安装关系：

```text
相机前方 = BMI088 -X
云台上方 = BMI088 +Z
```

因此在 `Modules/imu/ins_task.c` 中，`BMI088_ToGimbalFrame()` 把 BMI088 原始坐标映射到工程坐标：

```c
gimbal_vec[X] = -bmi088_vec[X];
gimbal_vec[Y] = -bmi088_vec[Y];
gimbal_vec[Z] =  bmi088_vec[Z];
```

这一步会同时作用于加速度和角速度，并且初始化四元数时也使用同一套映射，避免初始化姿态和运行时姿态不一致。

## IMU 反馈用法

云台控制中，IMU 反馈来源为：

```text
yaw 角度反馈   = YawTotalAngle
yaw 角速度反馈 = Gyro[2]
pitch 角度反馈 = Pitch
pitch 角速度反馈 = Gyro[1]
```

注意：pitch 使用 `Gyro[1]`，因为在当前右手坐标中 pitch 绕 `Y` 轴。

## Pitch 正方向

在常见右手坐标 `+X 前, +Y 左, +Z 上` 中，绕 `+Y` 的数学正方向与“相机抬头/低头”的直观方向容易混淆。

本工程业务上明确规定：

```text
+pitch = 相机抬头
```

因此在 `Application/robot_def.h` 中，pitch 的 IMU 反馈符号通过这两个宏统一处理：

```c
#define GIMBAL_PITCH_IMU_ANGLE_TO_AXIS_SIGN    (-1.0f)
#define GIMBAL_PITCH_IMU_GYRO_TO_AXIS_SIGN     (-1.0f)
```

调试时，上层看到的业务 pitch 应该满足：

```text
相机抬头时，业务 pitch 增大
相机低头时，业务 pitch 减小
```

## GM6020 方向适配

GM6020 官方定义的是电机自身正方向，不直接定义“相机抬头/低头”。

当前机构已确认：

```text
yaw GM6020 官方正方向 = 业务 +yaw
pitch GM6020 官方正方向 = 相机低头
```

因为业务定义是 `+pitch = 相机抬头`，所以 pitch 业务轴和 pitch 电机官方方向相反：

```c
#define GIMBAL_YAW_AXIS_TO_MOTOR_SIGN      1.0f
#define GIMBAL_PITCH_AXIS_TO_MOTOR_SIGN    (-1.0f)
```

底层 CAN 原始输出和 GM6020 官方方向的关系单独由以下宏表示，通常保持 `+1.0f`：

```c
#define GM6020_YAW_RAW_CURRENT_TO_OFFICIAL_SIGN      1.0f
#define GM6020_PITCH_RAW_CURRENT_TO_OFFICIAL_SIGN    1.0f
#define GM6020_YAW_OFFICIAL_OUTPUT_TO_CAN_SIGN       1.0f
#define GM6020_PITCH_OFFICIAL_OUTPUT_TO_CAN_SIGN     1.0f
```

## 调试检查

上板低限幅调试时，建议按以下顺序检查：

```text
1. 手动逆时针 yaw，业务 yaw / YawTotalAngle 应增大。
2. 手动抬头 pitch，业务 pitch 应增大。
3. 给一个小的 +yaw 目标，云台应向业务 +yaw 方向运动。
4. 给一个小的 +pitch 目标，云台应抬头。
```

如果方向不对，优先按层定位：

```text
IMU 姿态/角速度方向错：检查 BMI088_ToGimbalFrame() 和 GIMBAL_*_IMU_*_SIGN。
业务轴和电机官方方向相反：检查 GIMBAL_*_AXIS_TO_MOTOR_SIGN。
CAN 原始输出和电机官方方向相反：检查 GM6020_*_OFFICIAL_OUTPUT_TO_CAN_SIGN。
电流反馈方向相反：检查 GM6020_*_RAW_CURRENT_TO_OFFICIAL_SIGN。
```

不要通过修改 PID 参数正负号来修方向问题。
