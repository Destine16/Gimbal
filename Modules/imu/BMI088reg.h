#ifndef BMI088REG_H
#define BMI088REG_H

#define BMI088_ACC_CHIP_ID 0x00       // accel chip id 寄存器
#define BMI088_ACC_CHIP_ID_VALUE 0x1E // accel 期望 chip id

#define BMI088_ACC_ERR_REG 0x02                    // accel 错误状态寄存器
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2       // accel 配置错误位偏移
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
#define BMI088_FATAL_ERROR_SHFITS 0x0              // 致命错误位偏移
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

#define BMI088_ACC_STATUS 0x03                     // accel 状态寄存器
#define BMI088_ACCEL_DRDY_SHFITS 0x7               // accel data ready 位偏移
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

#define BMI088_ACCEL_XOUT_L 0x12                   // accel X 轴低字节
#define BMI088_ACCEL_XOUT_M 0x13                   // accel X 轴高字节
#define BMI088_ACCEL_YOUT_L 0x14                   // accel Y 轴低字节
#define BMI088_ACCEL_YOUT_M 0x15                   // accel Y 轴高字节
#define BMI088_ACCEL_ZOUT_L 0x16                   // accel Z 轴低字节
#define BMI088_ACCEL_ZOUT_M 0x17                   // accel Z 轴高字节

#define BMI088_SENSORTIME_DATA_L 0x18              // sensor time 低字节
#define BMI088_SENSORTIME_DATA_M 0x19              // sensor time 中字节
#define BMI088_SENSORTIME_DATA_H 0x1A              // sensor time 高字节

#define BMI088_ACC_INT_STAT_1 0x1D                 // accel 中断状态寄存器
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7     // accel data ready 中断状态位偏移
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

#define BMI088_TEMP_M 0x22                         // 温度高位寄存器

#define BMI088_TEMP_L 0x23                         // 温度低位寄存器

#define BMI088_ACC_CONF 0x40                       // accel 带宽/输出率配置寄存器
#define BMI088_ACC_CONF_MUST_Set 0x80             // 按手册要求必须置位的保留位
#define BMI088_ACC_BWP_SHFITS 0x4                 // accel 带宽模式位偏移
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)   // accel OSR4 模式
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)   // accel OSR2 模式
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS) // accel normal 模式

#define BMI088_ACC_ODR_SHFITS 0x0                 // accel 输出数据率位偏移
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)   // accel ODR 12.5Hz
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)     // accel ODR 25Hz
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)     // accel ODR 50Hz
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)    // accel ODR 100Hz
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)    // accel ODR 200Hz
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)    // accel ODR 400Hz
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)    // accel ODR 800Hz
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)   // accel ODR 1600Hz

#define BMI088_ACC_RANGE 0x41                      // accel 量程配置寄存器

#define BMI088_ACC_RANGE_SHFITS 0x0                // accel 量程位偏移
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)   // accel ±3g
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)   // accel ±6g
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)  // accel ±12g
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)  // accel ±24g

#define BMI088_INT1_IO_CTRL 0x53                   // accel INT1 引脚配置寄存器
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3       // INT1 使能位偏移
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2       // INT1 推挽/开漏位偏移
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS) // INT1 推挽
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS) // INT1 开漏
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1        // INT1 输出电平有效位偏移
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)  // INT1 低电平有效
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS) // INT1 高电平有效

#define BMI088_INT2_IO_CTRL 0x54                   // accel INT2 引脚配置寄存器
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3       // INT2 使能位偏移
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2       // INT2 推挽/开漏位偏移
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS) // INT2 推挽
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS) // INT2 开漏
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1        // INT2 输出电平有效位偏移
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)  // INT2 低电平有效
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS) // INT2 高电平有效

#define BMI088_INT_MAP_DATA 0x58                   // accel 中断源映射寄存器
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6  // DRDY 映射到 INT2 的位偏移
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS)
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2  // DRDY 映射到 INT1 的位偏移
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)

#define BMI088_ACC_SELF_TEST 0x6D                  // accel 自检控制寄存器
#define BMI088_ACC_SELF_TEST_OFF 0x00              // accel 关闭自检
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D  // accel 正向自检激励
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09  // accel 反向自检激励

#define BMI088_ACC_PWR_CONF 0x7C                   // accel 电源模式配置寄存器
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03           // accel suspend mode
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00            // accel active mode

#define BMI088_ACC_PWR_CTRL 0x7D                   // accel 电源控制寄存器
#define BMI088_ACC_ENABLE_ACC_OFF 0x00             // accel 关闭
#define BMI088_ACC_ENABLE_ACC_ON 0x04              // accel 打开

#define BMI088_ACC_SOFTRESET 0x7E                  // accel 软复位寄存器
#define BMI088_ACC_SOFTRESET_VALUE 0xB6            // accel 软复位命令值

#define BMI088_GYRO_CHIP_ID 0x00                   // gyro chip id 寄存器
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F             // gyro 期望 chip id

#define BMI088_GYRO_X_L 0x02                       // gyro X 轴低字节
#define BMI088_GYRO_X_H 0x03                       // gyro X 轴高字节
#define BMI088_GYRO_Y_L 0x04                       // gyro Y 轴低字节
#define BMI088_GYRO_Y_H 0x05                       // gyro Y 轴高字节
#define BMI088_GYRO_Z_L 0x06                       // gyro Z 轴低字节
#define BMI088_GYRO_Z_H 0x07                       // gyro Z 轴高字节

#define BMI088_GYRO_INT_STAT_1 0x0A                // gyro 中断状态寄存器
#define BMI088_GYRO_DYDR_SHFITS 0x7                // gyro data ready 位偏移
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

#define BMI088_GYRO_RANGE 0x0F                     // gyro 量程配置寄存器
#define BMI088_GYRO_RANGE_SHFITS 0x0               // gyro 量程位偏移
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)  // gyro ±2000 deg/s
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)  // gyro ±1000 deg/s
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)   // gyro ±500 deg/s
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)   // gyro ±250 deg/s
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)   // gyro ±125 deg/s

#define BMI088_GYRO_BANDWIDTH 0x10                 // gyro 带宽/输出率配置寄存器
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80        // 按手册要求必须置位的保留位
#define BMI088_GYRO_2000_532_HZ 0x00               // gyro 带宽配置档位
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02
#define BMI088_GYRO_400_47_HZ 0x03
#define BMI088_GYRO_200_23_HZ 0x04
#define BMI088_GYRO_100_12_HZ 0x05
#define BMI088_GYRO_200_64_HZ 0x06
#define BMI088_GYRO_100_32_HZ 0x07

#define BMI088_GYRO_LPM1 0x11                      // gyro 电源/低功耗模式寄存器
#define BMI088_GYRO_NORMAL_MODE 0x00               // gyro normal mode
#define BMI088_GYRO_SUSPEND_MODE 0x80              // gyro suspend mode
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20         // gyro deep suspend mode

#define BMI088_GYRO_SOFTRESET 0x14                 // gyro 软复位寄存器
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6           // gyro 软复位命令值

#define BMI088_GYRO_CTRL 0x15                      // gyro 控制寄存器
#define BMI088_DRDY_OFF 0x00                       // 关闭 data ready
#define BMI088_DRDY_ON 0x80                        // 打开 data ready

#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16         // gyro INT3/INT4 引脚配置寄存器
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3      // INT4 推挽/开漏位偏移
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS) // INT4 推挽
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS) // INT4 开漏
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2       // INT4 输出电平有效位偏移
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)  // INT4 低电平有效
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS) // INT4 高电平有效
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1      // INT3 推挽/开漏位偏移
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS) // INT3 推挽
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS) // INT3 开漏
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0       // INT3 输出电平有效位偏移
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)  // INT3 低电平有效
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS) // INT3 高电平有效

#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18          // gyro 中断源映射寄存器

#define BMI088_GYRO_DRDY_IO_OFF 0x00               // DRDY 不映射到任何引脚
#define BMI088_GYRO_DRDY_IO_INT3 0x01              // DRDY 映射到 INT3
#define BMI088_GYRO_DRDY_IO_INT4 0x80              // DRDY 映射到 INT4
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4) // DRDY 同时映射到 INT3/INT4

#define BMI088_GYRO_SELF_TEST 0x3C                 // gyro 自检寄存器
#define BMI088_GYRO_RATE_OK_SHFITS 0x4             // 自检速率正常位偏移
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2           // 自检失败位偏移
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1            // 自检完成位偏移
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0           // 触发自检位偏移
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)

#endif
