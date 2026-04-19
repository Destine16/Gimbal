#include "bmi088_internal.h"

typedef struct
{
    uint8_t reg;
    uint8_t value;
    BMI088_Error_t error;
} BMI088_InitItem_t;

volatile uint32_t bmi088_debug_accel_chip_id_before_reset = 0u;
volatile uint32_t bmi088_debug_accel_chip_id_after_reset = 0u;
volatile uint32_t bmi088_debug_gyro_chip_id_before_reset = 0u;
volatile uint32_t bmi088_debug_gyro_chip_id_after_reset = 0u;

// accel 初始化表: {寄存器地址, 目标配置值, 该项写入失败时对应的错误位}
static const BMI088_InitItem_t bmi088_accel_init_table[BMI088_WRITE_ACCEL_REG_NUM] =
    {
        // accel 上电使能
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        // accel 进入 active mode
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        // accel 正常模式, 800Hz 输出数据率
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        // accel 量程配置为 ±6g
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        // 配置 accel 的 INT1 引脚: 使能该中断脚, 采用推挽输出, 中断有效电平为高
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_HIGH, BMI088_INT1_IO_CTRL_ERROR},
        // 把 accel 的 data ready 事件映射到 INT1, 这样每来一帧新 accel 数据都会从 INT1 发中断
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

// gyro 初始化表: {寄存器地址, 目标配置值, 该项写入失败时对应的错误位}
static const BMI088_InitItem_t bmi088_gyro_init_table[BMI088_WRITE_GYRO_REG_NUM] =
    {
        // gyro 量程配置为 ±2000 deg/s
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        // gyro 带宽/输出率配置
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        // gyro 正常工作模式
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        // 打开 gyro data ready
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        // 配置 gyro 的 INT3 引脚: 采用推挽输出, 中断有效电平为高
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_HIGH, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        // 把 gyro 的 data ready 事件映射到 INT3, 这样每来一帧新 gyro 数据都会从 INT3 发中断
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

BMI088_Error_t BMI088Init(SPI_HandleTypeDef *bmi088_SPI)
{
    BMI088_Error_t init_error = BMI088_NO_ERROR;

    // 绑定当前用于 BMI088 的 SPI 总线
    BMI088_SPI = bmi088_SPI;
    // 参考 Chassis 的 bring-up 顺序:
    // 初始化前先确保两路片选都回到非选中态,给 BMI088 一个明确稳定的空闲总线窗口
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    // 自制板只接 BMI088 时,MCU 往往比传感器更早进入任务调度;
    // 这里给一次明确的上电稳定窗口,让 cold boot 更接近 Chassis 上成熟的上电时序。
    DWT_Delay(0.05f);

    init_error |= bmi088_accel_init();
    init_error |= bmi088_gyro_init();

    // 使用离线静态数据固化出的默认参数; 重新标定时通过离线数据处理更新这些宏。
    BMI088.GyroOffset[0] = GxOFFSET;
    BMI088.GyroOffset[1] = GyOFFSET;
    BMI088.GyroOffset[2] = GzOFFSET;
    BMI088.gNorm = gNORM;
    BMI088.AccelScale = 9.81f / BMI088.gNorm;
    BMI088.TempWhenCali = 23.5f;

    return init_error;
}

BMI088_Error_t bmi088_accel_init(void)
{
    BMI088_Error_t local_error = BMI088_NO_ERROR;
    uint8_t res = 0;

    // 上电后先读两次 chip id,确认 SPI 通信链路基本正常
    bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_debug_accel_chip_id_before_reset = res;

    // 对 accel 做一次软复位,让寄存器回到已知初始状态
    bmi088_write_accel_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    DWT_Delay(0.08);

    // 复位后读取一次 chip id 仅做诊断记录。参考 Chassis:
    // cold boot 下真正决定“器件是否存在”的是 reset 前的探测,而不是 reset 后立刻的 WHO_AM_I。
    bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_debug_accel_chip_id_after_reset = res;

    // 依次写入初始化表中的 6 项 accel 配置,并逐项读回校验
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        // 第 0 列是寄存器地址,第 1 列是目标配置值
        bmi088_write_accel_reg(bmi088_accel_init_table[write_reg_num].reg, bmi088_accel_init_table[write_reg_num].value);
        // 参考 Chassis: accel 上电相关的前两步对等待时间更敏感,需要更保守的时序
        if (write_reg_num == 0u)
            DWT_Delay(0.005f);
        else if (write_reg_num == 1u)
            DWT_Delay(0.05f);
        else
            DWT_Delay(0.001f);

        // 读回刚才写入的寄存器,检查配置是否真正生效
        bmi088_read_accel_reg(bmi088_accel_init_table[write_reg_num].reg, &res);
        DWT_Delay(0.001);

        if (res != bmi088_accel_init_table[write_reg_num].value)
        {
            // 当前实现不在第一项失败时立刻退出,而是把错误码累积起来,便于一次看出哪些配置项异常
            local_error |= bmi088_accel_init_table[write_reg_num].error;
        }
    }

    return local_error;
}

BMI088_Error_t bmi088_gyro_init(void)
{
    BMI088_Error_t local_error = BMI088_NO_ERROR;
    uint8_t res = 0;

    // 上电后先读两次 chip id,确认 SPI 通信链路基本正常
    bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_debug_gyro_chip_id_before_reset = res;

    // 对 gyro 做一次软复位,让寄存器回到已知初始状态
    bmi088_write_gyro_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    DWT_Delay(0.08);

    // 复位后读取一次 chip id 仅做诊断记录,不作为 cold boot 的硬失败条件。
    // 你的 Chassis 实现同样只在 reset 前验证 gyro 是否存在。
    bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_debug_gyro_chip_id_after_reset = res;

    // 依次写入初始化表中的 6 项 gyro 配置,并逐项读回校验
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        // 第 0 列是寄存器地址,第 1 列是目标配置值
        bmi088_write_gyro_reg(bmi088_gyro_init_table[write_reg_num].reg, bmi088_gyro_init_table[write_reg_num].value);
        DWT_Delay(0.001);

        // 读回刚才写入的寄存器,检查配置是否真正生效
        bmi088_read_gyro_reg(bmi088_gyro_init_table[write_reg_num].reg, &res);
        DWT_Delay(0.001);

        if (res != bmi088_gyro_init_table[write_reg_num].value)
        {
            // 与 accel 初始化保持一致: 不重试当前项,只累计 gyro 对应错误码
            local_error |= bmi088_gyro_init_table[write_reg_num].error;
        }
    }

    return local_error;
}
