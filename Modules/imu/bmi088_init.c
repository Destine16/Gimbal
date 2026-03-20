#include "bmi088_internal.h"

// accel 初始化表: {寄存器地址, 目标配置值, 该项写入失败时对应的错误码}
static uint8_t bmi088_accel_init_table[BMI088_WRITE_ACCEL_REG_NUM][3] =
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

// gyro 初始化表: {寄存器地址, 目标配置值, 该项写入失败时对应的错误码}
static uint8_t bmi088_gyro_init_table[BMI088_WRITE_GYRO_REG_NUM][3] =
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

static void Calibrate_MPU_Offset(IMU_Data_t *bmi088);

uint8_t BMI088Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate)
{
    uint8_t init_error = BMI088_NO_ERROR;

    // 绑定当前用于 BMI088 的 SPI 总线
    BMI088_SPI = bmi088_SPI;

    // accel/gyro 各自返回本模块初始化错误,这里统一汇总
    init_error |= bmi088_accel_init();
    init_error |= bmi088_gyro_init();

    if (calibrate && init_error != BMI088_NO_SENSOR)
    {
        // 芯片在线时优先做一次上电静态标定
        Calibrate_MPU_Offset(&BMI088);
    }
    else
    {
        // 不在线标定时直接回退到离线默认参数
        BMI088.GyroOffset[0] = GxOFFSET;
        BMI088.GyroOffset[1] = GyOFFSET;
        BMI088.GyroOffset[2] = GzOFFSET;
        BMI088.gNorm = gNORM;
        BMI088.AccelScale = 9.81f / BMI088.gNorm;
        BMI088.TempWhenCali = 40;
    }

    return init_error;
}

uint8_t bmi088_accel_init(void)
{
    uint8_t local_error = BMI088_NO_ERROR;
    uint8_t res = 0;

    // 上电后先读两次 chip id,确认 SPI 通信链路基本正常
    bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &res);
    DWT_Delay(0.001);

    // 对 accel 做一次软复位,让寄存器回到已知初始状态
    bmi088_write_accel_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    DWT_Delay(0.08);

    // 复位后再次读取 chip id,确认器件已经重新正常响应
    bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &res);
    DWT_Delay(0.001);

    // WHO_AM_I 校验失败说明当前总线上没有正确读到 accel 芯片
    if (res != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    // 依次写入初始化表中的 6 项 accel 配置,并逐项读回校验
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        // 第 0 列是寄存器地址,第 1 列是目标配置值
        bmi088_write_accel_reg(bmi088_accel_init_table[write_reg_num][0], bmi088_accel_init_table[write_reg_num][1]);
        DWT_Delay(0.001);

        // 读回刚才写入的寄存器,检查配置是否真正生效
        bmi088_read_accel_reg(bmi088_accel_init_table[write_reg_num][0], &res);
        DWT_Delay(0.001);

        if (res != bmi088_accel_init_table[write_reg_num][1])
        {
            // 当前实现不在第一项失败时立刻退出,而是把错误码累积起来,便于一次看出哪些配置项异常
            local_error |= bmi088_accel_init_table[write_reg_num][2];
        }
    }

    return local_error;
}

uint8_t bmi088_gyro_init(void)
{
    uint8_t local_error = BMI088_NO_ERROR;
    uint8_t res = 0;

    // 上电后先读两次 chip id,确认 SPI 通信链路基本正常
    bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID, &res);
    DWT_Delay(0.001);

    // 对 gyro 做一次软复位,让寄存器回到已知初始状态
    bmi088_write_gyro_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    DWT_Delay(0.08);

    // 复位后再次读取 chip id,确认器件已经重新正常响应
    bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID, &res);
    DWT_Delay(0.001);
    bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID, &res);
    DWT_Delay(0.001);

    // WHO_AM_I 校验失败说明当前总线上没有正确读到 gyro 芯片
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    // 依次写入初始化表中的 6 项 gyro 配置,并逐项读回校验
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        // 第 0 列是寄存器地址,第 1 列是目标配置值
        bmi088_write_gyro_reg(bmi088_gyro_init_table[write_reg_num][0], bmi088_gyro_init_table[write_reg_num][1]);
        DWT_Delay(0.001);

        // 读回刚才写入的寄存器,检查配置是否真正生效
        bmi088_read_gyro_reg(bmi088_gyro_init_table[write_reg_num][0], &res);
        DWT_Delay(0.001);

        if (res != bmi088_gyro_init_table[write_reg_num][1])
        {
            // 与 accel 初始化保持一致: 不重试当前项,只累计 gyro 对应错误码
            local_error |= bmi088_gyro_init_table[write_reg_num][2];
        }
    }

    return local_error;
}

static void Calibrate_MPU_Offset(IMU_Data_t *bmi088)
{
    static float startTime;
    static uint16_t CaliTimes = 6000;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t raw_data;
    float gyroMax[3], gyroMin[3];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;

    startTime = DWT_GetTimeline_s();
    do
    {
        if (DWT_GetTimeline_s() - startTime > 12)
        {
            // 超时仍不稳定时,回退到离线默认标定参数
            bmi088->GyroOffset[0] = GxOFFSET;
            bmi088->GyroOffset[1] = GyOFFSET;
            bmi088->GyroOffset[2] = GzOFFSET;
            bmi088->gNorm = gNORM;
            bmi088->TempWhenCali = 40;
            break;
        }

        DWT_Delay(0.005);
        // 每次重试前先清空本轮累计量
        bmi088->gNorm = 0;
        bmi088->GyroOffset[0] = 0;
        bmi088->GyroOffset[1] = 0;
        bmi088->GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; ++i)
        {
            // 标定阶段仍使用阻塞读链路,保证流程简单可控
            bmi088_read_accel_regs(BMI088_ACCEL_XOUT_L, buf, 6);
            raw_data = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->Accel[0] = raw_data * BMI088_ACCEL_SEN;
            raw_data = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Accel[1] = raw_data * BMI088_ACCEL_SEN;
            raw_data = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Accel[2] = raw_data * BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(bmi088->Accel[0] * bmi088->Accel[0] +
                              bmi088->Accel[1] * bmi088->Accel[1] +
                              bmi088->Accel[2] * bmi088->Accel[2]);
            bmi088->gNorm += gNormTemp;

            bmi088_read_gyro_regs(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                raw_data = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->Gyro[0] = raw_data * BMI088_GYRO_SEN;
                bmi088->GyroOffset[0] += bmi088->Gyro[0];
                raw_data = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->Gyro[1] = raw_data * BMI088_GYRO_SEN;
                bmi088->GyroOffset[1] += bmi088->Gyro[1];
                raw_data = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->Gyro[2] = raw_data * BMI088_GYRO_SEN;
                bmi088->GyroOffset[2] += bmi088->Gyro[2];
            }

            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    gyroMax[j] = bmi088->Gyro[j];
                    gyroMin[j] = bmi088->Gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    if (bmi088->Gyro[j] > gyroMax[j])
                        gyroMax[j] = bmi088->Gyro[j];
                    if (bmi088->Gyro[j] < gyroMin[j])
                        gyroMin[j] = bmi088->Gyro[j];
                }
            }

            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; ++j)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];

            // 静止条件被破坏时,提前结束本轮统计并整体重来
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
            {
                break;
            }

            DWT_Delay(0.0005);
        }

        bmi088->gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; ++i)
            bmi088->GyroOffset[i] /= (float)CaliTimes;

        // 记下标定时温度,后面如果要做温漂补偿可以作为参考
        bmi088_read_accel_regs(BMI088_TEMP_M, buf, 2);
        raw_data = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (raw_data > 1023)
            raw_data -= 2048;
        bmi088->TempWhenCali = raw_data * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        caliCount++;
    } while (gNormDiff > 0.5f ||
             fabsf(bmi088->gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(bmi088->GyroOffset[0]) > 0.01f ||
             fabsf(bmi088->GyroOffset[1]) > 0.01f ||
             fabsf(bmi088->GyroOffset[2]) > 0.01f);

    // 用标定得到的实际重力模长把 accel 修正到 9.81 m/s^2
    bmi088->AccelScale = 9.81f / bmi088->gNorm;
}
