#include "bmi088_internal.h"

IMU_Data_t BMI088;
// BMI088 模块内部统一上下文: 换算系数、标定观测量、异步状态机和 DMA 缓冲都收进这里
BMI088_Context_t bmi088_ctx = {
    .accel_sen = BMI088_ACCEL_6G_SEN,
    .gyro_sen = BMI088_GYRO_2000_SEN,
    .cali_offset = 1,
    .async = {
        .state = BMI088_ASYNC_IDLE,
    },
};

#if defined(BMI088_USE_SPI)

#define BMI088_BLOCKING_SPI_TIMEOUT_MS 1000U
#define BMI088_ACCEL_READ_OVERHEAD     2U
#define BMI088_GYRO_READ_OVERHEAD      1U
#define BMI088_BLOCKING_MAX_FRAME_LEN  10U

static void bmi088_write_single_reg(uint8_t reg, uint8_t data);
static void bmi088_read_single_reg(uint8_t reg, uint8_t *return_data);

void bmi088_write_accel_reg(uint8_t reg, uint8_t data)
{
    // accel 片选包一层,上层不用每次手动控制 CS
    BMI088_ACCEL_NS_L();
    bmi088_write_single_reg(reg, data);
    BMI088_ACCEL_NS_H();
}

void bmi088_read_accel_reg(uint8_t reg, uint8_t *data)
{
    uint8_t tx_buf[BMI088_ACCEL_READ_OVERHEAD + 1U] = {reg | 0x80U, 0x55U, 0x55U};
    uint8_t rx_buf[BMI088_ACCEL_READ_OVERHEAD + 1U] = {0};

    BMI088_ACCEL_NS_L();
    // 参考 Chassis 的稳定实现: 单次事务里把地址+dummy+数据完整打完,
    // 避免单字节 HAL 调用带来的额外片内状态切换和时序抖动。
    HAL_SPI_TransmitReceive(BMI088_SPI,
                            tx_buf,
                            rx_buf,
                            BMI088_ACCEL_READ_OVERHEAD + 1U,
                            BMI088_BLOCKING_SPI_TIMEOUT_MS);
    BMI088_ACCEL_NS_H();
    *data = rx_buf[2];
}

void bmi088_read_accel_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx_buf[BMI088_BLOCKING_MAX_FRAME_LEN] = {0};
    uint8_t rx_buf[BMI088_BLOCKING_MAX_FRAME_LEN] = {0};

    if (len > (BMI088_BLOCKING_MAX_FRAME_LEN - BMI088_ACCEL_READ_OVERHEAD))
        len = BMI088_BLOCKING_MAX_FRAME_LEN - BMI088_ACCEL_READ_OVERHEAD;

    tx_buf[0] = reg | 0x80U;
    memset(&tx_buf[1], 0x55, BMI088_ACCEL_READ_OVERHEAD + len - 1U);

    BMI088_ACCEL_NS_L();
    HAL_SPI_TransmitReceive(BMI088_SPI,
                            tx_buf,
                            rx_buf,
                            BMI088_ACCEL_READ_OVERHEAD + len,
                            BMI088_BLOCKING_SPI_TIMEOUT_MS);
    BMI088_ACCEL_NS_H();
    memcpy(buf, &rx_buf[BMI088_ACCEL_READ_OVERHEAD], len);
}

void bmi088_write_gyro_reg(uint8_t reg, uint8_t data)
{
    // gyro 片选包一层,上层不用每次手动控制 CS
    BMI088_GYRO_NS_L();
    bmi088_write_single_reg(reg, data);
    BMI088_GYRO_NS_H();
}

void bmi088_read_gyro_reg(uint8_t reg, uint8_t *data)
{
    BMI088_GYRO_NS_L();
    // gyro 读时序比 accel 更直接: 地址后下一拍就能回出有效寄存器数据,不用额外补一拍 dummy
    bmi088_read_single_reg(reg, data);
    BMI088_GYRO_NS_H();
}

void bmi088_read_gyro_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx_buf[BMI088_BLOCKING_MAX_FRAME_LEN] = {0};
    uint8_t rx_buf[BMI088_BLOCKING_MAX_FRAME_LEN] = {0};

    if (len > (BMI088_BLOCKING_MAX_FRAME_LEN - BMI088_GYRO_READ_OVERHEAD))
        len = BMI088_BLOCKING_MAX_FRAME_LEN - BMI088_GYRO_READ_OVERHEAD;

    tx_buf[0] = reg | 0x80U;
    memset(&tx_buf[1], 0x55, len);

    BMI088_GYRO_NS_L();
    HAL_SPI_TransmitReceive(BMI088_SPI,
                            tx_buf,
                            rx_buf,
                            BMI088_GYRO_READ_OVERHEAD + len,
                            BMI088_BLOCKING_SPI_TIMEOUT_MS);
    BMI088_GYRO_NS_H();
    memcpy(buf, &rx_buf[BMI088_GYRO_READ_OVERHEAD], len);
}

static void bmi088_write_single_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {reg, data};
    uint8_t rx_buf[2] = {0};

    // SPI 写单寄存器保持成一次完整事务,与 Chassis 的阻塞初始化路径一致
    HAL_SPI_TransmitReceive(BMI088_SPI,
                            tx_buf,
                            rx_buf,
                            2U,
                            BMI088_BLOCKING_SPI_TIMEOUT_MS);
}

static void bmi088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    uint8_t tx_buf[2] = {reg | 0x80U, 0x55U};
    uint8_t rx_buf[2] = {0};

    HAL_SPI_TransmitReceive(BMI088_SPI,
                            tx_buf,
                            rx_buf,
                            2U,
                            BMI088_BLOCKING_SPI_TIMEOUT_MS);
    *return_data = rx_buf[1];
}

#endif

void BMI088_Read(IMU_Data_t *bmi088)
{
    static uint8_t buf[8] = {0};    // 临时 SPI 接收缓冲区,足够容纳一次 gyro 8 字节连续读
    static int16_t raw_data;        // 两字节拼接后的原始有符号数据中间量

    // 连续读取 accel 的 6 个数据字节: X_L/X_H, Y_L/Y_H, Z_L/Z_H
    bmi088_read_accel_regs(BMI088_ACCEL_XOUT_L, buf, 6);

    // 两个字节拼成一个 int16 原始值,再按当前量程系数和加速度模长修正系数换算成工程量
    raw_data = (int16_t)((buf[1]) << 8) | buf[0];
    bmi088->Accel[0] = raw_data * bmi088_ctx.accel_sen * bmi088->AccelScale;
    raw_data = (int16_t)((buf[3]) << 8) | buf[2];
    bmi088->Accel[1] = raw_data * bmi088_ctx.accel_sen * bmi088->AccelScale;
    raw_data = (int16_t)((buf[5]) << 8) | buf[4];
    bmi088->Accel[2] = raw_data * bmi088_ctx.accel_sen * bmi088->AccelScale;

    // 从 gyro chip id 开始连续读 8 字节: chip id + X/Y/Z 三轴原始数据; 这里 chip id 与数据寄存器相邻,顺手校验代价很低
    bmi088_read_gyro_regs(BMI088_GYRO_CHIP_ID, buf, 8);
    // 先确认读回来的第 1 字节仍然是 gyro chip id,避免本次 SPI 读链路异常时直接使用脏数据
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (bmi088_ctx.cali_offset)
        {
            // 开启零偏补偿时,角速度换算后再减去静态标定得到的 gyro offset
            raw_data = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = raw_data * bmi088_ctx.gyro_sen - bmi088->GyroOffset[0];
            raw_data = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = raw_data * bmi088_ctx.gyro_sen - bmi088->GyroOffset[1];
            raw_data = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = raw_data * bmi088_ctx.gyro_sen - bmi088->GyroOffset[2];
        }
        else
        {
            // 关闭零偏补偿时,直接按量程系数换算角速度
            raw_data = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = raw_data * bmi088_ctx.gyro_sen;
            raw_data = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = raw_data * bmi088_ctx.gyro_sen;
            raw_data = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = raw_data * bmi088_ctx.gyro_sen;
        }
    }

    // 温度寄存器一共 11 位有效数据,这里连续读出高/低位后重新拼接
    bmi088_read_accel_regs(BMI088_TEMP_M, buf, 2);

    raw_data = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    // BMI088 温度原始值是有符号 11 位,大于 1023 说明需要做补码还原
    if (raw_data > 1023)
        raw_data -= 2048;

    // 按手册给出的线性关系换算成摄氏温度
    bmi088->Temperature = raw_data * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void bmi088_parse_gyro_frame(const uint8_t *rx_buf)
{
    int16_t raw_data;

    // 运行期异步 DMA 与 Chassis 对齐:
    // 从 GYRO_X_L 开始连续读 6 字节,rx_buf[0] 是命令拍回读的无效字节,
    // 有效数据按 X_L/X_H, Y_L/Y_H, Z_L/Z_H 排在 rx_buf[1..6]。
    raw_data = (int16_t)((rx_buf[2]) << 8) | rx_buf[1];
    BMI088.Gyro[0] = raw_data * bmi088_ctx.gyro_sen - (bmi088_ctx.cali_offset ? BMI088.GyroOffset[0] : 0.0f);
    raw_data = (int16_t)((rx_buf[4]) << 8) | rx_buf[3];
    BMI088.Gyro[1] = raw_data * bmi088_ctx.gyro_sen - (bmi088_ctx.cali_offset ? BMI088.GyroOffset[1] : 0.0f);
    raw_data = (int16_t)((rx_buf[6]) << 8) | rx_buf[5];
    BMI088.Gyro[2] = raw_data * bmi088_ctx.gyro_sen - (bmi088_ctx.cali_offset ? BMI088.GyroOffset[2] : 0.0f);
}

void bmi088_parse_accel_frame(const uint8_t *rx_buf)
{
    int16_t raw_data;

    // 解析一次 accel DMA 回读帧: 把 X/Y/Z 三轴原始值换算成加速度,并乘上静态标定得到的 AccelScale
    // DMA 连续读里 rx_buf[0] 是回读命令字节,有效数据从 rx_buf[2] 开始
    raw_data = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    BMI088.Accel[0] = raw_data * bmi088_ctx.accel_sen * BMI088.AccelScale;
    raw_data = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    BMI088.Accel[1] = raw_data * bmi088_ctx.accel_sen * BMI088.AccelScale;
    raw_data = (int16_t)((rx_buf[7]) << 8) | rx_buf[6];
    BMI088.Accel[2] = raw_data * bmi088_ctx.accel_sen * BMI088.AccelScale;
}

void bmi088_parse_temp_frame(const uint8_t *rx_buf)
{
    // 解析一次温度 DMA 回读帧: 从 2 个字节里拼出 11 位有符号原始值,再换算成摄氏度
    int16_t raw_data = (int16_t)((rx_buf[2] << 3) | (rx_buf[3] >> 5));

    // 温度原始值是 11 位有符号数,需要按手册还原补码
    if (raw_data > 1023)
        raw_data -= 2048;

    // 按 BMI088 温度换算公式得到当前温度
    BMI088.Temperature = raw_data * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}
