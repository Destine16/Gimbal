#include "bmi088_internal.h"

#if defined(BMI088_USE_SPI)

static void bmi088_write_single_reg(uint8_t reg, uint8_t data);
static void bmi088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void bmi088_read_following_bytes(uint8_t *buf, uint8_t len);

void bmi088_write_accel_reg(uint8_t reg, uint8_t data)
{
    // accel 片选包一层,上层不用每次手动控制 CS
    BMI088_ACCEL_NS_L();
    bmi088_write_single_reg(reg, data);
    BMI088_ACCEL_NS_H();
}

void bmi088_read_accel_reg(uint8_t reg, uint8_t *data)
{
    BMI088_ACCEL_NS_L();
    // accel 读寄存器在地址后需要一个 dummy byte,随后下一拍才回出有效数据
    BMI088_read_write_byte(reg | 0x80);
    BMI088_read_write_byte(0x55);
    *data = BMI088_read_write_byte(0x55);
    BMI088_ACCEL_NS_H();
}

void bmi088_read_accel_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_ACCEL_NS_L();
    // accel 连续读同样需要先补一个 dummy byte,后面再把 len 个字节依次时钟出来
    BMI088_read_write_byte(reg | 0x80);
    BMI088_read_write_byte(0x55);
    bmi088_read_following_bytes(buf, len);
    BMI088_ACCEL_NS_H();
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
    bmi088_read_single_reg(reg, data);
    BMI088_GYRO_NS_H();
}

void bmi088_read_gyro_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(reg | 0x80);
    bmi088_read_following_bytes(buf, len);
    BMI088_GYRO_NS_H();
}

static void bmi088_write_single_reg(uint8_t reg, uint8_t data)
{
    // SPI 写单寄存器: 先发寄存器地址,再发要写入的 1 字节数据
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void bmi088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    // SPI 读单寄存器: reg | 0x80 把地址最高位置 1,表示读操作; 后面的 dummy 字节用于继续提供时钟
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

static void bmi088_read_following_bytes(uint8_t *buf, uint8_t len)
{
    // 连续发送 dummy byte,把后续寄存器内容一个个时钟出来
    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
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
    bmi088->Accel[0] = raw_data * BMI088_ACCEL_SEN * bmi088->AccelScale;
    raw_data = (int16_t)((buf[3]) << 8) | buf[2];
    bmi088->Accel[1] = raw_data * BMI088_ACCEL_SEN * bmi088->AccelScale;
    raw_data = (int16_t)((buf[5]) << 8) | buf[4];
    bmi088->Accel[2] = raw_data * BMI088_ACCEL_SEN * bmi088->AccelScale;

    // 从 gyro chip id 开始连续读 8 字节: chip id + X/Y/Z 三轴原始数据; 这里 chip id 与数据寄存器相邻,顺手校验代价很低
    bmi088_read_gyro_regs(BMI088_GYRO_CHIP_ID, buf, 8);
    // 先确认读回来的第 1 字节仍然是 gyro chip id,避免本次 SPI 读链路异常时直接使用脏数据
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            // 开启零偏补偿时,角速度换算后再减去静态标定得到的 gyro offset
            raw_data = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = raw_data * BMI088_GYRO_SEN - bmi088->GyroOffset[0];
            raw_data = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = raw_data * BMI088_GYRO_SEN - bmi088->GyroOffset[1];
            raw_data = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = raw_data * BMI088_GYRO_SEN - bmi088->GyroOffset[2];
        }
        else
        {
            // 关闭零偏补偿时,直接按量程系数换算角速度
            raw_data = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = raw_data * BMI088_GYRO_SEN;
            raw_data = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = raw_data * BMI088_GYRO_SEN;
            raw_data = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = raw_data * BMI088_GYRO_SEN;
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

    // 第 1 个回读字节是 gyro chip id,不对就丢掉这帧
    if (rx_buf[1] != BMI088_GYRO_CHIP_ID_VALUE)
        return;

    raw_data = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    BMI088.Gyro[0] = raw_data * BMI088_GYRO_SEN - (caliOffset ? BMI088.GyroOffset[0] : 0.0f);
    raw_data = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    BMI088.Gyro[1] = raw_data * BMI088_GYRO_SEN - (caliOffset ? BMI088.GyroOffset[1] : 0.0f);
    raw_data = (int16_t)((rx_buf[7]) << 8) | rx_buf[6];
    BMI088.Gyro[2] = raw_data * BMI088_GYRO_SEN - (caliOffset ? BMI088.GyroOffset[2] : 0.0f);
}

void bmi088_parse_accel_frame(const uint8_t *rx_buf)
{
    int16_t raw_data;

    // DMA 连续读里 rx_buf[0] 是回读命令字节,有效数据从 rx_buf[2] 开始
    raw_data = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    BMI088.Accel[0] = raw_data * BMI088_ACCEL_SEN * BMI088.AccelScale;
    raw_data = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    BMI088.Accel[1] = raw_data * BMI088_ACCEL_SEN * BMI088.AccelScale;
    raw_data = (int16_t)((rx_buf[7]) << 8) | rx_buf[6];
    BMI088.Accel[2] = raw_data * BMI088_ACCEL_SEN * BMI088.AccelScale;
}

void bmi088_parse_temp_frame(const uint8_t *rx_buf)
{
    int16_t raw_data = (int16_t)((rx_buf[2] << 3) | (rx_buf[3] >> 5));

    // 温度原始值是 11 位有符号数,需要按手册还原补码
    if (raw_data > 1023)
        raw_data -= 2048;

    BMI088.Temperature = raw_data * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}
