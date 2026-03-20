#include "bmi088_internal.h"

void BMI088_AsyncEnable(void)
{
    // 启用运行期 BMI088 异步采集,并清空上一轮遗留的状态/标志位
    bmi088_async_enabled = 1;
    bmi088_async_valid = 0;
    bmi088_have_gyro = 0;
    bmi088_have_accel = 0;
    bmi088_pending_gyro = 0;
    bmi088_pending_accel = 0;
    bmi088_async_seq = 0;
    bmi088_async_state = BMI088_ASYNC_IDLE;
}

void BMI088_AsyncDisable(void)
{
    // 关闭运行期异步采集,同时把状态机和 pending 标志恢复到空闲状态
    bmi088_async_enabled = 0;
    bmi088_async_reset_state();
}

uint8_t BMI088_FetchData(IMU_Data_t *bmi088, uint32_t *seq)
{
    uint32_t primask = __get_PRIMASK();
    // 拷贝共享样本时暂时关中断,避免和 DMA 完成回调并发写同一份数据
    __disable_irq();

    if (seq)
        *seq = bmi088_async_seq;
    if (bmi088_async_valid && bmi088)
        memcpy(bmi088, &BMI088, sizeof(IMU_Data_t));

    if (!primask)
        __enable_irq();

    return bmi088_async_valid;
}

void BMI088_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!bmi088_async_enabled || BMI088_SPI == NULL)
        return;

    // gyro/accel 都由 data ready EXTI 触发; 如果 SPI 正忙就先挂成 pending
    if (GPIO_Pin == BMI088_GYRO_INT_Pin)
    {
        if (bmi088_async_state == BMI088_ASYNC_IDLE)
            (void)bmi088_start_gyro_dma_transfer();
        else
            bmi088_pending_gyro = 1;
    }
    else if (GPIO_Pin == BMI088_ACCEL_INT_Pin)
    {
        if (bmi088_async_state == BMI088_ASYNC_IDLE)
            (void)bmi088_start_accel_dma_transfer();
        else
            bmi088_pending_accel = 1;
    }
}

void BMI088_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != BMI088_SPI)
        return;

    switch (bmi088_async_state)
    {
    case BMI088_ASYNC_GYRO_BUSY:
        // gyro DMA 完成后即可更新角速度; 如果 accel 已就绪,这帧样本就完整了
        BMI088_GYRO_NS_H();
        bmi088_parse_gyro_frame(bmi088_spi_rx_buf);
        bmi088_have_gyro = 1;
        if (bmi088_have_accel)
            bmi088_async_valid = 1;
        if (bmi088_async_valid)
            ++bmi088_async_seq;
        bmi088_async_state = BMI088_ASYNC_IDLE;
        bmi088_service_pending_transfer();
        break;

    case BMI088_ASYNC_ACCEL_BUSY:
        // accel DMA 完成后先更新加速度,随后立刻链式启动温度读取
        BMI088_ACCEL_NS_H();
        bmi088_parse_accel_frame(bmi088_spi_rx_buf);
        bmi088_have_accel = 1;
        bmi088_async_state = BMI088_ASYNC_IDLE;
        if (!bmi088_start_temp_dma_transfer())
        {
            if (bmi088_have_gyro)
                bmi088_async_valid = 1;
            if (bmi088_async_valid)
                ++bmi088_async_seq;
            bmi088_service_pending_transfer();
        }
        break;

    case BMI088_ASYNC_TEMP_BUSY:
        // 温度 DMA 完成后,当前 accel 这一拍的附带信息也齐了
        BMI088_ACCEL_NS_H();
        bmi088_parse_temp_frame(bmi088_spi_rx_buf);
        if (bmi088_have_gyro && bmi088_have_accel)
            bmi088_async_valid = 1;
        if (bmi088_async_valid)
            ++bmi088_async_seq;
        bmi088_async_state = BMI088_ASYNC_IDLE;
        bmi088_service_pending_transfer();
        break;

    default:
        bmi088_async_reset_state();
        break;
    }
}

void BMI088_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != BMI088_SPI)
        return;

    // SPI/DMA 出错时直接中止本次传输,把状态机拉回空闲
    HAL_SPI_Abort_IT(hspi);
    bmi088_async_reset_state();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BMI088_EXTI_Callback(GPIO_Pin);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    BMI088_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    BMI088_SPI_ErrorCallback(hspi);
}

uint8_t bmi088_start_gyro_dma_transfer(void)
{
    if (BMI088_SPI == NULL || bmi088_async_state != BMI088_ASYNC_IDLE)
        return 0;

    // 从 chip id 开始读 8 字节,顺手校验这次 burst read 的目标器件是否正确
    bmi088_spi_tx_buf[0] = BMI088_GYRO_CHIP_ID | 0x80;
    memset(&bmi088_spi_tx_buf[1], 0x55, 8);
    BMI088_GYRO_NS_L();
    bmi088_async_state = BMI088_ASYNC_GYRO_BUSY;
    if (HAL_SPI_TransmitReceive_DMA(BMI088_SPI, bmi088_spi_tx_buf, bmi088_spi_rx_buf, 9) != HAL_OK)
    {
        BMI088_GYRO_NS_H();
        bmi088_async_state = BMI088_ASYNC_IDLE;
        return 0;
    }

    return 1;
}

uint8_t bmi088_start_accel_dma_transfer(void)
{
    if (BMI088_SPI == NULL || bmi088_async_state != BMI088_ASYNC_IDLE)
        return 0;

    // accel 数据寄存器从 XOUT_L 开始连续 6 字节,再加 1 个命令字节
    bmi088_spi_tx_buf[0] = BMI088_ACCEL_XOUT_L | 0x80;
    memset(&bmi088_spi_tx_buf[1], 0x55, 7);
    BMI088_ACCEL_NS_L();
    bmi088_async_state = BMI088_ASYNC_ACCEL_BUSY;
    if (HAL_SPI_TransmitReceive_DMA(BMI088_SPI, bmi088_spi_tx_buf, bmi088_spi_rx_buf, 8) != HAL_OK)
    {
        BMI088_ACCEL_NS_H();
        bmi088_async_state = BMI088_ASYNC_IDLE;
        return 0;
    }

    return 1;
}

uint8_t bmi088_start_temp_dma_transfer(void)
{
    if (BMI088_SPI == NULL || bmi088_async_state != BMI088_ASYNC_IDLE)
        return 0;

    // 温度寄存器只有 2 字节有效数据,这里单独补一拍短 DMA 读取
    bmi088_spi_tx_buf[0] = BMI088_TEMP_M | 0x80;
    memset(&bmi088_spi_tx_buf[1], 0x55, 3);
    BMI088_ACCEL_NS_L();
    bmi088_async_state = BMI088_ASYNC_TEMP_BUSY;
    if (HAL_SPI_TransmitReceive_DMA(BMI088_SPI, bmi088_spi_tx_buf, bmi088_spi_rx_buf, 4) != HAL_OK)
    {
        BMI088_ACCEL_NS_H();
        bmi088_async_state = BMI088_ASYNC_IDLE;
        return 0;
    }

    return 1;
}

void bmi088_service_pending_transfer(void)
{
    if (bmi088_async_state != BMI088_ASYNC_IDLE)
        return;

    // SPI 空闲后优先补 accel,再补 gyro,避免高频中断撞车时丢掉最近一次触发
    if (bmi088_pending_accel)
    {
        bmi088_pending_accel = 0;
        if (bmi088_start_accel_dma_transfer())
            return;
    }

    if (bmi088_pending_gyro)
    {
        bmi088_pending_gyro = 0;
        (void)bmi088_start_gyro_dma_transfer();
    }
}

void bmi088_async_reset_state(void)
{
    // 无论何种异常退出,都先释放两个片选并清空 pending 标志
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    bmi088_async_state = BMI088_ASYNC_IDLE;
    bmi088_pending_accel = 0;
    bmi088_pending_gyro = 0;
}
