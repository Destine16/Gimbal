#include "bmi088_internal.h"

static void bmi088_publish_if_ready(void)
{
    // 只有自上次发布后 accel 和 gyro 都至少更新过一次,才认为形成了一帧新的完整样本
    if (bmi088_ctx.async.have_gyro && bmi088_ctx.async.have_accel)
    {
        bmi088_ctx.async.valid = 1;
        ++bmi088_ctx.async.seq;
        bmi088_ctx.async.have_gyro = 0;
        bmi088_ctx.async.have_accel = 0;
    }
}

void BMI088_AsyncEnable(void)
{
    // 启用运行期 BMI088 异步采集,并清空上一轮遗留的状态/标志位
    bmi088_ctx.async.enabled = 1;
    bmi088_ctx.async.valid = 0;
    bmi088_ctx.async.have_gyro = 0;
    bmi088_ctx.async.have_accel = 0;
    bmi088_ctx.async.pending_gyro = 0;
    bmi088_ctx.async.pending_accel = 0;
    bmi088_ctx.async.seq = 0;
    bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
}

void BMI088_AsyncDisable(void)
{
    // 关闭运行期异步采集,同时把状态机和 pending 标志恢复到空闲状态
    bmi088_ctx.async.enabled = 0;
    bmi088_async_reset_state();
}

uint8_t BMI088_FetchData(IMU_Data_t *bmi088, uint32_t *seq)
{
    uint32_t primask = __get_PRIMASK(); // 进入函数前的中断开关状态,用于退出时按原状态恢复
    // 拷贝共享样本时暂时关中断,避免和 DMA 完成回调并发写同一份数据
    __disable_irq();

    // 如调用方需要,顺手带回当前样本序号
    if (seq)
        *seq = bmi088_ctx.async.seq;
    // 只有异步链路已经形成过完整有效样本时,才把共享 BMI088 数据复制给调用方
    if (bmi088_ctx.async.valid && bmi088)
        memcpy(bmi088, &BMI088, sizeof(IMU_Data_t));

    // 若进入前中断本来是开的,这里恢复开中断; 若进入前已经关中断,则保持原状
    if (!primask)
        __enable_irq();

    // 返回当前是否已经存在至少一帧可读的完整样本
    return bmi088_ctx.async.valid;
}

void BMI088_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 由 HAL_GPIO_EXTI_Callback() 转发进来; 当 BMI088 的 gyro/accel data ready 引脚产生外部中断时触发
    // 作用是把“传感器有新数据”转换成一次具体的 SPI DMA 读取请求
    if (!bmi088_ctx.async.enabled || BMI088_SPI == NULL)
        return;

    // gyro/accel 都由 data ready EXTI 触发; 如果 SPI 正忙就先挂成 pending
    if (GPIO_Pin == BMI088_GYRO_INT_Pin)
    {
        // gyro data ready 中断到了: 若 SPI 当前空闲,就立刻启动一次 gyro DMA 读取;
        // 若 SPI 正忙,则先记成 pending,等当前事务完成后再补读
        if (bmi088_ctx.async.state == BMI088_ASYNC_IDLE)
            (void)bmi088_start_gyro_dma_transfer();
        else
            bmi088_ctx.async.pending_gyro = 1;
    }
    else if (GPIO_Pin == BMI088_ACCEL_INT_Pin)
    {
        // accel data ready 中断到了: 处理逻辑与 gyro 相同,空闲则直接读,忙则挂 pending
        if (bmi088_ctx.async.state == BMI088_ASYNC_IDLE)
            (void)bmi088_start_accel_dma_transfer();
        else
            bmi088_ctx.async.pending_accel = 1;
    }
}

void BMI088_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // 由 HAL_SPI_TxRxCpltCallback() 转发进来; 当 BMI088 这次 SPI DMA 收发完成后触发
    // 作用是根据当前状态机判断“刚完成的是哪一笔传输”,再解析数据、发布样本并补发 pending 请求
    if (hspi != BMI088_SPI)
        return;

    switch (bmi088_ctx.async.state)
    {
    case BMI088_ASYNC_GYRO_BUSY:
        // gyro DMA 完成后即可更新角速度; 若 accel 也已在上次发布后更新过,则现在可以发布一帧完整样本
        BMI088_GYRO_NS_H();
        bmi088_parse_gyro_frame(bmi088_ctx.async.rx_buf);
        bmi088_ctx.async.have_gyro = 1;
        bmi088_publish_if_ready();
        bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        bmi088_service_pending_transfer();
        break;

    case BMI088_ASYNC_ACCEL_BUSY:
        // accel DMA 完成后先更新加速度,随后立刻链式启动温度读取
        BMI088_ACCEL_NS_H();
        bmi088_parse_accel_frame(bmi088_ctx.async.rx_buf);
        bmi088_ctx.async.have_accel = 1;
        bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        if (!bmi088_start_temp_dma_transfer())
        {
            // 温度附带读取失败时,仍允许先按最新 accel + gyro 发布样本,避免整帧数据被卡住
            bmi088_publish_if_ready();
            bmi088_service_pending_transfer();
        }
        break;

    case BMI088_ASYNC_TEMP_BUSY:
        // 温度 DMA 完成后,当前 accel 这一拍的附带信息也齐了; 若 gyro 也已更新,则发布一帧完整样本
        BMI088_ACCEL_NS_H();
        bmi088_parse_temp_frame(bmi088_ctx.async.rx_buf);
        bmi088_publish_if_ready();
        bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
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
    if (BMI088_SPI == NULL || bmi088_ctx.async.state != BMI088_ASYNC_IDLE)
        return 0;

    // 组织一次 gyro burst read 的 DMA 发包:
    // 第 1 字节发“从 chip id 开始读”的命令,后面 8 个 0x55 只是为了持续提供 SPI 时钟;
    // 因此 DMA 结束后 rx_buf[0] 是“命令拍”同时回来的无效字节,真正寄存器内容从 rx_buf[1] 开始
    bmi088_ctx.async.tx_buf[0] = BMI088_GYRO_CHIP_ID | 0x80;
    memset(&bmi088_ctx.async.tx_buf[1], 0x55, 8);
    // 拉低 gyro 片选,正式选中器件
    BMI088_GYRO_NS_L();
    bmi088_ctx.async.state = BMI088_ASYNC_GYRO_BUSY;
    // 启动一次 9 字节 SPI DMA 全双工传输: 1 字节命令 + 8 字节回读数据
    if (HAL_SPI_TransmitReceive_DMA(BMI088_SPI, bmi088_ctx.async.tx_buf, bmi088_ctx.async.rx_buf, 9) != HAL_OK)
    {
        // DMA 启动失败时立即释放片选并回退到空闲状态
        BMI088_GYRO_NS_H();
        bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        return 0;
    }

    // 返回 1 表示本次 gyro DMA 已经成功发起,后续等待 HAL_SPI_TxRxCpltCallback() 收尾
    return 1;
}

uint8_t bmi088_start_accel_dma_transfer(void)
{
    if (BMI088_SPI == NULL || bmi088_ctx.async.state != BMI088_ASYNC_IDLE)
        return 0;

    // accel 连续读比 gyro 多 1 拍 dummy:
    // 总线上一共发送 8 字节 = 1 字节读命令 + 1 字节额外 dummy + 6 字节有效数据对应的时钟
    // 因此这里需要补 7 个 0x55; DMA 结束后 rx_buf[0] 是命令拍回读,rx_buf[1] 是 dummy 拍回读,
    // 真正的 X/Y/Z 六个数据字节从 rx_buf[2] 开始
    bmi088_ctx.async.tx_buf[0] = BMI088_ACCEL_XOUT_L | 0x80;
    memset(&bmi088_ctx.async.tx_buf[1], 0x55, 7);
    BMI088_ACCEL_NS_L();
    bmi088_ctx.async.state = BMI088_ASYNC_ACCEL_BUSY;
    if (HAL_SPI_TransmitReceive_DMA(BMI088_SPI, bmi088_ctx.async.tx_buf, bmi088_ctx.async.rx_buf, 8) != HAL_OK)
    {
        BMI088_ACCEL_NS_H();
        bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        return 0;
    }

    return 1;
}

uint8_t bmi088_start_temp_dma_transfer(void)
{
    if (BMI088_SPI == NULL || bmi088_ctx.async.state != BMI088_ASYNC_IDLE)
        return 0;

    // 温度连续读同样走 accel 这一侧时序,因此也要多 1 拍 dummy:
    // 总线上一共发送 4 字节 = 1 字节读命令 + 1 字节额外 dummy + 2 字节温度有效数据对应的时钟
    // 所以这里要补 3 个 0x55; DMA 结束后 rx_buf[0] 是命令拍回读,rx_buf[1] 是 dummy 拍回读,
    // 真正的温度两字节数据在 rx_buf[2] 和 rx_buf[3]
    bmi088_ctx.async.tx_buf[0] = BMI088_TEMP_M | 0x80;
    memset(&bmi088_ctx.async.tx_buf[1], 0x55, 3);
    BMI088_ACCEL_NS_L();
    bmi088_ctx.async.state = BMI088_ASYNC_TEMP_BUSY;
    if (HAL_SPI_TransmitReceive_DMA(BMI088_SPI, bmi088_ctx.async.tx_buf, bmi088_ctx.async.rx_buf, 4) != HAL_OK)
    {
        BMI088_ACCEL_NS_H();
        bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        return 0;
    }

    return 1;
}

void bmi088_service_pending_transfer(void)
{
    if (bmi088_ctx.async.state != BMI088_ASYNC_IDLE)
        return;

    // SPI 空闲后优先补 accel,再补 gyro,避免高频中断撞车时丢掉最近一次触发
    if (bmi088_ctx.async.pending_accel)
    {
        bmi088_ctx.async.pending_accel = 0;
        if (bmi088_start_accel_dma_transfer())
            return;
    }

    if (bmi088_ctx.async.pending_gyro)
    {
        bmi088_ctx.async.pending_gyro = 0;
        (void)bmi088_start_gyro_dma_transfer();
    }
}

void bmi088_async_reset_state(void)
{
    // 无论何种异常退出,都先释放两个片选并清空未发布的半帧状态
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
    bmi088_ctx.async.have_accel = 0;
    bmi088_ctx.async.have_gyro = 0;
    bmi088_ctx.async.pending_accel = 0;
    bmi088_ctx.async.pending_gyro = 0;
}
