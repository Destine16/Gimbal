#include "bmi088_internal.h"

#define BMI088_ASYNC_FALLBACK_MS 5u
#define BMI088_DMA_TIMEOUT_MS    5u

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
    uint32_t now = HAL_GetTick();

    // 启用运行期 BMI088 异步采集,并清空上一轮遗留的状态/标志位
    bmi088_ctx.async.enabled = 1;
    bmi088_ctx.async.valid = 0;
    bmi088_ctx.async.have_gyro = 0;
    bmi088_ctx.async.have_accel = 0;
    // 兜底轮询模式下,启动后先主动拉起一轮 gyro + accel 采样
    bmi088_ctx.async.pending_gyro = 1;
    bmi088_ctx.async.pending_accel = 1;
    bmi088_ctx.async.prefer_accel = 0;
    bmi088_ctx.async.seq = 0;
    bmi088_ctx.async.last_update_tick = now;
    bmi088_ctx.async.last_transfer_tick = now;
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

void BMI088_AsyncPoll(void)
{
    uint32_t now;

    if (!bmi088_ctx.async.enabled || BMI088_SPI == NULL)
        return;

    now = HAL_GetTick();

    if (((bmi088_ctx.async.state != BMI088_ASYNC_IDLE) ||
         (HAL_SPI_GetState(BMI088_SPI) != HAL_SPI_STATE_READY)) &&
        ((now - bmi088_ctx.async.last_transfer_tick) > BMI088_DMA_TIMEOUT_MS))
    {
        bmi088_async_reset_state();
        bmi088_ctx.async.pending_accel = 1;
        bmi088_ctx.async.pending_gyro = 1;
        bmi088_ctx.async.last_transfer_tick = now;
    }

    if ((now - bmi088_ctx.async.last_update_tick) > BMI088_ASYNC_FALLBACK_MS)
    {
        bmi088_ctx.async.pending_accel = 1;
        bmi088_ctx.async.pending_gyro = 1;
    }

    bmi088_service_pending_transfer();
}

void BMI088_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 由 HAL_GPIO_EXTI_Callback() 转发进来; 当 BMI088 的 gyro/accel data ready 引脚产生外部中断时触发
    // 作用是把“传感器有新数据”转换成一次具体的 SPI DMA 读取请求
    if (!bmi088_ctx.async.enabled || BMI088_SPI == NULL)
        return;

    // 参考 Chassis 的稳定实现:
    // EXTI 只负责把“有新数据”记成 pending,真正是否启动 DMA 统一交给轮询服务函数决定,
    // 避免在中断上下文里与 HAL SPI 状态机交叉调用。
    if (GPIO_Pin == BMI088_GYRO_INT_Pin)
    {
        bmi088_ctx.async.pending_gyro = 1;
    }
    else if (GPIO_Pin == BMI088_ACCEL_INT_Pin)
    {
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
        BMI088_GYRO_NS_H();
        bmi088_parse_gyro_frame(bmi088_ctx.async.rx_buf);
        bmi088_ctx.async.have_gyro = 1;
        bmi088_ctx.async.last_update_tick = HAL_GetTick();
        bmi088_ctx.async.prefer_accel = 1;
        bmi088_publish_if_ready();
        bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        bmi088_service_pending_transfer();
        break;

    case BMI088_ASYNC_ACCEL_BUSY:
        BMI088_ACCEL_NS_H();
        bmi088_parse_accel_frame(bmi088_ctx.async.rx_buf);
        bmi088_ctx.async.have_accel = 1;
        bmi088_ctx.async.last_update_tick = HAL_GetTick();
        bmi088_ctx.async.prefer_accel = 0;
        bmi088_publish_if_ready();
        bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        bmi088_service_pending_transfer();
        break;

    case BMI088_ASYNC_TEMP_BUSY:
    default:
        bmi088_async_reset_state();
        break;
    }
}

void BMI088_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != BMI088_SPI)
        return;

    // 运行期 SPI/DMA 出错时直接拉回空闲,并请求下一轮重新采样。
    HAL_SPI_Abort_IT(hspi);
    bmi088_async_reset_state();
    bmi088_ctx.async.pending_accel = 1;
    bmi088_ctx.async.pending_gyro = 1;
    bmi088_ctx.async.last_transfer_tick = HAL_GetTick();
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
    if (HAL_SPI_GetState(BMI088_SPI) != HAL_SPI_STATE_READY)
    {
        bmi088_ctx.async.pending_gyro = 1;
        return 0;
    }

    // 与 Chassis 保持一致: 直接从 GYRO_X_L 开始读 6 字节有效数据,
    // 总长度 7 字节 = 1 字节命令 + 6 字节数据时钟。
    bmi088_ctx.async.tx_buf[0] = BMI088_GYRO_X_L | 0x80;
    memset(&bmi088_ctx.async.tx_buf[1], 0x55, 6);
    BMI088_GYRO_NS_L();
    bmi088_ctx.async.state = BMI088_ASYNC_GYRO_BUSY;
    bmi088_ctx.async.last_transfer_tick = HAL_GetTick();
    if (HAL_SPI_TransmitReceive_DMA(BMI088_SPI, bmi088_ctx.async.tx_buf, bmi088_ctx.async.rx_buf, 7) != HAL_OK)
    {
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
    if (HAL_SPI_GetState(BMI088_SPI) != HAL_SPI_STATE_READY)
    {
        bmi088_ctx.async.pending_accel = 1;
        return 0;
    }

    // accel 连续读比 gyro 多 1 拍 dummy:
    // 总线上一共发送 8 字节 = 1 字节读命令 + 1 字节额外 dummy + 6 字节有效数据对应的时钟
    // 因此这里需要补 7 个 0x55; DMA 结束后 rx_buf[0] 是命令拍回读,rx_buf[1] 是 dummy 拍回读,
    // 真正的 X/Y/Z 六个数据字节从 rx_buf[2] 开始
    bmi088_ctx.async.tx_buf[0] = BMI088_ACCEL_XOUT_L | 0x80;
    memset(&bmi088_ctx.async.tx_buf[1], 0x55, 7);
    BMI088_ACCEL_NS_L();
    bmi088_ctx.async.state = BMI088_ASYNC_ACCEL_BUSY;
    bmi088_ctx.async.last_transfer_tick = HAL_GetTick();
    if (HAL_SPI_TransmitReceive_DMA(BMI088_SPI, bmi088_ctx.async.tx_buf, bmi088_ctx.async.rx_buf, 8) != HAL_OK)
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

    // 两侧都待采时按上一次完成的类型交替,避免长期偏向某一边
    if (bmi088_ctx.async.pending_accel && bmi088_ctx.async.pending_gyro)
    {
        if (bmi088_ctx.async.prefer_accel)
        {
            bmi088_ctx.async.pending_accel = 0;
            if (bmi088_start_accel_dma_transfer())
                return;
        }
        else
        {
            bmi088_ctx.async.pending_gyro = 0;
            if (bmi088_start_gyro_dma_transfer())
                return;
        }
    }

    if (bmi088_ctx.async.pending_gyro)
    {
        bmi088_ctx.async.pending_gyro = 0;
        if (bmi088_start_gyro_dma_transfer())
            return;
    }

    if (bmi088_ctx.async.pending_accel)
    {
        bmi088_ctx.async.pending_accel = 0;
        (void)bmi088_start_accel_dma_transfer();
    }
}

void bmi088_async_reset_state(void)
{
    if (BMI088_SPI != NULL && HAL_SPI_GetState(BMI088_SPI) != HAL_SPI_STATE_READY)
        (void)HAL_SPI_Abort(BMI088_SPI);

    // 无论何种异常退出,都先释放两个片选并清空未发布的半帧状态
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
    bmi088_ctx.async.have_accel = 0;
    bmi088_ctx.async.have_gyro = 0;
    bmi088_ctx.async.pending_accel = 0;
    bmi088_ctx.async.pending_gyro = 0;
}
