#ifndef BMI088_INTERNAL_H
#define BMI088_INTERNAL_H

#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "BMI088reg.h"
#include "bsp_dwt.h"

#include <math.h>
#include <string.h>

// 对外共享样本仍保留为公开全局; 其余运行期状态收进 bmi088_ctx
extern IMU_Data_t BMI088;

// BMI088 运行期异步采集状态机:
// 当前 accel / gyro 共用同一条 SPI1 和同一组 DMA 资源,因此同一时刻只会有一件事务在执行
typedef enum
{
    BMI088_ASYNC_IDLE = 0,   // SPI 空闲,可以启动新的 BMI088 传输
    BMI088_ASYNC_GYRO_BUSY,  // 当前正在进行 gyro DMA 读
    BMI088_ASYNC_ACCEL_BUSY, // 当前正在进行 accel DMA 读
    BMI088_ASYNC_TEMP_BUSY,  // 当前正在进行温度 DMA 读
} BMI088_AsyncState_e;

typedef struct
{
    volatile uint8_t enabled;           // 是否启用 EXTI + DMA 异步链路
    volatile uint8_t valid;             // 是否已经形成过至少一帧可供上层读取的完整样本
    volatile uint8_t have_gyro;         // 自上次发布样本后,gyro 是否已经更新过
    volatile uint8_t have_accel;        // 自上次发布样本后,accel 是否已经更新过
    volatile uint8_t pending_gyro;      // SPI 忙时挂起的 gyro 触发
    volatile uint8_t pending_accel;     // SPI 忙时挂起的 accel 触发
    volatile uint8_t prefer_accel;      // 轮询兜底时,两侧都待采时优先谁
    volatile uint32_t seq;              // 完整新样本序号; 只有自上次发布后 accel 和 gyro 都有新数据时才递增
    volatile uint32_t last_update_tick; // 最近一次成功完成 BMI088 DMA 读的系统节拍
    volatile uint32_t last_transfer_tick;// 最近一次发起 BMI088 DMA 读的系统节拍
    volatile BMI088_AsyncState_e state; // 当前异步状态机状态
    uint8_t tx_buf[9];                  // DMA 发送缓冲区
    uint8_t rx_buf[9];                  // DMA 接收缓冲区
} BMI088_AsyncContext_t;

typedef struct
{
    float accel_sen;         // accel 原始计数到工程量的换算系数
    float gyro_sen;          // gyro 原始计数到工程量的换算系数
    uint8_t cali_offset;     // 运行时是否减去静态零偏
    BMI088_AsyncContext_t async;
} BMI088_Context_t;

// BMI088 模块内部统一上下文; core/init/async 共享这一个状态对象
extern BMI088_Context_t bmi088_ctx;

// 核心寄存器访问接口
void bmi088_write_accel_reg(uint8_t reg, uint8_t data);
void bmi088_read_accel_reg(uint8_t reg, uint8_t *data);
void bmi088_read_accel_regs(uint8_t reg, uint8_t *buf, uint8_t len);
void bmi088_write_gyro_reg(uint8_t reg, uint8_t data);
void bmi088_read_gyro_reg(uint8_t reg, uint8_t *data);
void bmi088_read_gyro_regs(uint8_t reg, uint8_t *buf, uint8_t len);

// DMA 回调里复用的数据解析逻辑
void bmi088_parse_gyro_frame(const uint8_t *rx_buf);
void bmi088_parse_accel_frame(const uint8_t *rx_buf);
void bmi088_parse_temp_frame(const uint8_t *rx_buf);

// 运行期异步传输状态机
uint8_t bmi088_start_gyro_dma_transfer(void);
uint8_t bmi088_start_accel_dma_transfer(void);
uint8_t bmi088_start_temp_dma_transfer(void);
void bmi088_service_pending_transfer(void);
void bmi088_async_reset_state(void);
void BMI088_AsyncPoll(void);

#endif
