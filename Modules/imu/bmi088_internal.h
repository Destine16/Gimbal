#ifndef BMI088_INTERNAL_H
#define BMI088_INTERNAL_H

#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "BMI088reg.h"
#include "bsp_dwt.h"

#include <math.h>
#include <string.h>

// BMI088 共享换算系数与对外样本
extern float BMI088_ACCEL_SEN;
extern float BMI088_GYRO_SEN;
extern IMU_Data_t BMI088;

// 静态标定结果与调试观测量
extern float gyroDiff[3], gNormDiff; // 陀螺零偏波动与重力模长波动
extern uint8_t caliOffset;           // 运行时是否减去静态零偏
extern int16_t caliCount;            // 标定重试次数统计

typedef enum
{
    BMI088_ASYNC_IDLE = 0,   // SPI 空闲,可以启动新的 BMI088 传输
    BMI088_ASYNC_GYRO_BUSY,  // 当前正在进行 gyro DMA 读
    BMI088_ASYNC_ACCEL_BUSY, // 当前正在进行 accel DMA 读
    BMI088_ASYNC_TEMP_BUSY,  // 当前正在进行温度 DMA 读
} BMI088_AsyncState_e;

// 运行期异步采集状态
extern volatile uint8_t bmi088_async_enabled;            // 是否启用 EXTI + DMA 异步链路
extern volatile uint8_t bmi088_async_valid;              // 是否已经形成过至少一帧有效样本
extern volatile uint8_t bmi088_have_gyro;                // 当前样本周期内 gyro 是否已更新
extern volatile uint8_t bmi088_have_accel;               // 当前样本周期内 accel 是否已更新
extern volatile uint8_t bmi088_pending_gyro;             // SPI 忙时挂起的 gyro 触发
extern volatile uint8_t bmi088_pending_accel;            // SPI 忙时挂起的 accel 触发
extern volatile uint32_t bmi088_async_seq;               // 完整新样本序号
extern volatile BMI088_AsyncState_e bmi088_async_state;  // 当前异步状态机状态

extern uint8_t bmi088_spi_tx_buf[9]; // DMA 发送缓冲区
extern uint8_t bmi088_spi_rx_buf[9]; // DMA 接收缓冲区

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

#endif
