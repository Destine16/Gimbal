#include "bmi088_internal.h"

// BMI088 共享状态: 对外样本、标定参数、运行期异步采集状态
float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

float gyroDiff[3], gNormDiff; // 静态标定时用于判断是否稳定: 三轴零偏波动和重力模长波动
uint8_t caliOffset = 1;       // 运行时是否减去静态标定得到的 gyro offset
int16_t caliCount = 0;        // 标定重试次数统计

IMU_Data_t BMI088;

volatile uint8_t bmi088_async_enabled = 0;   // 运行期是否启用 BMI088 异步采集
volatile uint8_t bmi088_async_valid = 0;     // 是否已经拿到过至少一帧有效异步样本
volatile uint8_t bmi088_have_gyro = 0;       // 当前样本周期内是否已收到 gyro 数据
volatile uint8_t bmi088_have_accel = 0;      // 当前样本周期内是否已收到 accel 数据
volatile uint8_t bmi088_pending_gyro = 0;    // gyro EXTI 到来但 SPI 忙,等待稍后补启动
volatile uint8_t bmi088_pending_accel = 0;   // accel EXTI 到来但 SPI 忙,等待稍后补启动
volatile uint32_t bmi088_async_seq = 0;      // 异步样本序号; 每形成一帧新的完整 BMI088 样本后递增,供上层判断这次取到的是新数据还是旧数据
volatile BMI088_AsyncState_e bmi088_async_state = BMI088_ASYNC_IDLE; // 当前 BMI088 异步采集状态

uint8_t bmi088_spi_tx_buf[9];                // BMI088 SPI DMA 发送缓冲区
uint8_t bmi088_spi_rx_buf[9];                // BMI088 SPI DMA 接收缓冲区
