#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "stdint.h"
#include "main.h"

// 温度换算: Temperature = raw * factor + offset
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

// 初始化配置表长度
#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

// 驱动内部的数据就绪标志位编号
#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

// BMI088 上电和配置阶段的阻塞等待时间常量
#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

// BMI088 芯片 I2C 地址; 当前工程实际使用 SPI,这里只是保留芯片级常量
#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

// 不同量程下 accel 原始计数到工程量的换算系数
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

// 不同量程下 gyro 原始计数到角速度工程量的换算系数
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

// 默认离线标定参数；当前工程优先使用在线静态标定
#define GxOFFSET 0.0f
#define GyOFFSET 0.0f
#define GzOFFSET 0.0f
#define gNORM 9.81f

/* IMU数据结构体 */
typedef struct
{
    // 当前三轴加速度工程量
    float Accel[3];

    // 当前三轴角速度工程量
    float Gyro[3];

    // 标定时温度与当前温度
    float TempWhenCali;
    float Temperature;

    // 加速度模长修正系数; 运行时通常用 9.81 / gNorm
    float AccelScale;
    // 陀螺仪静态零偏
    float GyroOffset[3];

    // 静态标定得到的重力模长
    float gNorm;
} IMU_Data_t;

/* BMI088初始化/自检错误码 */
enum
{
    BMI088_NO_ERROR = 0x00,
    // accel 初始化阶段寄存器配置失败
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    // gyro 初始化阶段寄存器配置失败
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    // 自检失败标志位
    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    // 读不到有效 chip id 或器件未响应
    BMI088_NO_SENSOR = 0xFF,
};

extern IMU_Data_t BMI088;

/**
 * @brief 初始化BMI088,传入连接的SPI总线handle,以及是否进行在线标定
 * 
 * @param bmi088_SPI handle
 * @param calibrate  1为进行在线标定,0使用离线数据
 * @return uint8_t   成功则返回BMI088_NO_ERROR
 */
extern uint8_t BMI088Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate);

/**
 * @brief 加速计初始化
 * 
 * @return uint8_t 
 */
extern uint8_t bmi088_accel_init(void);

/**
 * @brief 陀螺仪初始化
 * 
 * @return uint8_t 
 */
extern uint8_t bmi088_gyro_init(void);

/**
 * @brief 读取一次BMI088的数据,包括gyro和accel
 * 
 * @param bmi088 传入BMI088实例(结构体)
 */
extern void BMI088_Read(IMU_Data_t *bmi088);

/**
 * @brief 启用运行期BMI088异步采集; 初始化和静态标定阶段仍使用阻塞读
 */
extern void BMI088_AsyncEnable(void);

/**
 * @brief 关闭运行期BMI088异步采集
 */
extern void BMI088_AsyncDisable(void);

/**
 * @brief 安全复制最新一帧BMI088数据
 *
 * @param bmi088 目标缓存
 * @param seq    当前样本序号,可为空
 * @return uint8_t 1表示已经有有效异步样本,0表示还没有
 */
extern uint8_t BMI088_FetchData(IMU_Data_t *bmi088, uint32_t *seq);

// HAL EXTI / SPI DMA 完成回调最终会转到这里
extern void BMI088_EXTI_Callback(uint16_t GPIO_Pin);
extern void BMI088_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
extern void BMI088_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

#endif
