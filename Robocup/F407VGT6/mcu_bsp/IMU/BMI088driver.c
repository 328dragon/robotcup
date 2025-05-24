/**
  ********************************************************************************************************************************************************
  * @file    BMI088driver.c
  * @author  DJI Technology Co., Ltd.
  * @brief   BMI088惯性测量单元(IMU)驱动核心文件
  *
  * @attention
  * 本文件实现BMI088加速度计和陀螺仪的核心驱动功能，包括：
  *          - 传感器初始化及配置
  *          - 三轴运动数据采集
  *          - 温度数据读取
  *          - 寄存器级错误检测
  *
  * @section 硬件依赖
  * 1. 通信接口：
  *    - 必须定义BMI088_USE_SPI或BMI088_USE_IIC宏
  *    - SPI模式要求：CPOL=1, CPHA=1
  * 2. 灵敏度配置：
  *    - BMI088_ACCEL_SEN：加速度计量程系数(默认±3g)
  *    - BMI088_GYRO_SEN：陀螺仪量程系数(默认±2000dps)
  *
  * @section 使用限制
  * 1. 初始化时序：
  *    - 必须调用BMI088_init()完成硬件初始化
  *    - 复位后需等待BMI088_LONG_DELAY_TIME(≥30ms)
  * 2. 数据读取：
  *    - 需按传感器采样率调用BMI088_read()
  *    - 温度数据有±5℃的绝对误差
  *
  * @section 数据转换公式
  * 1. 加速度值(m/s?) = RAW_DATA × BMI088_ACCEL_SEN
  * 2. 角速度值(rad/s) = RAW_DATA × BMI088_GYRO_SEN
  * 3. 温度值(℃) = (RAW_DATA × 0.125) + 23.0
  * 
  * @note 编码方式：
  *      - 务必适用GB2312编码
  * 
  * @note 联系方式：
  *      - 7415 2789152534@qq.com
  ********************************************************************************************************************************************************
  */


  /**
   * @note： ATTENTION!!!
   * 在开始前请阅读这段话：
   *   - 请在main函数的初始化后采取如下的启动方式：
   * ----------------------------------------------------------------------------------------------------------------
   *     while(BMI088_init()){} // 确保BMI的正确启动 
   * ----------------------------------------------------------------------------------------------------------------
   * 
   *   - 请在it.c中使用定时器中断，每隔一段时间进行数据采样（建议典型值为；10ms采样一次），采样函数如下：
   * ----------------------------------------------------------------------------------------------------------------
   *     BMI088_read(gyro, accel, &temp); // 调用进行采样
   * ----------------------------------------------------------------------------------------------------------------
   *   - 为了方便取用BMI所存储的数值，定义了结构体变量bmi088_feedback_data，详见BMI088driver.h文件。
   */



#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

#include <stdio.h>
#include <string.h>

/*加速度计灵敏度，可更改量程*/
fp32 BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
/*陀螺仪灵敏度，可更改量程*/
fp32 BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

fp32 gyro[3], accel[3], temp;

bmi_feedback_data_t bmi088_feedback_data;

#if defined(BMI088_USE_SPI)

/*加速度相关宏*/
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

/*陀螺仪相关宏*/
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)


#endif
/**
 * @param: 决定二维数组的行数
 * @brief：数组中的每一行低矮报一个需要配置的加速度寄存器，包含三个元素：寄存器地址，要写入的值，错误码
 */
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};
/**
 * @param: 决定二维数组的行数
 * @brief：数组中的每一行低矮报一个需要配置的陀螺仪寄存器，包含三个元素：寄存器地址，要写入的值，错误码
 */
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

/**
 * @brief  BMI088加速度计初始化函数
 * 
 * @details 本函数完成BMI088加速度计的完整初始化流程，包括：
 *          1. 通过读取芯片ID验证通信链路
 *          2. 执行软件复位操作
 *          3. 复位后再次验证通信
 *          4. 配置所有必要寄存器并校验配置结果
 * 
 * @return bool_t 初始化状态
 *         @retval BMI088_NO_ERROR 初始化成功
 *         @retval BMI088_NO_SENSOR 芯片ID验证失败（未检测到传感器）
 *         @retval BMI088_ACC_*_ERROR 寄存器配置失败（具体错误码见返回值定义）
 * 
 * @section 典型调用流程
 * @code
 * if(bmi088_accel_init() != BMI088_NO_ERROR) {
 *     // 错误处理
 * }
 */

bool_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}
/*同上，为陀螺仪初始化*/
bool_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}


/**
 * @brief  读取BMI088传感器数据（加速度/角速度/温度）
 * 
 * @details 本函数通过SPI/I2C接口同步读取：
 *          - 三轴加速度值（m/s?）
 *          - 三轴角速度值（rad/s）
 *          - 温度值（℃）
 *          原始数据经过量程灵敏度换算和温度补偿处理
 * 
 * @param[out] gyro      三轴角速度输出数组（rad/s）
 *                       数组顺序: [0]-X轴, [1]-Y轴, [2]-Z轴
 * @param[out] accel     三轴加速度输出数组（m/s?）
 *                       数组顺序: [0]-X轴, [1]-Y轴, [2]-Z轴
 * @param[out] temperate 温度值输出指针（℃）
 * 
 * @note 使用前必须确保：
 *       - 传感器已成功初始化（bmi088_accel_init/bmi088_gyro_init）
 *       - BMI088_ACCEL_SEN/BMI088_GYRO_SEN已根据量程正确配置
 *       - 调用间隔需满足传感器采样率要求
 * 
 * @section 数据转换说明
 * 1. 加速度数据：
 *    - 从BMI088_ACCEL_XOUT_L开始连续读取6字节
 *    - 原始值(int16) × BMI088_ACCEL_SEN → 物理量(m/s?)
 * 
 * 2. 陀螺仪数据：
 *    - 读取时会验证芯片ID(BMI088_GYRO_CHIP_ID)
 *    - 原始值(int16) × BMI088_GYRO_SEN → 物理量(rad/s)
 * 
 * 3. 温度数据：
 *    - 从BMI088_TEMP_M读取2字节
 *    - 原始值(int11) × 0.125 + 23 → 温度值(℃)
 */
void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET; 

    bmi088_feedback_data.g_x = gyro[0];
    bmi088_feedback_data.g_y = gyro[1];
    bmi088_feedback_data.g_z = gyro[2];
    bmi088_feedback_data.a_x = accel[0];
    bmi088_feedback_data.a_y = accel[1];
    bmi088_feedback_data.a_z = accel[2];
    bmi088_feedback_data.tem = *temperate;
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}

// /**
//  * 向VOFA上报，采用了VOFA定义的FireWater协议。
//  * @param *huart:需要上报的串口
//  */
// void Send_BMI088_To_VOFA(UART_HandleTypeDef *huart,float gyro[3], float accel[3], float temp)
// {
//     char buffer[128]; // 缓冲区
//     int length = 0;

//     // 格式化数据（FireWater 协议：逗号分隔 + \r\n 结尾）
//     length = snprintf(buffer, sizeof(buffer),
// 			"d:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f\r\n",  // 格式
//         gyro[0], gyro[1], gyro[2],                  // 陀螺仪 X,Y,Z
//         accel[0], accel[1], accel[2],               // 加速度计 X,Y,Z
//         temp                                        // 温度
//     );

//     // 通过串口 发送
//     HAL_UART_Transmit(huart, (uint8_t*)buffer, length, HAL_MAX_DELAY);
// }


static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
