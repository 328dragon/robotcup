/**
 **************************************************************************************************************************************
  * @file    BMI088Middleware.c
  * @author  DJI Technology Co., Ltd. 大疆创新
  * @brief   BMI088传感器硬件抽象层(HAL)实现文件,服务于顶层BMI088driver.c文件。
  *
  * @attention
  * 本文件提供BMI088加速度计/陀螺仪的底层硬件操作接口，包括：
  *          - SPI通信控制
  *          - 精确延时函数
  *          - 片选信号管理
  * 
  * @note 硬件依赖：
  *       - STM32 HAL库（需已初始化SPI1和GPIO）
  *       - 系统时钟必须配置正确（影响延时函数精度）
  *       - 硬件连接要求：
  *           CS1_ACCEL_Pin -> 加速度计片选引脚
  *           CS1_GYRO_Pin  -> 陀螺仪片选引脚
  *           hspi1         -> 共享SPI接口
  * 
  * @note 使用限制：
  *       1. BMI088_delay_us()的168倍率需根据实际CPU主频调整：
  *          - 168MHz系统时钟：ticks = us * 168
  *          - 84MHz系统时钟： ticks = us * 84
  *       2. 必须调用BMI088_GPIO_init()和BMI088_com_init()完成硬件初始化
  *       3. SPI参数要求：
  *          - 模式: CPOL=1, CPHA=1
  *          - 速率: ≤10MHz（建议值）
  * 
  * @note 编码方式：
  *      - 务必适用GB2312编码
  * 
  * @note 联系方式：
  *      - 7415 2789152534@qq.com
  **************************************************************************************************************************************
  */
	
	
#include "BMI088Middleware.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us)
{

    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 168;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }


}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi3, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

