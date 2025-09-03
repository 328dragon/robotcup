#include "soft_pwm.h"
#include "main.h"
volatile uint16_t g_pwm_period = 200;  	//控制舵机（定时器中断间隔0.1ms，总周期20ms）
volatile uint16_t g_pwm_cnt = 0;
volatile uint16_t g_pwm_high = 5;    	//默认高电平时间（0.5ms到2.5ms）

static void set_pwm_gpio(uint8_t gpio_state)	//设置用于模拟pwm的gpio电平
{
	if(gpio_state == 1) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	if(gpio_state == 0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
}

void SoftPwmTimerISR(void)		//用在定时器中断里
{
    g_pwm_cnt++;

    if(g_pwm_cnt <= g_pwm_high)
	{
		set_pwm_gpio(1);
    }
	else
	{
		set_pwm_gpio(0);
    }

	if(g_pwm_cnt >= g_pwm_period)
	{
		g_pwm_cnt = 0;
	}
}

void SoftPwmSetPeriod(uint16_t period)
{
    //限制最小周期为1，避免除零错误
    if (period == 0) period = 1;

    g_pwm_period = period;
}

void SoftPwmSetHigh(uint16_t high)
{
    //高电平时间不超过周期
    g_pwm_high = (high > g_pwm_period) ? g_pwm_period : high;
}