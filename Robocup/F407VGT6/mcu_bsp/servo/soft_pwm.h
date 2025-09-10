#ifndef SOFT_PWM_H
#define SOFT_PWM_H
#include <stdint.h>
#include "main.h"
#include "string.h"

#define SOFT_PWM_CNT 5
#define SOFT_PWM_BASE_TIM_PERIOD 100 // 基础定时器周期，单位为微秒

#define Servo_zero  500
#define Servo_max		2500
//封装后
// PWM通道配置结构体
typedef struct {
    uint16_t period;       // 周期值
	uint16_t increment;				//每次增量
    uint16_t high_time;         // 高电平值
    uint16_t cnt;          // 计数器
    GPIO_TypeDef* port;    // GPIO端口
		int sevo_max;
    uint16_t pin;          // GPIO引脚
    uint16_t real_us_cnt;
} SoftPwmChannel;

//注册软件pwm
void SOFTPWMRegister(SoftPwmChannel* instance, GPIO_TypeDef* port, 
                uint16_t pin, uint16_t period, uint16_t high,int _sevo_max);

// 设置PWM周期
void SoftPwmSetPeriod(SoftPwmChannel* _instance, uint16_t period);
//设置角度
void Setangle(SoftPwmChannel* _channel, uint16_t angle);

// 定时器中断服务函数（需要传入通道数组和数量）
void SoftPwmTimerISR() ;


#endif
