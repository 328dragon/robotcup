#ifndef SOFT_PWM_H
#define SOFT_PWM_H
#include <stdint.h>
#include "main.h"
#include "string.h"

#define SOFT_PWM_CNT 5
#define SOFT_PWM_BASE_TIM_PERIOD 100 // 基础定时器周期，单位为us

#define servo_zero 0.5  //ms
#define servo_max  2.5
//封装后
// PWM通道配置结构体
typedef struct {
    float period;       // 周期值
    float high;         // 高电平值
    float cnt;          // 计数器
    GPIO_TypeDef* port;    // GPIO端口
    float pin;          // GPIO引脚
	int angel_max;
} SoftPwmChannel;


//注册软件pwm
void SOFTPWMRegister(SoftPwmChannel* instance, GPIO_TypeDef* port, 
                float pin, float period,int max_angle);

// 设置PWM周期
void SoftPwmSetPeriod(SoftPwmChannel* _instance, float period);

void SoftSetAngle(SoftPwmChannel* _channel, int  angle);
// 定时器中断服务函数（需要传入通道数组和数量）
void SoftPwmTimerISR() ;


#endif
