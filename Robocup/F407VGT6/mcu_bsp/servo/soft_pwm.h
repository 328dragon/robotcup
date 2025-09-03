#ifndef SOFT_PWM_H
#define SOFT_PWM_H
#include <stdint.h>
#include "main.h"

//封装后
// PWM通道配置结构体
typedef struct {
    uint16_t period;       // 周期值
    uint16_t high;         // 高电平值
    uint16_t cnt;          // 计数器
    GPIO_TypeDef* port;    // GPIO端口
    uint16_t pin;          // GPIO引脚
} SoftPwmChannel;

// 初始化PWM通道
void SoftPwmInit(SoftPwmChannel* channel, GPIO_TypeDef* port, 
                uint16_t pin, uint16_t period, uint16_t high);

// 设置PWM周期
void SoftPwmSetPeriod(SoftPwmChannel* channel, uint16_t period);

// 设置PWM高电平时间
void SoftPwmSetHigh(SoftPwmChannel* channel, uint16_t high);

// 定时器中断服务函数（需要传入通道数组和数量）
void SoftPwmTimerISR(SoftPwmChannel* channel);

extern SoftPwmChannel soft_pwm_f;

#endif
