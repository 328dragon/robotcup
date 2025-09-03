#include "soft_pwm.h"
#define SOFT_PWM_CNT 1
SoftPwmChannel soft_pwm_f={NULL};

///封装后
// 初始化PWM通道
void SoftPwmInit(SoftPwmChannel* channel, GPIO_TypeDef* port, 
                uint16_t pin, uint16_t period, uint16_t high) {  
    channel->port = port;
    channel->pin = pin;
    channel->period = (period == 0) ? 1 : period;
    channel->high = (high > channel->period) ? channel->period : high;
    channel->cnt = 0;
}

// 设置PWM周期
void SoftPwmSetPeriod(SoftPwmChannel* channel, uint16_t period) {
    if (channel == NULL) return;
    channel->period = (period == 0) ? 1 : period;
    // 确保高电平时间不超过新周期
    if (channel->high > channel->period) {
        channel->high = channel->period;
    }
}

// 设置PWM高电平时间
void SoftPwmSetHigh(SoftPwmChannel* channel, uint16_t high) {
    if (channel == NULL) return;
    channel->high = (high > channel->period) ? channel->period : high;
}

// 定时器中断服务函数
void SoftPwmTimerISR(SoftPwmChannel* channel) {
    
        // 更新计数器
        channel->cnt++;
        
        // 控制GPIO电平
        if (channel->cnt <= channel->high) {
            HAL_GPIO_WritePin(channel->port, channel->pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(channel->port, channel->pin, GPIO_PIN_RESET);
        }
        
        // 计数器溢出重置
        if (channel->cnt >= channel->period) {
            channel->cnt = 0;
        }

}