#include "soft_pwm.h"
static uint8_t idx;
static SoftPwmChannel *soft_pwm_instance[SOFT_PWM_CNT]={NULL};
///封装后
// 初始化PWM通道
void SoftPwmInit(SoftPwmChannel* _channel, GPIO_TypeDef* port, 
                uint16_t pin, uint16_t period, uint16_t high) {  
    _channel->port = port;
    _channel->pin = pin;
    _channel->period = (period == 0) ? 1 : period;
    _channel->high = (high > _channel->period) ? _channel->period : high;
    _channel->cnt = 0;
}
//注册软件pwm
void SOFTPWMRegister(SoftPwmChannel* instance, GPIO_TypeDef* port, 
                uint16_t pin, uint16_t period, uint16_t high) {
   memset(instance, 0, sizeof(SoftPwmChannel));
   soft_pwm_instance[idx++] = instance;               
   SoftPwmInit(instance, port, pin, period, high);
}

// 设置PWM周期
void SoftPwmSetPeriod(SoftPwmChannel* _instance, uint16_t period) {
    if (_instance == NULL) return;
    _instance->period = (period == 0) ? 1 : period;
    // 确保高电平时间不超过新周期
    if (_instance->high > _instance->period) {
        _instance->high = _instance->period;
    }
}

// 设置PWM高电平时间
void SoftPwmSetHigh(SoftPwmChannel* _channel, uint16_t high) {
    if (_channel == NULL) return;
    _channel->high = (high > _channel->period) ? _channel->period : high;
}

// 定时器中断服务函数
void SoftPwmTimerISR() {
    for(int i=0; i<idx; i++) {
        // 更新计数器
        soft_pwm_instance[i]->cnt++;
        
        // 控制GPIO电平
        if (soft_pwm_instance[i]->cnt <= soft_pwm_instance[i]->high) {
            HAL_GPIO_WritePin(soft_pwm_instance[i]->port, soft_pwm_instance[i]->pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(soft_pwm_instance[i]->port, soft_pwm_instance[i]->pin, GPIO_PIN_RESET);
        }
        
        // 计数器溢出重置
        if (soft_pwm_instance[i]->cnt >= soft_pwm_instance[i]->period) {
            soft_pwm_instance[i]->cnt = 0;
        }
    }
}