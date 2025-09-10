#include "soft_pwm.h"
static uint8_t idx;
static SoftPwmChannel *soft_pwm_instance[SOFT_PWM_CNT]={NULL};\
float angle_high=0;
///封装后
// 初始化PWM通道
void SoftPwmInit(SoftPwmChannel* _channel, GPIO_TypeDef* port, 
                float pin, float period,int max_angle) {  
    _channel->port = port;
    _channel->pin = pin;
    _channel->period = (period == 0) ? 1 : ((period*1000)/SOFT_PWM_BASE_TIM_PERIOD);
    _channel->cnt = 0;
}
//注册软件pwm
void SOFTPWMRegister(SoftPwmChannel* instance, GPIO_TypeDef* port, 
                float pin, float period,int max_angle) {
   memset(instance, 0, sizeof(SoftPwmChannel));
   soft_pwm_instance[idx++] = instance;               
   SoftPwmInit(instance, port, pin, period,max_angle);
}

// 设置PWM周期
void SoftPwmSetPeriod(SoftPwmChannel* _instance, float period) {
    if (_instance == NULL) return;
    _instance->period = (period == 0) ? 1 : ((period*1000)/SOFT_PWM_BASE_TIM_PERIOD);
    // 确保高电平时间不超过新周期

}


//单位为us
// 设置PWM高电平时间
void SoftPwmSetHigh(SoftPwmChannel* _channel, float high) {
    if (_channel == NULL) return;

	float high_temp=high*1.00f/SOFT_PWM_BASE_TIM_PERIOD;
	
    _channel->high = (high_temp > _channel->period) ? _channel->period : high_temp;

}
void SoftSetAngle(SoftPwmChannel* _channel, int  angle)
{
angle_high=(angle*1.0f/180)*(servo_max-servo_zero)*1000+500;
	
	SoftPwmSetHigh(_channel,angle_high);
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