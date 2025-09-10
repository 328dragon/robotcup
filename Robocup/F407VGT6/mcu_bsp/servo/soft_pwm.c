#include "soft_pwm.h"
static uint8_t idx;
static SoftPwmChannel *soft_pwm_instance[SOFT_PWM_CNT]={NULL};
///封装后
// 初始化PWM通道
//_period单位也是us
static  void SoftPwmInit(SoftPwmChannel* _channel, GPIO_TypeDef* port, 
                uint16_t pin, uint16_t _period, uint16_t high,int _sevo_max) {  
    _channel->port = port;
    _channel->pin = pin;
    _channel->period = (_period == 0) ? 1 : _period;
		_channel->increment=	_channel->period/SOFT_PWM_BASE_TIM_PERIOD;	//每次增量				
    _channel->high_time = (high > _channel->period) ? _channel->period : high;
    _channel->cnt = 0;
		_channel->sevo_max=_sevo_max;
}

//注册软件pwm
void SOFTPWMRegister(SoftPwmChannel* instance, GPIO_TypeDef* port, 
                uint16_t pin, uint16_t period, uint16_t high,int _sevo_max) {
   memset(instance, 0, sizeof(SoftPwmChannel));
   soft_pwm_instance[idx++] = instance;               
   SoftPwmInit(instance, port, pin, period, high,_sevo_max);
}

// 设置PWM周期
void SoftPwmSetPeriod(SoftPwmChannel* _instance, uint16_t period) {
    if (_instance == NULL) return;
    _instance->period = (period == 0) ? 1 : period;
    // 确保高电平时间不超过新周期
    if (_instance->high_time > _instance->period) {
        _instance->high_time = _instance->period;
    }
}



// 设置PWM高电平时间
void SoftPwmSetHigh(SoftPwmChannel* _channel, uint16_t _high_time) {
    if (_channel == NULL) return;
    _channel->high_time = (_high_time > _channel->period) ? _channel->period : _high_time;
}


//angle,是按照度来的，角度不会特别高哈
void Setangle(SoftPwmChannel* _channel, uint16_t angle)
{
uint16_t angle_high_time=((Servo_max-Servo_zero)/_channel->sevo_max)*angle;//每度对应us数*对应角度 
		
SoftPwmSetHigh(_channel,angle_high_time);
}


// 定时器中断服务函数
void SoftPwmTimerISR() {
    for(int i=0; i<idx; i++) {
			
        // 更新计数器,SOFT_PWM_BASE_TIM_PERIOD （us）进一次
        soft_pwm_instance[i]->cnt++;
soft_pwm_instance[i]->real_us_cnt=soft_pwm_instance[i]->cnt*soft_pwm_instance[i]->increment;
			
        // 控制GPIO电平
        if (soft_pwm_instance[i]->real_us_cnt <= soft_pwm_instance[i]->high_time) {
            HAL_GPIO_WritePin(soft_pwm_instance[i]->port, soft_pwm_instance[i]->pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(soft_pwm_instance[i]->port, soft_pwm_instance[i]->pin, GPIO_PIN_RESET);
        }
        
        // 计数器溢出重置
        if (soft_pwm_instance[i]->cnt >= soft_pwm_instance[i]->period) {
            soft_pwm_instance[i]->cnt = 0;
					soft_pwm_instance[i]->real_us_cnt=0;
        }
    }
}