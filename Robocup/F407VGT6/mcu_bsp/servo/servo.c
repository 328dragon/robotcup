#include "servo.h"

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel) 
{
    servo->htim = htim;
    servo->channel = channel;
    servo->current_angle = 0;
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

void Servo_SetAngle(Servo_t *servo, float angle,float max_angle)
{
    servo->target_CCR = (angle * (SERVO_MAX - SERVO_MIN) / max_angle) + SERVO_MIN;
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->target_CCR);
    servo->current_angle = angle;
}
