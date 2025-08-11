#include "servo.h"

void Servo_SetAngle(Servo_t *servo, float angle,float max_angle)
{
    servo->target_CCR = (angle * (SERVO_MAX - SERVO_MIN) / max_angle) + SERVO_MIN;
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->target_CCR);
    servo->current_angle = angle;
}