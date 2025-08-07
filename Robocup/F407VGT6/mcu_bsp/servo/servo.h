#ifndef _SERVO_H_
#define _SERVO_H_

#include "main.h"
#include "tim.h"

#define SERVO_MAX 2500
#define SERVO_MIN 500

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t current_angle;
    uint32_t target_CCR; // 目标比较值
} Servo_t; 
// 25kg舵机对应时钟通道PSC为84-1, ARR为20000-1，APB1时钟线（TIM3/5），为84MHz
// 蓝色小舵机对应时钟通道PSC为168-1, ARR为20000-1，APB2时钟线（TIM9），为168MHz
// 故25kg舵机的CCR的范围为
void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(Servo_t *servo, uint32_t angle);

#endif