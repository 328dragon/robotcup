/*
 * @Author: Nagisa 2964793117@qq.com
 * @Date: 2025-08-07 20:57:32
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2025-08-08 15:58:28
 * @FilePath: \MDK-ARMd:\project\git\robotcup\Robocup\F407VGT6\mcu_bsp\servo\servo.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
    float current_angle;
    uint32_t target_CCR; // 目标比较值
} Servo_t; 
// 25kg舵机对应时钟通道PSC为84-1, ARR为20000-1，APB1时钟线（TIM3/5），为84MHz
// 蓝色小舵机对应时钟通道PSC为168-1, ARR为20000-1，APB2时钟线（TIM9），为168MHz
// 故25kg舵机的CCR的范围为
void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(Servo_t *servo, float angle,float max_angle);


#endif