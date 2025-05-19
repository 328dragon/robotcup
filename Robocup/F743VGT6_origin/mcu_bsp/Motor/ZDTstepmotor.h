#ifndef __ZDTSTEPMOTOR_H
#define __ZDTSTEPMOTOR_H

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "motor_def.h"

typedef struct 
{
    Motor_Controller_struct motor_controller_t;
    UART_HandleTypeDef *_USART;
    int8_t _dir; // 正转方向
    int16_t _target_rpm ;
    float _wheel_diameter;     // 轮子直径
    bool _have_pub_permission; // 是否有发布权限
    uint8_t _cmd_buffer[20];     // 命令缓冲区
}StepMotorZDT_t;

void Step_ZDT_Init(StepMotorZDT_t *zdt_mot,  uint32_t id ,UART_HandleTypeDef *_USART,int8_t _dir, float _wheel_diameter, bool _have_pub_permission);

void set_speed_target(StepMotorZDT_t *zdt_motor, float target);
void set_speed_pos_target(StepMotorZDT_t *zdt_motor, float target_speed, float target_pos);
float get_linear_speed(StepMotorZDT_t* zdt_motor);

#endif
