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
void set_speed_target(StepMotorZDT_t* zdt_motor,float target);

#endif
