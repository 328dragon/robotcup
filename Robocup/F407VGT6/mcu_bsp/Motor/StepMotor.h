//普通步进电机_库，不用延迟这个抽象方法，由于只与最底层时钟有关，可以看作和时钟没关系
#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

#define BUFFER_SIZE 500  // 缓冲区大小越大占用cpu时间越少
#define ITERATIONS 10    // 缓起缓停迭代次数
#define ITERATIONPULSE 8 // 缓起缓停迭代角度

typedef enum {
    BUSY,          // 正在输出脉冲
    IDLE,          // 空闲
    WaitIteration, // 等待迭代
    Start          // 启动
} StepMotorState;

typedef struct {
    /*缓启缓停相关*/
    uint16_t _iteration;                    // 当前迭代次数
    uint32_t _soft_target_pulse;            // 缓启缓停的目标脉冲数
    float _soft_target_rpm;                 // 缓启缓停的目标最大转速
    uint16_t _itertration_num;              // 缓启缓停的迭代次数
    uint16_t _iteration_pulse;              // 缓启缓停的每圈转速对应的脉冲数
    
    /*步进电机参数相关*/
    float _StepAngle;   // 步进角
    uint8_t _Subdivision; // 细分
    
    /*给一定数量脉冲实现相关*/
    uint16_t _pulse_mod;           // 取余的脉冲数
    uint32_t _target_pulse;        // 给一串脉冲的目标脉冲数
    uint32_t _target_freq;         // 给一串脉冲的目标频率
    uint16_t _target_number;        // 给一串脉冲的目标次数
    StepMotorState _state;
    
    /*硬件外设相关*/
    TIM_HandleTypeDef *_tim;
    GPIO_TypeDef *_ph_port;
    uint32_t _channel;
    uint16_t _ph_pin;                         // 预分频
    uint32_t _clock_base_frequency;           // 时钟频率经过预分频后的频率
    uint8_t _resolution;                      // 分辨率
    uint8_t _buffer[BUFFER_SIZE];
} StepMotor_t;

// 函数声明
void StepMotor_Init(StepMotor_t *motor, TIM_HandleTypeDef *tim, uint32_t channel, GPIO_TypeDef *ph_port, uint16_t ph_pin);
void StepMotor_setParam(StepMotor_t *motor, float StepAngle, uint8_t Subdivision, uint16_t target_iteration_num, uint16_t itertion_pulse);
bool StepMotor_isBusy(StepMotor_t *motor);
void StepMotor_update(StepMotor_t *motor, uint16_t dt);
void StepMotor_giveRPMPulseSoft(StepMotor_t *motor, float rpm, uint32_t pulse, uint16_t target_iteration_num, uint16_t itertion_pulse);
void StepMotor_giveRPMPulse(StepMotor_t *motor, float rpm, uint32_t pulse);
void StepMotor_giveRPMAngle(StepMotor_t *motor, float rpm, float angle);
void StepMotor_dmaCallBack(StepMotor_t *motor, TIM_HandleTypeDef *htim);
uint32_t StepMotor_angleToPulse(StepMotor_t *motor, float angle);

#endif