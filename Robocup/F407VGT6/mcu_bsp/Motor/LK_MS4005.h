#ifndef __LK_MS4005_H
#define __LK_MS4005_H

#include "stdint.h"
#include "motor_def.h"
#include "can.h"
#include "bsp_can.h"
#include "m_math.h"
#define LK_MS4005_NUM 4 // 最多支持4个电机
typedef enum
{   
    initial_mode=0,//初始模式
    vel_close_loop_mode=1,//速度闭环控制
    multiple_position_close_loop_mode=2,//多圈位置闭环控制
    incremental_position_close_loop_mode=3,//增量位置闭环控制
} LK_MS4005_Working_MODE_e;

//便于发送数据转换
typedef  struct
{
    //不完全是硬件信息，还有一些控制信息
    int32_t speedCtrl_trans;
    int16_t iqCtrl_trans;
    int32_t  angle_Ctrl_trans;
    uint16_t maxspeed_trans;
    int32_t angleIncrement_trans;
    //全是硬件信息

} LK_MS4005_Data_Hardware_t;



typedef struct lk_ms4005_t
{
    //通用参数
    uint32_t protocol_id;         // 协议id
    uint32_t mode_trans_id;       // 工作模式id,用于区分不同的工作模式
    CANInstance *can_instance;      // can实例
    Motor_Controller_struct motor_instnce;//电机实例
    //电机个性化参数
    LK_MS4005_Working_MODE_e wroking_mode; // 电机工作模式
    LK_MS4005_Data_Hardware_t hardware_t; // 硬件信息
} LK_MS4005_Controller_t;

#endif

void LK_MS4005_Get_Info(CANInstance *can_instance);

LK_MS4005_Controller_t *LK_MS4005_Register(CAN_HandleTypeDef *hcan, uint32_t protocol_id, uint16_t w_mode);

void Enable_LK(LK_MS4005_Controller_t *lk_ms4005_instance);

void Vel_Ctrl_LK(LK_MS4005_Controller_t *lk_ms4005_instance, float iqControl, float target_Angular_velocity);

void Control_LK_MS4005(LK_MS4005_Controller_t *lk_ms4005_instance);
