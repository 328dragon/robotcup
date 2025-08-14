/*
 * @Author: Nagisa 2964793117@qq.com
 * @Date: 2025-08-07 22:06:30
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2025-08-09 00:24:46
 * @FilePath: \MDK-ARMd:\project\git\robotcup\Robocup\F407VGT6\mcu_bsp\servo\upper.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _UPPER_H_
#define _UPPER_H_
/*
 * @file upper.h 该工程包括了上层机构的逻辑和接口函数实现
 */
#include "main.h"
#include "servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdbool.h"
// 所有舵机的常量或枚举常量
#define THING_GIMBAL_FIXED_DELTA 60
#define THING_GIMBAL_ORIGIN_ANGLE 0
#define PUMP_ON HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, 1);
#define PUMP_OFF HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, 0);
#define servo_zero 10
/*
 * @brief 前云台枚举常量
 */
typedef enum
{
    // 跟逻辑有关
    GOAL_PLACE = 83,
    FIND_PLATE = 79 + servo_zero,
    CENTER_PICK = 265 + servo_zero,
    // 跟颜色有关
    BLUE_PICK = 203 + servo_zero,
    GREEN_PICK = 236 + servo_zero,
    RED_PICK = 267 + servo_zero,
    WHITE_PICK = 300 + servo_zero,
    BLACK_PICK = 332 + servo_zero,
    //跟任务二放置有关
    THIRD_PLACE= 267 + servo_zero,
    SECOND_PLACE=236 + servo_zero,
    ONCE_PLACE= 203 + servo_zero,
} GimbalArm_Servoangle_t;

/*
 * @brief 物块颜色枚举常量
 */
typedef enum
{
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_BLACK,
    COLOR_WHITE
} Color_t;

/*
 * @brief 物块ABC枚举常量
 */
typedef enum
{
    none=-1,
    Champion=0,
    Runner_up,
    Bronze_medal
} Ranking_t;

/*
 * @brief 料盘槽结构体变量
 */
typedef struct
{
    Color_t _color;
    Ranking_t _rank;
    int _angle;
    int _number;
} ThingStore_t;

/*
 * @brief 上层机构状态机
 */
typedef enum
{
    PICKINGIN,
    GETCOLORIN,
    PUTINGIN,
    PICKINGOUT,
    PUTTINGOUT,
    PICKINGABC,
    GETABCIN,
    PUTINGINABC,
    IDLE
} UpperTaskFlag;

typedef enum
{
    down_location = 0,
    middle_location = 1,
    up_location = 2,
    pick_middle_location = 3,
    down_put_lcoation = 4,
} upper_location;

// 打算servo数组一共3个舵机，第一个舵机控制前云台，第二个控制升降，第三个控制转盘

// 用全局数组存储记忆内容，功能函数全部使用指针操作

// 确定有舵机数组全局数组，Servo_t servos;
// 确定有料盘槽数组全局数组，ThingStore_t g_plate_things;
// 本局任务的颜色顺序全局数组 Color_t color_task[6]

// 确定当前颜色传感器颜色全局变量，Color_t g_current_color_ptr;
// 确定当前状态机标志全局变量，UpperTaskFlag g_upperflag;
// 确定本局任务的颜色对应数字全局变量，int color_task_index;
// 确定有当前颜色识别轮数全局变量, int CurrentColorLoop;
// 确定当前放置好的物块任务轮数，int PutGoalLoop;

void GetColorTask(Color_t *color_task, int *color_task_index);
void DistributionLoop(Servo_t *servos, ThingStore_t *plate_things, Color_t *current_color_ptr, UpperTaskFlag *upperflag, int *CurrentColorLoop);
void PutGoal(Color_t *color_task, Servo_t *servos, ThingStore_t *plate_things, UpperTaskFlag *upperflag, int *PutGoalLoop);
void upper_move_distance(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void upper_to_target(upper_location target_position);
void Get_ABC_Task( ThingStore_t *plate_things, int *ranking_task_index);
void Distribution_ABC(Servo_t *servos, ThingStore_t *plate_things, UpperTaskFlag *upperflag, int *CurrentrankingLoop);
void PutABCGoal(Ranking_t *ranking_task, Servo_t *servos, ThingStore_t *plate_things, UpperTaskFlag *upperflag, int *PutGOAL_RANKIONGLOOP);

#endif