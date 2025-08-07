#ifndef _UPPER_H_
#define _UPPER_H_
/*
    * @file upper.h 该工程包括了上层机构的逻辑和接口函数实现
*/
#include "main.h"
#include "servo.h"
//所有舵机的常量或枚举常量
#define THING_GIMBAL_FIXED_DELTA
#define ASS_SERVO_OPEN
#define ASS_SERVO_CLOSE
/*
    * @brief 抬升舵机枚举常量
*/
typedef enum
{
    PICK_DOWN,
    UP,
    PUT_DOWN
} Lift_Servoangle_t;
/*
    * @brief 前云台枚举常量
*/
typedef enum
{
    PICK_LEFT,
    FIND_PLATE,
    GOAL
} GimbalArm_Servoangle_t;
/*
    * @brief 物块颜色枚举常量
*/
typedef enum
{
    RED,
    GREEN,
    BLUE,
    BLACK,
    WHITE
} Color_t;

/*
    * @brief 料盘槽结构体变量
*/
typedef struct
{
    int _color;
    int _angle;
    int _number;
}ThingStore_t;
// 用全局数组存储记忆内容，功能函数全部尽量使用指针操作
void DistrubutionLoop(ThingStore_t* plate_things,Color_t* current_color_ptr);
void ThingGimbalControl(ThingStore_t* plate_things,GimbalArm_Servoangle_t* gimbal_angle);
int PickStoreLoop();
int PickPutLoop();

#endif