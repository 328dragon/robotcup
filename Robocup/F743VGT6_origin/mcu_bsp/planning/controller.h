#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "Lib_pormise.h"
#include "Motor.h"
#include "Kinematic.h"
#include "Lib_List.h"

typedef enum {
    LOCATION_CONTROL,     // 位置闭环
    SPEED_CONTROL_SELF,   // 自身坐标系速度开环
    SPEED_CONTROL_GROUND  // 大地坐标系速度开环
} ControlMode_t;

typedef struct {
    IMotorSpeed_t** MotorList;
    float target_speed[4];
    float current_speed[4];
    ControlMode_t ControlMode;
    SimpleStatus_t status;
    pid_Increment_template_t pid_x;
    pid_Increment_template_t pid_y;
    pid_Increment_template_t pid_yaw;
    Kinematic_t* kinematic;
} Controller_t;

// 构造函数
void Controller_Init(Controller_t* controller, IMotorSpeed_t** MotorList, Kinematic_t* kinematic);

// 成员函数
void Controller_KinematicAndControlUpdate(Controller_t* controller, uint16_t dt);
void Controller_KinematicAndControlUpdateWithYaw(Controller_t* controller, uint16_t dt, float yaw);
void Controller_setMotorTargetSpeed(Controller_t* controller, float* target_speed);
void Controller_MotorUpdate(Controller_t* controller, uint16_t dt);
void Controller_StatusUpdate(Controller_t* controller, odom_t* odom_in);
void Controller_set_vel_target(Controller_t* controller, cmd_vel_t cmd_vel_in, bool use_ground_control);
void Controller_control_update(Controller_t* controller, odom_t* odom_in);
SimpleStatus_t* Controller_SetClosePosition(Controller_t* controller, const odom_t* target_odom, const odom_t* target_error, bool clearodom);

#endif