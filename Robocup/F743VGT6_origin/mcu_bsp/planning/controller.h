#ifndef __CONTROLLER_H
#define __CONTROLLER_H


//#include "Motor.h"

#include "Lib_pormise.h"
#include "Kinematic.h"
#include "pid.h"
#include "motor_def.h"
#include "ZDTstepmotor.h"

typedef enum {
    LOCATION_CONTROL,     // 位置闭环
    SPEED_CONTROL_SELF,   // 自身坐标系速度开环
    SPEED_CONTROL_GROUND  // 大地坐标系速度开环
} ControlMode_t;

typedef struct _{  
	StepMotorZDT_t *zdt_mot;
Motor_Controller_struct **MotorList;
    float target_speed[4];
    float current_speed[4];
    ControlMode_t ControlMode;
    SimpleStatus_t status;
    pid_t pid_x;
    pid_t pid_y;
    pid_t pid_yaw;
    Kinematic_t* kinematic;
	void (*setmotor_speed) (StepMotorZDT_t*, float *);
	void (*Controller_MotorUpdate)(struct _*, uint16_t);
} Controller_t;

//初始化函数

void Controller_Init(Controller_t *controller, Motor_Controller_struct **MotorList, Kinematic_t *kinematic);
void Controller_setMotorTargetSpeed(StepMotorZDT_t *_stepzdt,float *target_vel);

//
void Controller_KinematicAndControlUpdate(Controller_t* controller, uint16_t dt);
void Controller_KinematicAndControlUpdateWithYaw(Controller_t* controller, uint16_t dt, float yaw);
void Controller_StatusUpdate(Controller_t* controller, odom_t* odom_in);
void Controller_set_vel_target(Controller_t* controller, cmd_vel_t cmd_vel_in, bool use_ground_control);
void Controller_control_update(Controller_t* controller, odom_t* odom_in);
SimpleStatus_t* Controller_SetClosePosition(Controller_t* controller, const odom_t* target_odom, const odom_t* target_error, bool clearodom);

#endif