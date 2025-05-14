#ifndef __KINEMATIC_H
#define __KINEMATIC_H
#include "stdint.h"
#include "math.h"
#define PI 3.1415926535

// 左上为0，右上为1，左下为2，右下为3
typedef struct {
    float linear_x;
    float linear_y;
    float angular_z;
} cmd_vel_t;

typedef struct {
    float x;
    float y;
    float yaw;
} odom_t;

typedef enum {
    O_shape,
    X_shape
} chassis_t;

typedef struct {
    float a; // 轮子到车身中心的距离
    float b; // 轮子到车身中心的距离
    chassis_t diclass; // 车轮布局类型
    odom_t current_odom; // 当前里程计数据
    odom_t target_odom;
    cmd_vel_t current_vel;
    cmd_vel_t target_val;
    odom_t _odom_error;
} Kinematic_t;

void Kinematic_init(Kinematic_t *_Kinematic, float _a, float _b, chassis_t _disclass);

// 函数声明
void Kinematic_inv( cmd_vel_t *cmd_vel_in, float *speed_control,  Kinematic_t *_Kinematic);
void  Kinematic_inv_global( cmd_vel_t *cmd_vel_in, float *speed_control,  odom_t *odom_in,  Kinematic_t *_Kinematic);
void Kinematic_forward( float *current_speed, cmd_vel_t *cmd_vel_in,  Kinematic_t *_Kinematic);
void Kinematic_CalculationUpdate(uint16_t dt,  cmd_vel_t *cmd_vel_in, odom_t *odom_in);
void Kinematic_CalculationUpdateWithYaw(uint16_t dt,  cmd_vel_t *cmd_vel_in, odom_t *odom_in, float yaw);
void Kinematic_ClearOdometry(Kinematic_t *_Kinematic);
void Kinematic_update_odom( odom_t *odom_in, Kinematic_t *_Kinematic);

#endif
