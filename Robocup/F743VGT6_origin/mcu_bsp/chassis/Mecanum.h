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
} class_t;

typedef struct {
    float a; // 轮子到车身中心的距离
    float b; // 轮子到车身中心的距离
    class_t diclass; // 车轮布局类型
    odom_t current_odom; // 当前里程计数据
    odom_t target_odom;
    cmd_vel_t current_vel;
    cmd_vel_t target_val;
    odom_t _odom_error;
} Mecanum_t;

// 函数声明
void inv(const cmd_vel_t *cmd_vel_in, float *speed_control, const Mecanum_t *mecanum);
void inv_global(const cmd_vel_t *cmd_vel_in, float *speed_control, const odom_t *odom_in, const Mecanum_t *mecanum);
void forward(const float *current_speed, cmd_vel_t *cmd_vel_in, const Mecanum_t *mecanum);
void CalculationUpdate(uint16_t dt, const cmd_vel_t *cmd_vel_in, odom_t *odom_in);
void CalculationUpdateWithYaw(uint16_t dt, const cmd_vel_t *cmd_vel_in, odom_t *odom_in, float yaw);
void ClearOdometry(Mecanum_t *mecanum);
void update_odom(const odom_t *odom_in, Mecanum_t *mecanum);

#endif