#include "Kinematic.h"
#include <math.h>
void Kinematic_init(Kinematic_t *_Kinematic,float _a,float _b,chassis_t _disclass)
{
_Kinematic->a = _a;
_Kinematic->b = _b;
_Kinematic->diclass = _disclass;
_Kinematic->current_odom.x = 0;
_Kinematic->current_odom.y = 0;
_Kinematic->current_odom.yaw = 0;
}


/**
 * @brief 根据自身坐标系解算运动学逆解(只算一次)
 * @param cmd_vel_in 输入速度指令
 * @param speed_control 输出速度控制数组
 * @param _Kinematic 麦轮结构体指针
 */
void Kinematic_inv( cmd_vel_t *cmd_vel_in, float *speed_control,  Kinematic_t *_Kinematic) {
    float a = _Kinematic->a;
    float b = _Kinematic->b;
    
    switch (_Kinematic->diclass) {
        // o型
        case O_shape:
            speed_control[0] = cmd_vel_in->linear_y - cmd_vel_in->linear_x + cmd_vel_in->angular_z * (a + b);
            speed_control[1] = cmd_vel_in->linear_y + cmd_vel_in->linear_x - cmd_vel_in->angular_z * (a + b);
            speed_control[2] = cmd_vel_in->linear_y - cmd_vel_in->linear_x - cmd_vel_in->angular_z * (a + b);
            speed_control[3] = cmd_vel_in->linear_y + cmd_vel_in->linear_x + cmd_vel_in->angular_z * (a + b);
            break;

        // x型
        case X_shape:
            // 左上 (0) 轮：受 linear_x、linear_y、angular_z 的影响
            speed_control[0] = cmd_vel_in->linear_x - cmd_vel_in->linear_y - cmd_vel_in->angular_z * (a + b);

            // 右上 (1) 轮：受 linear_x、linear_y、angular_z 的影响
            speed_control[1] = cmd_vel_in->linear_x + cmd_vel_in->linear_y + cmd_vel_in->angular_z * (a + b);

            // 左下 (2) 轮：受 linear_x、linear_y、angular_z 的影响
            speed_control[2] = cmd_vel_in->linear_x + cmd_vel_in->linear_y - cmd_vel_in->angular_z * (a + b);

            // 右下 (3) 轮：受 linear_x、linear_y、angular_z 的影响
            speed_control[3] = cmd_vel_in->linear_x - cmd_vel_in->linear_y + cmd_vel_in->angular_z * (a + b);
            break;

        default:
            break;
    }
}

/**
 * @brief 根据大地坐标系解算运动学逆解(需要根据当前姿态实时计算)
 * @param cmd_vel_in 输入速度指令
 * @param speed_control 输出速度控制数组
 * @param odom_in 里程计信息指针
 * @param _Kinematic 麦轮结构体指针
 */
void  Kinematic_inv_global( cmd_vel_t *cmd_vel_in, float *speed_control,  odom_t *odom_in,  Kinematic_t *_Kinematic) {
    float yaw = odom_in->yaw;
    cmd_vel_t cmd_vel_body;
    
    // 将大地坐标系下的目标速度转换为车身坐标系下的目标速度
    float target_vx = cmd_vel_in->linear_x;
    float target_vy = cmd_vel_in->linear_y;
    
    cmd_vel_body.linear_x = target_vx * cos(yaw) + target_vy * sin(yaw);
    cmd_vel_body.linear_y = -target_vx * sin(yaw) + target_vy * cos(yaw);
    cmd_vel_body.angular_z = cmd_vel_in->angular_z;
    
    Kinematic_inv(&cmd_vel_body, speed_control, _Kinematic);
}

/**
 * @brief 从当前车身的速度推算底盘的速度
 * @param current_speed 当前速度数组
 * @param cmd_vel_in 输出速度指令
 * @param _Kinematic 麦轮结构体指针
 */
void Kinematic_forward( float *current_speed, cmd_vel_t *cmd_vel_in,  Kinematic_t *_Kinematic) {
    float a = _Kinematic->a;
    float b = _Kinematic->b;
    float v0 = current_speed[0]; // 左上轮子速度
    float v1 = current_speed[1]; // 右上轮子速度
    float v2 = current_speed[2]; // 左下轮子速度
    float v3 = current_speed[3]; // 右下轮子速度                                
	
    // 修正后的正解算公式
    cmd_vel_in->linear_x = (v0 + v1 + v2 + v3) / 4.0;               // X轴方向速度
    cmd_vel_in->linear_y = (-v0 + v1 + v2 - v3) / 4.0;              // Y轴方向速度
    cmd_vel_in->angular_z = (-v0 + v1 - v2 + v3) / (4.0 * (a + b)); // 角速度
}

/**
 * @brief 里程计更新函数，需要传递进dt(单位为ms)
 * @param dt 传入的时间间隔(毫秒)
 * @param cmd_vel_in 传入当前的速度指针
 * @param odom_in 传入被更新里程计指针
 */
void Kinematic_CalculationUpdate(uint16_t dt,  cmd_vel_t *cmd_vel_in, odom_t *odom_in) {
    float delta_t = (float)dt / 1000;
    float dyaw = cmd_vel_in->angular_z * delta_t;
    float dx = cmd_vel_in->linear_x * delta_t;
    float dy = cmd_vel_in->linear_y * delta_t;
    
    odom_in->yaw += dyaw;
    odom_in->y += dx * sinf(odom_in->yaw) + dy * cosf(odom_in->yaw);
    odom_in->x += dx * cosf(odom_in->yaw) - dy * sinf(odom_in->yaw);
}

/**
 * @brief 带yaw的里程计更新函数
 * @param dt 传入的时间间隔(毫秒)
 * @param cmd_vel_in 传入当前的速度指针
 * @param odom_in 传入被更新里程计指针
 * @param yaw 传入的yaw角度
 */
void Kinematic_CalculationUpdateWithYaw(uint16_t dt,  cmd_vel_t *cmd_vel_in, odom_t *odom_in, float yaw) {
    float delta_t = (float)dt / 1000;
    float dyaw = cmd_vel_in->angular_z * delta_t;
    float dx = cmd_vel_in->linear_x * delta_t;
    float dy = cmd_vel_in->linear_y * delta_t;
    
    odom_in->yaw = yaw;
    odom_in->y += dx * sinf(odom_in->yaw) + dy * cosf(odom_in->yaw);
    odom_in->x += dx * cosf(odom_in->yaw) - dy * sinf(odom_in->yaw);
}

/**
 * @brief 清除里程计数据
 * @param _Kinematic 麦轮结构体指针
 */
void Kinematic_ClearOdometry(Kinematic_t *_Kinematic) {
    _Kinematic->current_odom.x = 0;
    _Kinematic->current_odom.y = 0;
    _Kinematic->current_odom.yaw = 0;
}

/**
 * @brief 更新里程计
 * @param odom_in 传入的里程计指针
 * @param _Kinematic 麦轮结构体指针
 */
void Kinematic_update_odom( odom_t *odom_in, Kinematic_t *_Kinematic) {
    _Kinematic->current_odom = *odom_in;
}