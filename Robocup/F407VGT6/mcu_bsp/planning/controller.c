#include "controller.h"
// 绝对式，速度只是一个参考，调用内部函数
void Controller_setMotor_Targetpos_vel(Controller_t *controller, float *target_pos, float *target_vel)
{
    for (int i = 0; i < 4; i++)
    {
        controller->zdt_mot[i]->motor_controller_t.set.velocity = target_vel[i];
        controller->zdt_mot[i]->motor_controller_t.set.deg_pos = target_pos[i];
    }
    set_speed_pos_target(controller->zdt_mot[0], target_vel[0], target_pos[0]);
    set_speed_pos_target(controller->zdt_mot[1], target_vel[1], target_pos[1]);
    set_speed_pos_target(controller->zdt_mot[2], target_vel[2], target_pos[2]);
    set_speed_pos_target(controller->zdt_mot[3], target_vel[3], target_pos[3]);
}

void Controller_setMotorTargetSpeed(Controller_t *controller, float *target_vel)
{
    for (int i = 0; i < 4; i++)
    {
        controller->zdt_mot[i]->motor_controller_t.set.velocity = target_vel[i];
    }
    set_speed_target(controller->zdt_mot[0], target_vel[0]);
    set_speed_target(controller->zdt_mot[1], target_vel[1]);
    set_speed_target(controller->zdt_mot[2], target_vel[2]);
    set_speed_target(controller->zdt_mot[3], target_vel[3]);
}

void ZDTController_MotorUpdate(Controller_t *controller, uint16_t dt)
{
    for (int i = 0; i < 4; i++)
    {
        // 更新状态
        Motor_Update_Getinfo(&controller->zdt_mot[i]->motor_controller_t, &dt);
        // 更新电机速度
        controller->current_speed[i] = get_linear_speed(controller->zdt_mot[i]);
    }
}

void Controller_Init(Controller_t *controller, StepMotorZDT_t **_zdt_motor, Kinematic_t *kinematic)
{
    for (int i = 0; i < 4; i++)
    {
        controller->zdt_mot[i] = _zdt_motor[i];
    }
    controller->kinematic = kinematic;
    controller->ControlMode = LOCATION_CONTROL;
    controller->setmotor_speed = Controller_setMotorTargetSpeed;
    controller->Controller_MotorUpdate = ZDTController_MotorUpdate;
    controller->setmotor_pos_vel = Controller_setMotor_Targetpos_vel;
    uint32_t init_x_maxout = 2;
    uint32_t init_y_maxout = 2;
    uint32_t init_yaw_maxout = 3;
    // 初始化PID控制器
    PID_struct_init(&controller->pid_x, POSITION_PID, init_x_maxout, 0.4f, 0.7f, 0.04f, 0);
    PID_struct_init(&controller->pid_y, POSITION_PID, init_y_maxout, 0.4f, 0.7f, 0.04f, 0);
    PID_struct_init(&controller->pid_yaw, POSITION_PID, init_yaw_maxout, 0.6f, 0.4f, 0.04f, 0);

    // 初始化状态
    SimpleStatus_t_init(&controller->status);

    // 初始化速度数组
    for (int i = 0; i < 4; i++)
    {
        controller->target_speed[i] = 0.0f;
        controller->current_speed[i] = 0.0f;
    }
}

void Controller_StatusUpdate(Controller_t *controller, odom_t *odom_in)
{
    if (controller->ControlMode == LOCATION_CONTROL)
    {
        if (!SimpleStatus_t_isResolved(&controller->status))
        {
            odom_t *target_odom = &controller->kinematic->target_odom;
            odom_t *odom_error = &controller->kinematic->_odom_error;

            if (fabs(odom_in->x - target_odom->x) < odom_error->x &&
                fabs(odom_in->y - target_odom->y) < odom_error->y &&
                fabs(odom_in->yaw - target_odom->yaw) < odom_error->yaw)
            {
                SimpleStatus_t_resolve(&controller->status);
            }
        }
    }
}

void Controller_set_vel_target(Controller_t *controller, cmd_vel_t cmd_vel_in, bool use_ground_control)
{
    controller->kinematic->target_val = cmd_vel_in;

    if (use_ground_control)
    {
        controller->ControlMode = SPEED_CONTROL_GROUND;
    }
    else
    {
        controller->ControlMode = SPEED_CONTROL_SELF;
        Kinematic_inv(&(controller->kinematic->target_val), controller->target_speed, controller->kinematic);
    }
}
void Controller_set_pos_vel_target(Controller_t *controller, odom_t target_pos_odom, cmd_vel_t cmd_vel_in, bool use_ground_control)
{
    controller->kinematic->target_odom = target_pos_odom;
    controller->kinematic->target_val = cmd_vel_in;

    if (use_ground_control)
    {
        controller->ControlMode = SPEED_CONTROL_GROUND;
    }
    else
    {
        controller->ControlMode = LOCATION_CONTROL;
        Kinematic_inv(&(controller->kinematic->target_val), controller->target_speed, controller->kinematic);
    }
}

void Controller_control_update(Controller_t *controller, odom_t *odom_in)
{
    switch (controller->ControlMode)
    {
    case LOCATION_CONTROL:
    {
        if (SimpleStatus_t_isResolved(&controller->status))
        {
            controller->ControlMode = SPEED_CONTROL_SELF;
            cmd_vel_t zero_vel = {0, 0, 0};
            Kinematic_inv(&zero_vel, controller->target_speed, controller->kinematic);
            return;
        }

        odom_t *target_odom = &controller->kinematic->target_odom;

//        float vx = pid_calc(&controller->pid_x, odom_in->x, target_odom->x);
//        float vy = pid_calc(&controller->pid_y, odom_in->y, target_odom->y);
//        float v_yaw = pid_calc(&controller->pid_yaw, odom_in->yaw, target_odom->yaw);
       	float vx = pid_calc(&controller->pid_x,odom_in->x, target_odom->x) + controller->kinematic->target_val.linear_x;
          float vy = pid_calc(&controller->pid_y, odom_in->y,target_odom->y) + controller->kinematic->target_val.linear_y;
         float v_yaw = pid_calc(&controller->pid_yaw, odom_in->yaw, target_odom->yaw) + controller->kinematic->target_val.angular_z;

        //  float vx = pid_calc(&controller->pid_x, target_odom->x, odom_in->x) + controller->kinematic->target_val.linear_x;
        //         float vy = pid_calc(&controller->pid_y, target_odom->y, odom_in->y) + controller->kinematic->target_val.linear_y;
        //          float v_yaw = pid_calc(&controller->pid_yaw, target_odom->yaw, odom_in->yaw) + controller->kinematic->target_val.angular_z;

        cmd_vel_t vel = {vx, vy, v_yaw};
        Kinematic_inv_global(&vel, controller->target_speed, odom_in, controller->kinematic); // 换算给四个电机具体速度
        break;
    }

    case SPEED_CONTROL_SELF:
    {
        // 自身坐标下的速度控制什么都不用做
        break;
    }

    case SPEED_CONTROL_GROUND:
    {
        Kinematic_inv(&controller->kinematic->target_val, controller->target_speed, controller->kinematic);
        break;
    }
    }
}

SimpleStatus_t *Controller_SetClosePosition(Controller_t *controller, odom_t *target_odom, odom_t *target_error, bool clearodom)
{
    // 设置闭环控制目标位置
    controller->kinematic->target_odom = *target_odom;
    controller->kinematic->_odom_error = *target_error;
    if (clearodom)
    {
        Kinematic_ClearOdometry(controller->kinematic);
    }
    controller->ControlMode = LOCATION_CONTROL;
    SimpleStatus_t_start(&controller->status);
    return &controller->status;
}

void Controller_KinematicAndControlUpdate(Controller_t *controller, uint16_t dt)
{
    Kinematic_forward(controller->current_speed, &controller->kinematic->current_vel, controller->kinematic);   // 从当前车身的速度current_speed[4]推算底盘的速度current_vel[3]{linear_x, linear_y, angular_z}
    Kinematic_CalculationUpdate(dt, &controller->kinematic->current_vel, &controller->kinematic->current_odom); // 里程计更新函数，需要传递进dt(单位为ms),由current_vel[3]乘以 dt得到当前的里程计数据current_odom[3]{x,y,yaw}
    // 核心控制函数
    Controller_control_update(controller, &controller->kinematic->current_odom); // 控制更新，传入当前里程计值，利用PID控制器（当前值current_odom,目标值target_odom）计算出速度指令，并通过Kinematic_inv_global算出具体电机速度
    Controller_StatusUpdate(controller, &controller->kinematic->current_odom);   // 状态更新（更新完成与否）,只判断，不做任何操作
    // 应用控制量
    controller->setmotor_speed(controller, controller->target_speed);
}

void Controller_KinematicAndControlUpdateWithYaw(Controller_t *controller, uint16_t dt, float yaw)
{
    Kinematic_forward(controller->current_speed, &controller->kinematic->current_vel, controller->kinematic);
    Kinematic_CalculationUpdateWithYaw(dt, &controller->kinematic->current_vel, &controller->kinematic->current_odom, yaw);
    Controller_control_update(controller, &controller->kinematic->current_odom);
    Controller_StatusUpdate(controller, &controller->kinematic->current_odom);
    // 这个函数随电机改变
    controller->setmotor_speed(controller, controller->target_speed);
}