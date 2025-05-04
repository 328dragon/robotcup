#include "controller.h"

void Controller_Init(Controller_t* controller, IMotorSpeed_t** MotorList, Kinematic_t* kinematic) {
    controller->MotorList = MotorList;
    controller->kinematic = kinematic;
    controller->ControlMode = SPEED_CONTROL_SELF;
    
    // 初始化PID控制器
    pid_Increment_template_t_init(&controller->pid_x, 0.3f, 1.0f, 0.2f, -0.4f, 0.4f);
    pid_Increment_template_t_init(&controller->pid_y, 0.3f, 1.0f, 0.2f, -0.4f, 0.4f);
    pid_Increment_template_t_init(&controller->pid_yaw, 0.6f, 2.0f, 0.2f, -0.6f, 0.6f);
    
    // 初始化状态
    SimpleStatus_t_init(&controller->status);
    
    // 初始化速度数组
    for (int i = 0; i < 4; i++) {
        controller->target_speed[i] = 0.0f;
        controller->current_speed[i] = 0.0f;
    }
}

void Controller_setMotorTargetSpeed(Controller_t* controller, float* target_speed) {
    for (int i = 0; i < 4; i++) {
        if (controller->MotorList[i] != NULL) {
            IMotorSpeed_t_set_speed_target(controller->MotorList[i], target_speed[i]);
        }
    }
}

void Controller_MotorUpdate(Controller_t* controller, uint16_t dt) {
    for (int i = 0; i < 4; i++) {
        if (controller->MotorList[i] != NULL) {
            IMotorSpeed_t_update(controller->MotorList[i], &dt);
            controller->current_speed[i] = IMotorSpeed_t_get_linear_speed(controller->MotorList[i]);
        }
    }
}

void Controller_StatusUpdate(Controller_t* controller, odom_t* odom_in) {
    if (controller->ControlMode == LOCATION_CONTROL) {
        if (!SimpleStatus_t_isResolved(&controller->status)) {
            odom_t* target_odom = &controller->kinematic->target_odom;
            odom_t* odom_error = &controller->kinematic->_odom_error;
            
            if (fabs(odom_in->x - target_odom->x) < odom_error->x &&
                fabs(odom_in->y - target_odom->y) < odom_error->y &&
                fabs(odom_in->yaw - target_odom->yaw) < odom_error->yaw) {
                SimpleStatus_t_resolve(&controller->status);
            }
        }
    }
}

void Controller_set_vel_target(Controller_t* controller, cmd_vel_t cmd_vel_in, bool use_ground_control) {
    controller->kinematic->target_val = cmd_vel_in;
    
    if (use_ground_control) {
        controller->ControlMode = SPEED_CONTROL_GROUND;
    } else {
        controller->ControlMode = SPEED_CONTROL_SELF;
        Kinematic_t_inv(controller->kinematic, controller->kinematic->target_val, controller->target_speed);
    }
}

void Controller_control_update(Controller_t* controller, odom_t* odom_in) {
    switch (controller->ControlMode) {
        case LOCATION_CONTROL: {
            if (SimpleStatus_t_isResolved(&controller->status)) {
                controller->ControlMode = SPEED_CONTROL_SELF;
                cmd_vel_t zero_vel = {0, 0, 0};
                Kinematic_t_inv(controller->kinematic, zero_vel, controller->target_speed);
                return;
            }
            
            odom_t* target_odom = &controller->kinematic->target_odom;
            float vx = pid_Increment_template_t_cal(&controller->pid_x, target_odom->x, odom_in->x) + controller->kinematic->target_val.linear_x;
            float vy = pid_Increment_template_t_cal(&controller->pid_y, target_odom->y, odom_in->y) + controller->kinematic->target_val.linear_y;
            float v_yaw = pid_Increment_template_t_cal(&controller->pid_yaw, target_odom->yaw, odom_in->yaw) + controller->kinematic->target_val.angular_z;
            
            cmd_vel_t vel = {vx, vy, v_yaw};
            Kinematic_t_inv(controller->kinematic, vel, controller->target_speed, odom_in);
            break;
        }
        
        case SPEED_CONTROL_SELF: {
            // 自身坐标下的速度控制什么都不用做
            break;
        }
        
        case SPEED_CONTROL_GROUND: {
            Kinematic_t_inv(controller->kinematic, controller->kinematic->target_val, controller->target_speed, odom_in);
            break;
        }
    }
}

SimpleStatus_t* Controller_SetClosePosition(Controller_t* controller, const odom_t* target_odom, const odom_t* target_error, bool clearodom) {
    controller->kinematic->target_odom = *target_odom;
    controller->kinematic->_odom_error = *target_error;
    
    if (clearodom) {
        Kinematic_t_ClearOdometry(controller->kinematic);
    }
    
    controller->ControlMode = LOCATION_CONTROL;
    SimpleStatus_t_start(&controller->status);
    return &controller->status;
}

void Controller_KinematicAndControlUpdate(Controller_t* controller, uint16_t dt) {
    Kinematic_t_forward(controller->kinematic, &controller->kinematic->current_vel, controller->current_speed);
    Kinematic_t_CalculationUpdate(controller->kinematic, dt, &controller->kinematic->current_vel, &controller->kinematic->current_odom);
    Controller_control_update(controller, &controller->kinematic->current_odom);
    Controller_StatusUpdate(controller, &controller->kinematic->current_odom);
    Controller_setMotorTargetSpeed(controller, controller->target_speed);
}

void Controller_KinematicAndControlUpdateWithYaw(Controller_t* controller, uint16_t dt, float yaw) {
    Kinematic_t_forward(controller->kinematic, &controller->kinematic->current_vel, controller->current_speed);
    Kinematic_t_CalculationUpdateWithYaw(controller->kinematic, dt, &controller->kinematic->current_vel, &controller->kinematic->current_odom, yaw);
    Controller_control_update(controller, &controller->kinematic->current_odom);
    Controller_StatusUpdate(controller, &controller->kinematic->current_odom);
    Controller_setMotorTargetSpeed(controller, controller->target_speed);
}