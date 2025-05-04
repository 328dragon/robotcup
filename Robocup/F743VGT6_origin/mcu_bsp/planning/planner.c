#include "planner.h"

void Planner_init(Planner* self, Controller* controller) {
    self->target_t = 0;
    self->current_t = 0;
    self->target_odom = (odom_t){0, 0, 0};
    self->spline[0] = CubicSpline_init();
    self->spline[1] = CubicSpline_init();
    self->spline[2] = CubicSpline_init();
    self->controller = controller;
    self->promise = SimpleStatus_init();
    self->control_mode = PLANNER_MODE_OPEN_CONTROL;
}

SimpleStatus* Planner_LoactaionOpenControl(Planner* self, const odom_t* target_odom, float max_v, const cmd_vel_t* target_vel, bool clearodom) {
    SimpleStatus_start(&self->promise);
    
    float targetx, targety, targetyaw;

    if (!clearodom) {
        targetx = target_odom->x - self->controller->kinematic->current_odom.x;
        targety = target_odom->y - self->controller->kinematic->current_odom.y;
        targetyaw = target_odom->yaw - self->controller->kinematic->current_odom.yaw;
    } else {
        targetx = target_odom->x;
        targety = target_odom->y;
        targetyaw = target_odom->yaw;
        Controller_kinematic_ClearOdometry(self->controller->kinematic);
    }

    float target_t = Sqrt(targetx * targetx + targety * targety) / (max_v * 0.5);
    
    self->spline[0] = CubicSpline_create((float[]){0, 0}, (float[]){target_t, targetx}, (float[]){0, target_vel->linear_x});
    self->spline[1] = CubicSpline_create((float[]){0, 0}, (float[]){target_t, targety}, (float[]){0, target_vel->linear_y});
    self->spline[2] = CubicSpline_create((float[]){0, 0}, (float[]){target_t, targetyaw}, (float[]){0, target_vel->angular_z});
    
    self->target_t = target_t;
    self->current_t = 0;
    self->control_mode = PLANNER_MODE_OPEN_CONTROL;
    
    return &self->promise;
}

SimpleStatus* Planner_LoactaionCloseControl(Planner* self, const odom_t* target_odom, float max_v, const odom_t* target_error, bool clearodom) {
    Planner_LoactaionOpenControl(self, target_odom, max_v, &(cmd_vel_t){0, 0, 0}, clearodom);
    
    self->controller->kinematic->_odom_error = *target_error;
    self->control_mode = PLANNER_MODE_CLOSE_CONTROL;
    self->target_odom = *target_odom;
    self->start_odom = self->controller->kinematic->current_odom;
    
    return &self->promise;
}

void Planner_update(Planner* self, uint16_t dt) {
    self->current_t += dt;
    float t = (float)self->current_t / 1000.0;
    
    if (!SimpleStatus_isResolved(&self->promise)) {
        switch (self->control_mode) {
            case PLANNER_MODE_OPEN_CONTROL: {
                cmd_vel_t vel_target = {
                    CubicSpline_dx(&self->spline[0], t),
                    CubicSpline_dx(&self->spline[1], t),
                    CubicSpline_dx(&self->spline[2], t)
                };
                Controller_set_vel_target(self->controller, &vel_target, true);
                
                if (t >= self->target_t) {
                    SimpleStatus_resolve(&self->promise);
                }
                break;
            }
            case PLANNER_MODE_CLOSE_CONTROL: {
                t = t > self->target_t ? self->target_t : t;
                
                odom_t position = {
                    CubicSpline_evaluate(&self->spline[0], t) + self->start_odom.x,
                    CubicSpline_evaluate(&self->spline[1], t) + self->start_odom.y,
                    CubicSpline_evaluate(&self->spline[2], t) + self->start_odom.yaw
                };
                
                Controller_SetClosePosition(self->controller, &position, &self->controller->kinematic->_odom_error, false);
                
                odom_t* error = &self->controller->kinematic->_odom_error;
                odom_t* current = &self->controller->kinematic->current_odom;
                
                if (fabs(self->target_odom.x - current->x) < error->x &&
                    fabs(self->target_odom.y - current->y) < error->y &&
                    fabs(self->target_odom.yaw - current->yaw) < error->yaw) {
                    SimpleStatus_resolve(&self->promise);
                }
                break;
            }
            default:
                break;
        }
    }
}