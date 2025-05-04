#ifndef __PLANNER_H
#define __PLANNER_H

#include "Lib_Math.h"
#include "Lib_pormise.h"
#include "controller.h"

typedef enum {
    PLANNER_MODE_OPEN_CONTROL,
    PLANNER_MODE_CLOSE_CONTROL
} PlannerMode;

typedef struct {
    float target_t;
    uint16_t current_t;
    odom_t start_odom;
    odom_t target_odom;
    CubicSpline spline[3];
    Controller* controller;
    SimpleStatus promise;
    PlannerMode control_mode;
} Planner;

void Planner_init(Planner* self, Controller* controller);
SimpleStatus* Planner_LoactaionOpenControl(Planner* self, const odom_t* target_odom, float max_v, const cmd_vel_t* target_vel, bool clearodom);
SimpleStatus* Planner_LoactaionCloseControl(Planner* self, const odom_t* target_odom, float max_v, const odom_t* target_error, bool clearodom);
void Planner_update(Planner* self, uint16_t dt);

#endif