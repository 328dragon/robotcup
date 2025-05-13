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
    CubicSpline_t cub_spline[3];
    Controller_t* controller;
    SimpleStatus_t promise;
    PlannerMode control_mode;
} Planner_t;

void Planner_init(Planner_t* self, Controller_t* controller);
SimpleStatus_t* Planner_LoactaionOpenControl(Planner_t* self, const odom_t* target_odom, float max_v, const cmd_vel_t* target_vel, bool clearodom);
SimpleStatus_t* Planner_LoactaionCloseControl(Planner_t* self, const odom_t* target_odom, float max_v, const odom_t* target_error, bool clearodom);
void Planner_update(Planner_t* self, uint16_t dt);

#endif