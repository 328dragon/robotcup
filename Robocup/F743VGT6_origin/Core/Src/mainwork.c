#include "mainwork.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ZDTstepmotor.h"
#include "Kinematic.h"
#include "controller.h"
#include "planner.h"
#include "usart.h"
float DEBUG = 0.0f;
float DEBUG2 = 0.0f;
float DEBUG3 = 0.0f;
// 实例化
static Controller_t ChassisControl_instance;
static Kinematic_t kinematic_instance;
static Planner_t planner_instance;
static StepMotorZDT_t stepmotor_instances[4]; // 静态实例

Controller_t *ChassisControl_ptr; // 控制器实例
Kinematic_t *kinematic_ptr;       // 麦轮实例
Planner_t *planner_ptr;           // 规划
StepMotorZDT_t *stepmotor_ptr[4] = {
    &stepmotor_instances[0],
    &stepmotor_instances[1],
    &stepmotor_instances[2],
    &stepmotor_instances[3]};

TaskHandle_t Chassic_control_handle; // 底盘控制
TaskHandle_t main_cpp_handle;        // 主函数
TaskHandle_t Planner_update_handle;  // 轨迹规划

void OnChassicControl(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);

void main_work(void)
{
    Step_ZDT_Init(stepmotor_ptr[0], 1, &huart3, 1, 0.06f, false);
    Step_ZDT_Init(stepmotor_ptr[0], 2, &huart3, 1, 0.06f, false);
    Step_ZDT_Init(stepmotor_ptr[0], 3, &huart3, 1, 0.06f, false);
    Step_ZDT_Init(stepmotor_ptr[0], 4, &huart3, 1, 0.06f, true);

    ChassisControl_ptr = &ChassisControl_instance;
    kinematic_ptr = &kinematic_instance;
    planner_ptr = &planner_instance;
    Kinematic_init(kinematic_ptr, 0.6, 2, X_shape);
    Controller_Init(ChassisControl_ptr, stepmotor_ptr, kinematic_ptr);
    Planner_init(planner_ptr, ChassisControl_ptr);

    BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 600, NULL, 3, &Chassic_control_handle);
    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
    BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 1000, NULL, 4, &Planner_update_handle);
    if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS)
    {
        // 任务创建失败，进入死循环
        while (1)
        {
            // uart_printf("create task failed\n");
        }
    }
}

void Onmaincpp(void *pvParameters)
{
    while (1)
    {
        Controller_set_vel_target(ChassisControl_ptr, (cmd_vel_t){DEBUG, DEBUG2, DEBUG3}, true);
        vTaskDelay(500);
    }
}

void OnPlannerUpdate(void *pvParameters)
{
    uint16_t last_tick = xTaskGetTickCount();
    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
        Planner_update(planner_ptr, dt);
        vTaskDelay(50);
    }
}

void OnChassicControl(void *pvParameters)
{
    uint16_t last_tick = xTaskGetTickCount();
    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
        Controller_KinematicAndControlUpdate(ChassisControl_ptr, dt);
        // 步进不需要速度环，此处仅为了读取电机速度
        ChassisControl_ptr->Controller_MotorUpdate(ChassisControl_ptr, dt);
        vTaskDelay(10);
    }
}
