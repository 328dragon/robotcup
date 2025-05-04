#include "mainwork.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ZDTstepmotor.h"
#include "Mecanum.h"
float DEBUG = 0.0f;
float DEBUG2 = 0.0f;
float DEBUG3 = 0.0f;
// 实例化
Mecanum_t *mecanum_ptr;           // 麦轮实例
StepMotorZDT_t *stepmotor_ptr[4]; // 步进电机实例

TaskHandle_t main_cpp_handle;       // 主函数
TaskHandle_t Planner_update_handle; // 轨迹规划
TaskHandle_t Chassic_control_handle;
void OnChassicControl(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);

void main_work(void)
{

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

void OnChassicControl(void *pvParameters)
{
    uint16_t last_tick = xTaskGetTickCount();
    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();

        vTaskDelay(10);
    }
}

void Onmaincpp(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(1000);
    }
}

void OnPlannerUpdate(void *pvParameters)
{
    uint16_t last_tick = xTaskGetTickCount();
    // Kinematic.init(0.6, 2, 0.2); // 初始化运动学模型
    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
//        planner_ptr->update(dt);
        vTaskDelay(50);
    }
}