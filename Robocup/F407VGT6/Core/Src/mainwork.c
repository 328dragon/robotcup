// ​​Planner模块​​：负责轨迹规划，提供开环和闭环两种控制模式
// ​​Controller模块​​：负责控制算法执行和电机控制
// ​​Kinematic模块​​：负责运动学正逆解计算和里程计更新
// ​​FreeRTOS任务​​：提供实时调度框架

#include "mainwork.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ZDTstepmotor.h"
#include "Kinematic.h"
#include "controller.h"
#include "planner.h"
#include "usart.h"
#include "BMI088driver.h"
#include "lcd.h"
#include "lcd_init.h"
#include "pic.h"
float DEBUG = 0.0f;
float DEBUG2 = 0.0f;
float DEBUG3 = 0.0f;
odom_t deubg_target_odom = {1, 1, 0};
cmd_vel_t debug_target_erro = {0.01, 0.01, 0.01};

// 实例化
static Controller_t ChassisControl_instance;
static Kinematic_t kinematic_instance;
static Planner_t planner_instance;
static StepMotorZDT_t zdt_stepmotor_instances[4]; // 静态实例

Controller_t *ChassisControl_ptr; // 控制器实例
Kinematic_t *kinematic_ptr;       // 麦轮实例
Planner_t *planner_ptr;           // 规划
StepMotorZDT_t *zdt_stepmotor_ptr[4] = {
    &zdt_stepmotor_instances[0],
    &zdt_stepmotor_instances[1],
    &zdt_stepmotor_instances[2],
    &zdt_stepmotor_instances[3]};

TaskHandle_t LCD_Show_handle;        // 显示
TaskHandle_t Chassic_control_handle; // 底盘控制
TaskHandle_t main_cpp_handle;        // 主函数
TaskHandle_t Planner_update_handle;  // 轨迹规划
TaskHandle_t IMU_read_handle;        // IMU读取
void OnChassicControl(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);
void IMU_Read_task(void *pvParameters);     void LCD_Show_task(void *pvParameters);

void main_work(void)
{
    while (BMI088_init())
    {
        ;
    }

    // 注意电机编号如下所示
    Step_ZDT_Init(zdt_stepmotor_ptr[0], 2, &huart3, 0, 0.06f, false);
    Step_ZDT_Init(zdt_stepmotor_ptr[1], 1, &huart3, 1, 0.06f, false);
    Step_ZDT_Init(zdt_stepmotor_ptr[2], 3, &huart3, 0, 0.06f, false);
    Step_ZDT_Init(zdt_stepmotor_ptr[3], 4, &huart3, 1, 0.06f, true);

    ChassisControl_ptr = &ChassisControl_instance;
    kinematic_ptr = &kinematic_instance;
    planner_ptr = &planner_instance;
    Kinematic_init(kinematic_ptr, 0.6, 2, X_shape);
    Controller_Init(ChassisControl_ptr, zdt_stepmotor_ptr, kinematic_ptr);
    Planner_init(planner_ptr, ChassisControl_ptr);

    BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 1000, NULL, 3, &Chassic_control_handle);
    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
    BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 600, NULL, 4, &Planner_update_handle);
    BaseType_t ok5 = xTaskCreate(IMU_Read_task, "IMU_Read_task", 400, NULL, 4, &IMU_read_handle);
    BaseType_t ok6 = xTaskCreate(LCD_Show_task, "LCD_Show_task", 600, NULL, 1, &LCD_Show_handle);
    if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS || ok5 != pdPASS)
    {
        // 任务创建失败，进入死循环
        while (1)
        {
            // uart_printf("create task failed\n");
        }
    }
}

void LCD_Show_task(void *pvParameters)
{
    // 屏幕
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
    while (1)
    {
        // 显示
        // 陀螺仪
        LCD_ShowFloatNum1(0, 20, gyro[0], 4, RED, WHITE, 16);
        LCD_ShowString(48, 20, ",", RED, WHITE, 16, 0);
        LCD_ShowFloatNum1(58, 20, gyro[1], 4, RED, WHITE, 16);
        LCD_ShowString(106, 40, ",", RED, WHITE, 16, 0);
        LCD_ShowFloatNum1(116, 20, gyro[2], 4, RED, WHITE, 16);
        // 加速度
        LCD_ShowFloatNum1(0, 40, accel[0], 4, RED, WHITE, 16);
        LCD_ShowString(48, 40, ",", RED, WHITE, 16, 0);
        LCD_ShowFloatNum1(58, 40, accel[1], 4, RED, WHITE, 16);
        LCD_ShowString(106, 40, ",", RED, WHITE, 16, 0);
        LCD_ShowFloatNum1(116, 40, accel[2], 4, RED, WHITE, 16);
        // 显示temp
        LCD_ShowFloatNum1(10, 60, temp, 4, RED, WHITE, 16);
        LCD_ShowString(52, 60, ",", RED, WHITE, 16, 0);
        LCD_ShowString(62, 60, "gyro", RED, WHITE, 16, 0);
        LCD_ShowString(100, 60, ",", RED, WHITE, 16, 0);
        LCD_ShowString(106, 60, "accel", RED, WHITE, 16, 0);
        vTaskDelay(100);
    }
}

void IMU_Read_task(void *pvParameters)
{
    while (1)
    {
        BMI088_read(gyro, accel, &temp);
        vTaskDelay(10);
    }
}

void Onmaincpp(void *pvParameters)
{

    SimpleStatus_t *status = Planner_LoactaionCloseControl(planner_ptr, &deubg_target_odom, 0.2f, &debug_target_erro, true);
    int target_pot = 1;
    int zero_flag = 0;
    while (1)
    {
        if (SimpleStatus_t_isResolved(status))
        {
            if (zero_flag)
            {
                odom_t zero_odom = {0, 0, 0};
                status = Planner_LoactaionCloseControl(planner_ptr, &zero_odom, 0.2f, &debug_target_erro, true);
                planner_ptr->controller->kinematic->target_odom = (odom_t){0, 0, 0};
            }
            else  if(zero_flag==0)
            {
                target_pot++;
                if (target_pot == 2)
                {
                    deubg_target_odom = (odom_t){-1, -1, 0};
                }
                if (target_pot == 3)
                {
                    deubg_target_odom = (odom_t){0, 0, 0};
                    zero_flag = 1;
                }

                status = Planner_LoactaionCloseControl(planner_ptr, &deubg_target_odom, 0.2f, &debug_target_erro, true);
            }
        }

        vTaskDelay(200);
    }
}

void OnPlannerUpdate(void *pvParameters)
{
    uint16_t last_tick = xTaskGetTickCount();
    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
        Planner_update(planner_ptr, dt); // 轨迹规划
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
        //
        Controller_KinematicAndControlUpdate(ChassisControl_ptr, dt);
        // 步进不需要速度环，此处仅为了读取电机速度
        ChassisControl_ptr->Controller_MotorUpdate(ChassisControl_ptr, dt);
        vTaskDelay(10);
    }
}
