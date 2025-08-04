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
#include "string.h"
#include "bsp_usart.h"
#include "tim.h"
#include "tcs230.h"
#include "gray.h"
#define BUZZER_ON HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
#define BUZZER_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
// 灰度转弯值
__IO int turn_stop_flag = 0; // 转弯停止标志位
int turn_dir_all = -1;
__IO int turn_state = 0;
// 主函数状态机
int main_state = 0;
int motor_mode = 0;
// 颜色传感器 状态机
int goods_color = 90;
int read_cololr_flag = 0; // 颜色传感器读取标志位
int read_color_state = 0; // 颜色传感器读取状态
int temp_color = -1;
// 灰度
gray_state real_time_gray_state = orgin_gray; // 主灰度状态
// 前面灰度
int gray_count = 0;      // 前面灰度计数
int gray_count_last = 0; // 上次前面灰度计数
uint8_t digital_gray_data[8];
int sensor_weights[8] = {-7, -4, -3, -2, 2, 3, 4, 7}; // 传感器权重
unsigned char Digtal_gray;
unsigned char Anolog_gray[8] = {0};
unsigned char Normal[8] = {0};
//
int debug_pwm = 0;
int close_flag = 0;
int safe_flag = 0;
USARTInstance uart6 = {0};
void usart6_callback(void)
{
}
USART_Init_Config_s uart6_cfg = {
    .recv_buff_size = 90,
    .usart_handle = &huart6,
    .module_callback = usart6_callback,
};
//
float DEBUG = 0.0f;
float DEBUG2 = 0.0f;
float DEBUG3 = 0.0f;
int position_flag = 0;
int begin_flag = 1;
int safe_guard = 0;
cmd_vel_t debug_target_vel = {0.2, 0, 0};
odom_t debug_target_odom = {0, 0, 0};
odom_t debug_target_erro = {0.05, 0.05, 0.05};

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
TaskHandle_t tcs230_read_handle;     // tcs230颜色传感器读取
TaskHandle_t gray_read_handle;       // 灰度传感器
void OnChassicControl(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);
void IMU_Read_task(void *pvParameters);
void LCD_Show_task(void *pvParameters);
void tcs230_read_task(void *pvParameters);
void gray_read_task(void *pvParameters);

void main_work(void)
{
    HAL_UART_Receive_IT(&huart4, &RxData, 1);
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");

    while (BMI088_init())
    {
        ;
    }
    USARTRegister(&uart6, &uart6_cfg);
    memset(uart6.recv_buff, 0, uart6.recv_buff_size);
    // 注意电机编号如下所示

    Step_ZDT_Init(zdt_stepmotor_ptr[0], 1, &huart3, 0, 0.06f, false); // 左上
    Step_ZDT_Init(zdt_stepmotor_ptr[1], 2, &huart3, 1, 0.06f, false); // 右上
    Step_ZDT_Init(zdt_stepmotor_ptr[2], 4, &huart3, 0, 0.06f, false); // 左下
    Step_ZDT_Init(zdt_stepmotor_ptr[3], 3, &huart3, 1, 0.06f, true);  // 右下

    ChassisControl_ptr = &ChassisControl_instance;
    kinematic_ptr = &kinematic_instance;
    planner_ptr = &planner_instance;
    Kinematic_init(kinematic_ptr, 0.6, 2, X_shape);
    Controller_Init(ChassisControl_ptr, zdt_stepmotor_ptr, kinematic_ptr);
    Planner_init(planner_ptr, ChassisControl_ptr);

    BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 800, NULL, 3, &Chassic_control_handle);
    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
    BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 300, NULL, 4, &Planner_update_handle);
    BaseType_t ok5 = xTaskCreate(IMU_Read_task, "IMU_Read_task", 200, NULL, 4, &IMU_read_handle);
    BaseType_t ok6 = xTaskCreate(LCD_Show_task, "LCD_Show_task", 300, NULL, 1, &LCD_Show_handle);
    BaseType_t ok7 = xTaskCreate(tcs230_read_task, "tcs230_read_task", 100, NULL, 2, &tcs230_read_handle);
    BaseType_t ok8 = xTaskCreate(gray_read_task, "gray_read_task", 100, NULL, 2, &gray_read_handle);
    if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS || ok5 != pdPASS || ok7 != pdPASS)
    {
        // 任务创建失败，进入死循环
        while (1)
        {
            // uart_printf("create task failed\n");
        }
    }
}

void tcs230_read_task(void *pvParameters)
{

    while (1)
    {
        if (read_cololr_flag == 1)
        {

            switch (read_color_state)
            {
            case 0:
            {
                R = 0;
                G = 0;
                B = 0;
                temp_color = -1;
                printf("AT+COLOR\r\n");
                vTaskDelay(500);
                printf("AT+COLOR\r\n");
                read_color_state++;
                break;
            }
            case 1:
            {
                // 读取颜色传感器数据
                if (Get_RxFlag() && R != 0 && G != 0 && B != 0)
                {
                    temp_color = Get_Color();
                    if (temp_color != -1)
                    {
                        goods_color = temp_color;
                        read_color_state++;
                    }

                    else
                    {
                        // uart_printf("tcs230 read color failed\n");
                    }
                }
                printf("AT+COLOR\r\n");
                break;
            }
            case 2:
            {
                temp_color = -1;
                read_color_state = 0; // 重置状态机
                read_cololr_flag = 0;
                break;
            }
            default:
                break;
            }
        }
        // 读取颜色传感器数据
        vTaskDelay(100); // 延时200ms
    }
}
void gray_read_task(void *pvParameters)
{
    while (Ping())
    {
        vTaskDelay(5);
    }
    while (1)
    {
        gray_count++;
        // 读取灰度传感器数据
        Digtal_gray = IIC_Get_Digtal();

        for (int i = 0; i < 8; i++)
        {
            digital_gray_data[i] = 1 - ((Digtal_gray >> i) & 0x01); // 读取数 �灰度传感器数据
        }

        // 获取传感器模拟量结果
        if (IIC_Get_Anolog(Anolog_gray, 8))
        {
        }

        if (digital_gray_data[3] == 1 && digital_gray_data[4] == 1 && digital_gray_data[2] == 1 && digital_gray_data[5] == 1) // 中间4个
        {
            BUZZER_ON
            real_time_gray_state = all_black;
        }
        else
        {
            BUZZER_OFF
            real_time_gray_state = orgin_gray;
        }

        // 获取传感器归一化结果
        IIC_Anolog_Normalize(0xff); // 所有通道归一化都打开
        vTaskDelay(10);             // 设置完，需要等上一会。stm8的运算速度没stm32快，等一下，让传感器把数据刷新一下。
        if (IIC_Get_Anolog(Normal, 8))
        {
        }
        IIC_Anolog_Normalize(0xff); // 为了下一次循环是非归一化，所以清零

        vTaskDelay(10); // 延时100ms
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
        safe_flag++;
        if (safe_flag >= 20)
        {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, debug_pwm); // 抬升，500最低，800中间，1800最高
            if (close_flag == 1)
            {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 1200); // 夹爪，1800紧，1500松
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 1800); // 夹爪1100锁紧，1500松
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 1500); // 夹爪，1800紧，1500松
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 1500); // 夹爪1100锁紧，1500松
            }
        }

        if (turn_dir_all == -1)
        {
            turn_stop_flag = 0;
            turn_state = 0;
        }
        if ((turn_dir_all == 0) && (real_time_gray_state != all_black))
        {
            switch (turn_state)
            {

            case 0:
            {
                if (digital_gray_data[0] == 1)
                {

                    turn_state++;
                }
                break;
            }
            case 1:
            {
                if (digital_gray_data[1] == 1)

                {

                    turn_state++;
                }

                break;
            }
            case 2:
            {
                if (digital_gray_data[2] == 1)

                {
                    turn_stop_flag = 1;
                    turn_state = 0;
                }

                break;
            }

            default:
                break;
            }
        }
        else if (turn_dir_all == 1)
        {
        }
        vTaskDelay(10);
    }
}

static void move_to_next_line()
{
    motor_mode = 0;
    debug_target_vel = (cmd_vel_t){0.3, 0, 0};
    if (real_time_gray_state == all_black)
    {
        debug_target_vel = (cmd_vel_t){0, 0, 0};
        main_state++;
    }
}
static void turn_to_next_line()
{
    motor_mode = 0;
		turn_dir_all=0;
    debug_target_vel = (cmd_vel_t){0, 0, 0.05};
    if (turn_stop_flag == 1)
    {
        debug_target_vel = (cmd_vel_t){0, 0, 0};
        turn_dir_all = -1; // 重置转弯状态
        main_state++;
    }
}
void Onmaincpp(void *pvParameters)
{

    int safe_count = 0; // 保护锁
    while (1)
    {
        // 纯速度式验证没问题
        //      Controller_set_vel_target(ChassisControl_ptr, debug_target_vel, false);
        safe_count++;
        if (safe_count >= 3)
        {
            safe_guard = 1; // 保护锁打开
            switch (main_state)
            {
            case 0:
            {
                move_to_next_line();
                break;
            }
            case 1:
            {
                motor_mode = 1;
                debug_target_odom = (odom_t){0.1, 0, 0};
                debug_target_erro = (odom_t){0.01, 0.01, 0.01};
                position_flag++;
                Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 0.5, &debug_target_erro, 1);
                main_state++;
                break;
            }
            case 2:
            {

                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_to_next_line();
                }
                break;
            }
            case 3:
            {
                motor_mode = 1;
                debug_target_odom = (odom_t){0.1, 0, 0};
                debug_target_erro = (odom_t){0.01, 0.01, 0.01};
                position_flag++;
                Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 0.5, &debug_target_erro, 1);
                main_state++;
                break;
            }
            case 4:
            {

                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_to_next_line();
                }
                break;
            }
                        case 5:
            {
							                motor_mode = 1;
                debug_target_odom = (odom_t){0.15, 0, 0};
                debug_target_erro = (odom_t){0.01, 0.01, 0.01};
                position_flag++;
                Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 0.5, &debug_target_erro, 1);
                main_state++;
                break;
            }
            case 6:
            {

                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    turn_to_next_line();
                }
                break;
            }
            default:
                break;
            }
        }

        switch (motor_mode)
        {
        case 0:
        {
            Controller_set_vel_target(ChassisControl_ptr, debug_target_vel, false);
        }
        case 1:
        {
        }
        default:
            break;
        }

        vTaskDelay(100);
    }
}

// 轨迹规划更新任务
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
// 底盘更新任务,包括执行层
void OnChassicControl(void *pvParameters)
{
    uint16_t last_tick = xTaskGetTickCount();

    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
        if (safe_guard)
        {
            Controller_KinematicAndControlUpdate(ChassisControl_ptr, dt);
            // // 步进不需要速度环，此处仅为了读取电机速度
            ChassisControl_ptr->Controller_MotorUpdate(ChassisControl_ptr, dt);
        }
        else
        {
            float zero_speed[4] = {0, 0, 0, 0};
            ChassisControl_ptr->setmotor_speed(ChassisControl_ptr, zero_speed);
        }
        vTaskDelay(10);
    }
}
