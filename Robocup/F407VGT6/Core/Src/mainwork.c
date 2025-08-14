// ​​Planner模块​​：负责轨迹规划，提供开环和闭环两种控制模式
// ​​Controller模块​​：负责控制算法执行和电机控制
// ​​Kinematic模块​​：负责运动学正逆解计算和里程计更新
// ​​FreeRTOS任务​​：提供实时调度框架
#define DEBUG_TASK 0
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
#include "ch040.h"
#include "gw_color_iic.h"
#include "servo.h"
#include "upper.h"
#include "chassislogic.h"
#define BUZZER_ON HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
#define BUZZER_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
#define DEBUG_UPPER 0
#define DEBUG_CHASSIS 1
// #define DEBUG_PARAM_PUT 1
#define TASK2 1
// #define FINALTASK2 1

#define get_little_yellow_state HAL_GPIO_ReadPin(little_yellow_GPIO_Port, little_yellow_Pin)
#define abs(x) (x > 0 ? x : (-x))
float debug_angle[3] = {0, 0, 0}; // 调试角度
int debug_isOpened = 0;

int debug_speed = 0;
int debug_distance = 0; // 调试开始标志
float debug_chassis_speed[3] = {0};
float debug_chassis_distance[3] = {0};
static float Byte2Float(uint8_t *byte)
{
    float f;
    uint8_t *p = (uint8_t *)&f;
    p[0] = byte[0];
    p[1] = byte[1];
    p[2] = byte[2];
    p[3] = byte[3];
    return f;
}

Servo_t servo[3] = {
    {&htim9, TIM_CHANNEL_2, 0, 0},
    {&htim9, TIM_CHANNEL_1, 0, 0},
    {&htim5, TIM_CHANNEL_3, 0, 0}};
UpperTaskFlag upperflag = IDLE; // 上层机构状态机
UpperTaskFlag *upperflag_ptr = &upperflag;
ChassisTaskFlag chassisflag = IDLE_CHASSIS; // 底盘状态机
// ThingStore_t plate_things[5] = {0}; // 料盘槽数组
ThingStore_t plate_things[5] = {
    {COLOR_BLACK, 33, 0},
    {COLOR_WHITE, 93, 1},
    {COLOR_RED, 153, 2},
    {COLOR_BLUE, 213, 3},
    {COLOR_GREEN, 273, 4}};
Color_t color_task[5] = {COLOR_BLACK, COLOR_WHITE, COLOR_RED, COLOR_BLUE, COLOR_GREEN}; // 颜色任务数组
Color_t current_color_RGB = COLOR_BLACK;                                                // 当前颜色
Color_t current_color_HSL = COLOR_BLACK;                                                // 当前颜色
Color_t *current_color_ptr = &current_color_RGB;

ThingStore_Task2_t plate_task2_things[3] = {
    {GOLD, 33, 0},
    {SILVER, 93, 1},
    {BRONZE, 153, 2}
};
Rank_t rank_task[3] = {GOLD, SILVER, BRONZE}; // 排序任务数组
int CurrentColorLoop = 0;// 物块获取循环
int PutGoalLoop = 0;   // 目标放置循环
int CurrentRankLoop = 0;// 任务二物块获取循环
int PutRankLoop = 0;   // 任务二目标放置循环

float main_yaw = 0.0f; // imu存取的yaw
int safe_count = 0;    // 保护锁
// 主函数状态机
__IO int main_state = 0;
__IO int main_put_state = -1;
__IO int main_second_state = 0;
// 调试跑十字时候状态
//__IO int main_state = -1;
//__IO int main_put_state = 0;
// 测试气泵
//__IO int main_state = -2;
//__IO int main_put_state = -2;
int motor_mode = 0;
int qr_code = 0; // 香橙派获得的二维码数字（暂未使用）
int* qr_code_ptr = &qr_code;

int qr_mv_code = 0; // openmv获取的二维码数字（任务二）
int* qr_mv_code_ptr = &qr_mv_code;

int qr_mv_code2 = 0;// openmv获取的二维码数字(任务二)
int* qr_mv_code_ptr2 = &qr_mv_code2;

float find_circle_dx = 0; // 视觉传过来juli
float find_circle_dy = 0;
float final_circle_dx = 0; // 乘上系数后距离
float final_circle_dy = 0;
float final_Circle_px = 1;        // 纠正x系数
float final_Circle_py = 1;        // 纠正y系数
float pump_view_distance = 0.043; /// 吸盘到摄像头距离
// 颜色传感器
int GET_RGB_FLAG = 0;
int GET_HSL_FLAG = 0;
int goods_color_RGB = -1;
int goods_color_HSL = -1;
unsigned char RGB[3] = {0};
unsigned char HSL[3] = {0};
// 灰度
gray_state real_time_gray_state = orgin_gray;      // 主灰度状态
gray_state real_time_gray_state_side = orgin_gray; // 侧边灰度

// 侧边灰度
float gray_side_p = -0.0035;
float gray_data_side_middle = 0;
float gray_data_side_middle_temp = 0;
float gray_data_side_sum = 0;
float gray_data_side_sum_temp = 0;

uint8_t digital_gray_data_side[8];
int sensor_weights_side[8] = {-7, -4, -3, -2, 2, 3, 4, 7}; // 传感器权重
unsigned char Digtal_gray_side;
unsigned char Anolog_gray_side[8] = {0};
unsigned char Normal_side[8] = {0};
// 调试信息
int debug_pwm = 0;
int close_flag = 0;
int safe_flag = 0;

// 气泵
int pump_flag = 0;
int get_yellow_flag = 0;
int yellow_state = 0;
// 上升控制
int now_upper_loacation = 0;
int target_upper_loacation = 0;
float target_distance = 0;
float upper_target_vel = 0;
int upper_flag = 0;
// 串口接收
USARTInstance uart6 = {0};
USARTInstance uart3 = {0};
USARTInstance uart1 = {0};
USARTInstance uart2 = {0};
USARTInstance uart4 = {0};
// 读陀螺仪
void usart6_callback(void)
{
    if (uart6.recv_buff[0] == 0x5A && uart6.recv_buff[1] == 0xA5)
    {
        main_yaw = ch040_get_data(uart6.recv_buff);
    }
}

// 上位机通信，接收二维码
void usart1_callback(void)
{
    if (uart1.recv_buff[0] == 0x91 && uart1.recv_buff[1] == 0xCB)
    {

        qr_code = uart1.recv_buff[2];
    }
}

// 上位机 通信，接收纠正dx,dy
void usart2_callback(void)
{
    if (uart2.recv_buff[0] == 0x91 && uart2.recv_buff[1] == 0xCB)
    {
        // 四字节转浮点数
        find_circle_dx = Byte2Float(uart2.recv_buff + 2);
        find_circle_dy = Byte2Float(uart2.recv_buff + 6);
        // 限幅
        if (find_circle_dx > 0.1)
        {
            find_circle_dx = 0.1;
        }
        else if (find_circle_dx < -0.1)
        {
            find_circle_dx = -0.1;
        }
        if (find_circle_dy > 0.1)
        {
            find_circle_dy = 0.1;
        }
        else if (find_circle_dy < -0.1)
        {
            find_circle_dy = -0.1;
        }
        find_circle_dy += pump_view_distance;
        final_circle_dx = final_Circle_px * find_circle_dx;
        final_circle_dy = final_Circle_py * find_circle_dy;
    }
}

void usart4_callback(void)
{
    if (uart4.recv_buff[0] == 0x91 && uart4.recv_buff[1] == 0xCB)
    {
        qr_mv_code = uart4.recv_buff[2];
        qr_mv_code2 = uart4.recv_buff[2];
    }
}

USART_Init_Config_s uart6_cfg = {
    .recv_buff_size = 90,
    .usart_handle = &huart6,
    .module_callback = usart6_callback,
};
USART_Init_Config_s uart1_cfg = {
    .recv_buff_size = 40,
    .usart_handle = &huart1,
    .module_callback = usart1_callback,
};

USART_Init_Config_s uart2_cfg = {
    .recv_buff_size = 40,
    .usart_handle = &huart2,
    .module_callback = usart2_callback,
};

USART_Init_Config_s uart4_cfg = {
    .recv_buff_size = 40,
    .usart_handle = &huart4,
    .module_callback = usart4_callback,
};

void usart3_callback(void);
USART_Init_Config_s uart3_cfg = {
    .recv_buff_size = 60,
    .usart_handle = &huart3,
    .module_callback = usart3_callback,
};

//
float DEBUG = 0.0f;
float DEBUG2 = 0.0f;
float DEBUG3 = 0.0f;
int position_flag = 0;
int begin_flag = 1;
int safe_guard = 0;
cmd_vel_t debug_target_vel = {0, 0, 0};
odom_t debug_target_odom = {0, 0, 0};
odom_t debug_target_erro = {0.05, 0.05, 0.05};

// 实例化
static Controller_t ChassisControl_instance;
static Kinematic_t kinematic_instance;
static Planner_t planner_instance;
static StepMotorZDT_t zdt_stepmotor_instances[4]; // 静态实例
static StepMotorZDT_t upper_stepmotor_instance[1];

Controller_t *ChassisControl_ptr; // 控制器实例
Kinematic_t *kinematic_ptr;       // 麦轮实例
Planner_t *planner_ptr;           // 规划
StepMotorZDT_t *zdt_stepmotor_ptr[4] = {
    &zdt_stepmotor_instances[0],
    &zdt_stepmotor_instances[1],
    &zdt_stepmotor_instances[2],
    &zdt_stepmotor_instances[3],
};

TaskHandle_t LCD_Show_handle;        // 显示
TaskHandle_t Chassic_control_handle; // 底盘控制
TaskHandle_t main_cpp_handle;        // 主函数
TaskHandle_t Planner_update_handle;  // 轨迹规划
TaskHandle_t IMU_read_handle;        // IMU读取
TaskHandle_t tcs230_read_handle;     // tcs230颜色传感器读取
TaskHandle_t gray_read_handle;       // 灰度传感器
TaskHandle_t Get_Color_handle;       // 颜色传感器
void OnChassicControl(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);
void LCD_Show_task(void *pvParameters);
void gray_read_task(void *pvParameters);
void GwGet_color_task(void *pvParameters);
void UPPER_control_task(void *pvParameters);
void main_work(void)
{
    USARTRegister(&uart6, &uart6_cfg);
    USARTRegister(&uart3, &uart3_cfg);
    USARTRegister(&uart1, &uart1_cfg);
    USARTRegister(&uart2, &uart2_cfg);
    USARTRegister(&uart4, &uart4_cfg);
    memset(uart6.recv_buff, 0, uart6.recv_buff_size);
    memset(uart3.recv_buff, 0, uart3.recv_buff_size);
    memset(uart1.recv_buff, 0, uart1.recv_buff_size);
    memset(uart2.recv_buff, 0, uart2.recv_buff_size);
    memset(uart4.recv_buff, 0, uart4.recv_buff_size);

    // 注意电机编号如下所示
    //    Step_ZDT_Init(zdt_stepmotor_ptr[0], 1, &huart3, 0, 0.06f, false); // 左上
    //    Step_ZDT_Init(zdt_stepmotor_ptr[1], 2, &huart3, 1, 0.06f, false); // 右上
    //    Step_ZDT_Init(zdt_stepmotor_ptr[2], 4, &huart3, 0, 0.06f, false); // 左下
    //    Step_ZDT_Init(zdt_stepmotor_ptr[3], 3, &huart3, 1, 0.06f, true);  // 右下

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
    //	GetColorTask(color_task,&color_task_index);
    BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 300, NULL, 3, &Chassic_control_handle);
    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 800, NULL, 4, &main_cpp_handle);
    BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 200, NULL, 4, &Planner_update_handle);
    BaseType_t ok5 = xTaskCreate(GwGet_color_task, "GwGet_color", 200, NULL, 3, &Get_Color_handle);
    BaseType_t ok6 = xTaskCreate(LCD_Show_task, "LCD_Show_task", 200, NULL, 1, &LCD_Show_handle);
    BaseType_t ok8 = xTaskCreate(gray_read_task, "gray_read_task", 300, NULL, 2, &gray_read_handle);
    if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS || ok5 != pdPASS)
    {
        // 任务创建失败，进入死循环
        while (1)
        {
            // uart_printf("create task failed\n");
        }
    }
}

static void move_vel(float vel_x, float vel_y, float vel_yaw)
{
    motor_mode = 0;
    debug_target_vel = (cmd_vel_t){vel_x, vel_y, vel_yaw};
}

static void move_step_distance(float odom_x, float odom_y, float odom_yaw, bool clear_odom)
{
    motor_mode = 1;
    debug_target_odom = (odom_t){odom_x, odom_y, odom_yaw};
    debug_target_erro = (odom_t){0.005, 0.005, 0.005};
    Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 2.0f, &debug_target_erro, clear_odom);
}

void GwGet_color_task(void *pvParameters)
{
    while (Ping_color())
    {
        vTaskDelay(5);
    }

    while (1)
    {
        if (IIC_Get_RGB(RGB, 3))
        {
            goods_color_RGB = Get_GW_Color_RGB(RGB);
            current_color_RGB = goods_color_RGB;
        }
        if (IIC_Get_HSL(HSL, 3))
        {
            goods_color_HSL = Get_GW_Color_HSL(HSL);
            current_color_HSL = goods_color_HSL;
        }
        vTaskDelay(500);
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
        // 读取灰度传感器数据
        Digtal_gray_side = IIC_Get_Digtal(side);
        for (int i = 0; i < 8; i++)
        {
            digital_gray_data_side[i] = 1 - ((Digtal_gray_side >> i) & 0x01); // 读取侧边数字灰度传感器数据
        }

        // 获取传感器模拟量结果
        if (IIC_Get_Anolog(Anolog_gray_side, 8, side))
        {
        }

        // 获取传感器归一化结果
        IIC_Anolog_Normalize(0xff, side); // 所有通道归一化都打开
        vTaskDelay(10);                   // 设置完，需要等上一会。stm8的运算速度没stm32快，等一下，让传感器把数据刷新一下。
        if (IIC_Get_Anolog(Normal_side, 8, side))
        {
        }
        IIC_Anolog_Normalize(0xff, side);

        for (int i = 0; i < 8; i++)
        {
            gray_data_side_middle_temp += digital_gray_data_side[i] * sensor_weights_side[i] * gray_side_p;
            gray_data_side_sum_temp += digital_gray_data_side[i];
        }
        gray_data_side_middle = gray_data_side_middle_temp;
        gray_data_side_middle_temp = 0;
        gray_data_side_sum = gray_data_side_sum_temp;
        gray_data_side_sum_temp = 0;

        if (gray_data_side_sum >= 4)
        {
            real_time_gray_state_side = aim_black;
            BUZZER_ON;
        }
        else
        {
            real_time_gray_state_side = orgin_gray;
            BUZZER_OFF;
        }

        vTaskDelay(10); // 延时10ms
    }
}

void LCD_Show_task(void *pvParameters)
{
    // 屏幕
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);

    while (1)
    {
        if (DEBUG_UPPER == 1)
        {
            Servo_SetAngle(&servo[0], debug_angle[0], 270);
            Servo_SetAngle(&servo[1], debug_angle[1], 180);
            Servo_SetAngle(&servo[2], debug_angle[2], 360);
            HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, debug_isOpened);
        }
        DistributionLoop(servo, plate_things, current_color_ptr, upperflag_ptr, &CurrentColorLoop);
        PutGoal(color_task, servo, plate_things, upperflag_ptr, &PutGoalLoop);
        if(CurrentColorLoop ==5 && PutGoalLoop == 5)
        {
            DistributionRankLoop(rank_task, servo, plate_task2_things, upperflag_ptr, &CurrentRankLoop);
            PutRank(servo, plate_task2_things, upperflag_ptr, &PutRankLoop);
        }

        //        // 显示
        //        // 陀螺仪
        //        LCD_ShowFloatNum1(0, 20, gyro[0], 4, RED, WHITE, 16);
        //        LCD_ShowString(48, 20, ",", RED, WHITE, 16, 0);
        //        LCD_ShowFloatNum1(58, 20, gyro[1], 4, RED, WHITE, 16);
        //        LCD_ShowString(106, 40, ",", RED, WHITE, 16, 0);
        //   LCD_ShowFloatNum1(0, 20, HSL[0], 8, RED, WHITE, 16);
        // LCD_ShowFloatNum1(0, 40, goods_color_HSL, 8, RED, WHITE, 16);
        // 			LCD_ShowFloatNum1(0, 60, HSL[2], 8, RED, WHITE, 16);
        //        // 加速度
        //        LCD_ShowFloatNum1(0, 40, accel[0], 4, RED, WHITE, 16);
        //        LCD_ShowString(48, 40, ",", RED, WHITE, 16, 0);
        //        LCD_ShowFloatNum1(58, 40, accel[1], 4, RED, WHITE, 16);
        //        LCD_ShowString(106, 40, ",", RED, WHITE, 16, 0);
        //        LCD_ShowFloatNum1(116, 40, accel[2], 4, RED, WHITE, 16);
        //        // 显示temp
        //        LCD_ShowFloatNum1(10, 60, temp, 4, RED, WHITE, 16);
        //        LCD_ShowString(52, 60, ",", RED, WHITE, 16, 0);
        //        LCD_ShowString(62, 60, "gyro", RED, WHITE, 16, 0);
        //        LCD_ShowString(100, 60, ",", RED, WHITE, 16, 0);
        //        LCD_ShowString(106, 60, "accel", RED, WHITE, 16, 0);
        // vTaskDelay(100);
    }
}

void Onmaincpp(void *pvParameters)
{
#if DEBUG_TASK == 1
    int safe_count = 0; // 保护锁
    while (1)
    {
        safe_count++;
        if (safe_count >= 3)
        {
            safe_guard = 1; // 保护锁打开
            switch (main_state)
            {
            case 0:
            {
                setYawZero();
                Servo_SetAngle(&servo[1], UP, 180);
                vTaskDelay(200);
                move_step_distance(0, 0, 3.1415, 1);
                main_state++;
                break;
            }
            }
        }
        vTaskDelay(30);
    }
#endif
#ifdef DEBUG_PARAM_PUT
    main_state = 23;
    main_put_state = 0;
#endif
    int safe_count = 0; // 保护锁
    while (1)
    {
        safe_count++;
        if (safe_count >= 3)
        {
            safe_guard = 1; // 保护锁打开
#ifdef TASK1
            switch (main_state)
            {
            case 0:
            {
                setYawZero();

                Servo_SetAngle(&servo[1], UP, 180);
                vTaskDelay(200);
                move_step_distance(0, -0.3, 0, 1);
                main_state++;
                break;
            }
            case 1:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(100);
                    move_step_distance(0.6, 0, 0, 1);
                    main_state++;
                }
                break;
            }
            case 2:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_vel(0.1, 0, 0);
                    main_state++;
                    vTaskDelay(100);
                }
                break;
            }
            case 3:
            {
                if (real_time_gray_state_side == aim_black)
                {
                    move_vel(0, 0, 0);
                    main_state++;
                }
                break;
            }
                /////////////*********对十字中************/////////
            case 4:
            {
                vTaskDelay(100);
                move_step_distance(0, gray_data_side_middle, 0, 1);
                main_state++;
                break;
            }
            /// 到二维码处
            case 5:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(100);
                    move_step_distance(0.15, 0.04, 0, 1);
                    main_state++;
                }

                break;
            }

                /////////////等待直到二维码识别成功,到第一个物块处
            case 6:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise) && qr_mv_code != 0)
                {
                    vTaskDelay(500);
                    GetColorTask(color_task, qr_mv_code_ptr);
                    move_step_distance(0.15, 0.02, 0, 1);
                    main_state++;
                }
                break;
            }

            case 7:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(200);
                    move_step_distance(0.15, 0, 0, 1);
                    main_state++;
                }
                break;
            }
            case 8:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_step_distance(-0.0563, 0, 0, 1);
                    vTaskDelay(1000);
                    *upperflag_ptr = PICKINGIN;
                    main_state++;
                }
                break;
            }
                // **********抓完第一个，去找第二个物块***********///
                // 第一个到第二个是dx:346.6(mm),dy:341.136(mm)
            case 9:
            {
                if (*upperflag_ptr == IDLE) // 抓完第一个还是很正的
                {
                    vTaskDelay(1000);
                    move_step_distance(0.07, -0.41, 0, 1);
                    main_state++;
                }
                break;
            }
            case 10:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0.28, 0, 0, 1);
                    main_state++;
                }
            }

                ////********抓取第二个物块**********/////
            case 11:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_step_distance(-0.0563, 0, 0, 1);
                    vTaskDelay(1000);
                    *upperflag_ptr = PICKINGIN;
                    vTaskDelay(200);
                    main_state++;
                }

                break;
            }
                // *************抓第二个完成，去找第三个物块************////
                // 第二 个到第三个是dx:470.109(mm),dy:124.624(mm)
            case 12:
            {
                if (*upperflag_ptr == IDLE)
                {
                    vTaskDelay(1000);
                    move_step_distance(-0.1, -0.53, 0, 1);
                    main_state++;
                }
                break;
            }
            case 13:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0.23, 0, 0, 1);
                    main_state++;
                }
            }
                // *************抓第三个物块********//
            case 14:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_step_distance(-0.0563, 0, 0, 1);
                    vTaskDelay(1000);
                    *upperflag_ptr = PICKINGIN;

                    vTaskDelay(200);
                    main_state++;
                }

                break;
            }

                // *************抓第三个完成，去找第四个物块************////
                // 第三 个到第四个是dx:470.109(mm),dy:124.624(mm)
            case 15:
            {
                if (*upperflag_ptr == IDLE)
                {
                    vTaskDelay(1000);
                    move_step_distance(-0.33, -0.49, 0, 1);
                    main_state++;
                }
                break;
            }
            case 16:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0.3, 0, 0, 1);
                    main_state++;
                }
            }
                // *************抓第四个物块********//
            case 17:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_step_distance(-0.0563, 0, 0, 1);
                    vTaskDelay(1000);
                    *upperflag_ptr = PICKINGIN;

                    vTaskDelay(200);
                    main_state++;
                }

                break;
            }

                // *************抓第四个完成，去找第五个物块************////
                // 第四 个到第五个是dx:346.614(mm),dy:341.136(mm)
            case 18:
            {
                if (*upperflag_ptr == IDLE)
                {
                    vTaskDelay(1000);
                    move_step_distance(-0.63, 0, 0, 1);
                    main_state++;
                }
                break;
            }
            case 19:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(200);
                    move_step_distance(0, -0.38, 0, 1);
                    main_state++;
                }
            }

            case 20:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(200);
                    move_step_distance(0.36, 0, 0, 1);
                    main_state++;
                }
            }
                // *************抓第五个物块********//
            case 21:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_step_distance(-0.0563, 0, 0, 1);
                    vTaskDelay(1000);
                    *upperflag_ptr = PICKINGIN;
                    vTaskDelay(200);
                    main_state++;
                }

                break;
            }

            case 22:
            {
                if (*upperflag_ptr == IDLE)
                {
                    vTaskDelay(200);
                    main_state++;       // 防止出bug
                    main_put_state = 0; // 开启放置任务
                }

                break;
            }

            default:
                break;
            }

            //////**************************开启放置物块状态机******************************/////

            /////顺序////////////////****************************BADCE**********************//////
            if (main_state > 22)
            {
                switch (main_put_state)
                {
                case 0:
                {
                    vTaskDelay(500);
                    move_step_distance(0, 0, 1.571, 1);
                    main_put_state++;
                    break;
                }
                // 去找十字纠正自身
                case 1:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        vTaskDelay(200);
                        setYawZero();
                        vTaskDelay(500);
                        move_step_distance(0.35, 0.37, 0, 1);
                        main_put_state++;
                    }
                    break;
                }
                case 2:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        move_vel(0.1, 0, 0);
                        if (gray_data_side_sum >= 3)
                        {
                            move_vel(0, 0, 0);
                            main_put_state++;
                        }
                    }

                    break;
                }
                    /////////*************开始对第一个十字******************////////////////
                case 3:
                {

                    if (gray_data_side_middle != 0)
                    {
                        vTaskDelay(100);
                        move_step_distance(0, gray_data_side_middle, 0, 1);
                        main_put_state++;
                    }
                    else
                    {
                        main_put_state++;
                    }
                    break;
                }
                    ////////////*****从AAAAAAAAAAAAAAA到BBBBBBBBBBBBBB(-23，50)**********/////////
                case 4:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        // 注意要抬升到上面
                        vTaskDelay(100);
                        move_step_distance(0.40, 0.21, 0, 1);
                        main_put_state++;
                    }

                    break;
                }
                // 等待视觉
                case 5:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {

                        main_put_state++;
                    }
                    break;
                }

                ///////////***********放置第一个:::::::AAAAAAAAAAAAAAA********////////
                case 6:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        upperflag = PICKINGOUT;
                        vTaskDelay(200);
                        main_put_state++;
                    }
                    break;
                }

                ///////// 空闲状态然后去另一个地方a,b到a(20,25)////////
                case 7:
                {
                    if (*upperflag_ptr == IDLE)
                    {
                        vTaskDelay(200);
                        move_step_distance(-0.30, -0.23, 0, 1);
                        main_put_state++;
                    }
                    break;
                }
                case 8:
                {

                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        main_put_state++;
                    }
                    break;
                }

                //*************等视觉对准第二个************//////////
                case 9:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        //                        if (abs(final_circle_dx) < 0.05 && abs(final_circle_dy) < 0.05 && final_circle_dx != 0 && final_circle_dy != 0)
                        //                        {
                        //                            vTaskDelay(200);
                        //                             move_step_distance(final_circle_dy,-final_circle_dx, 0, 1);
                        //                            main_put_state++;
                        //                        }
                        main_put_state++;
                    }
                    break;
                }
                    ///////////***********放置第二个AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA:::::::********////////
                case 10:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        upperflag = PICKINGOUT;
                        vTaskDelay(200);
                        main_put_state++;
                    }
                    break;
                }
                    // ***************空闲状态然后去另一个地方A,A到D(-100,50)**************//////////
                case 11:
                {
                    if (*upperflag_ptr == IDLE)
                    {
                        vTaskDelay(200);
                        move_step_distance(0, 1.08, 0, 1);
                        main_put_state++;
                    }
                    break;
                }
                case 12:
                {

                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        vTaskDelay(200);
                        move_step_distance(0.28, 0, 0, 1);
                        main_put_state++;
                    }
                    break;
                }
                ////到c了等待对准第三个/////
                case 13:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        //                        if (abs(final_circle_dx) < 0.05 && abs(final_circle_dy) < 0.05 && final_circle_dx != 0 && final_circle_dy != 0)
                        //                        {
                        //                            vTaskDelay(200);
                        //                            move_step_distance(final_circle_dy,-final_circle_dx, 0, 1);
                        //                            main_put_state++;
                        //                        }
                        main_put_state++;
                    }
                    break;
                }
                ///////////***********放置第三个:::::::DDDDDDDDDDDDDDDDDDDDD********////////
                case 14:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        upperflag = PICKINGOUT;
                        vTaskDelay(200);
                        main_put_state++;
                    }
                    break;
                }
                /////////////等待空闲///////
                case 15:
                {

                    if (*upperflag_ptr == IDLE)
                    {
                        vTaskDelay(200);
                        move_step_distance(-0.30, -0.15, 0, 1);
                        main_put_state++;
                    }
                    break;
                }
                case 16:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        vTaskDelay(200);
                        //                        move_step_distance(-0.45, 0, 0, 1);
                        main_put_state++;
                    }
                    break;
                }
                case 17:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        //                        if (abs(final_circle_dx) < 0.05 && abs(final_circle_dy) < 0.05 && final_circle_dx != 0 && final_circle_dy != 0)
                        //                        {
                        //                            vTaskDelay(200);
                        //                            move_step_distance(final_circle_dy,-final_circle_dx, 0, 1);
                        //                            main_put_state++;
                        //                        }

                        main_put_state++;
                        // else if (wait_vision > 6)
                        // {
                        //     move_step_distance(0.01, 0.01, 0, 1);
                        //     main_put_state++;
                        //     wait_vision = 0;
                        // }
                    }
                    break;
                }
                ///////////////***************放置第四个物块CCCCCCCCCCCCCCCCCCCCCCCCCCCC***************////////
                case 18:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        upperflag = PICKINGOUT;
                        vTaskDelay(200);

                        main_put_state++;
                    }
                    break;
                }

                    /////////////等待放置完成///////
                case 19:
                {

                    if (*upperflag_ptr == IDLE)
                    {
                        vTaskDelay(200);
                        move_step_distance(0, 0.5, 0, 1);
                        main_put_state++;
                    }
                    break;
                }
                ///////////////////去找第五个物块EEEEEEEEEEEEEEEEEEEEE（-65，40）/////////////////
                case 20:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        vTaskDelay(200);
                        move_step_distance(0.29, 0.25, 0, 1);
                        main_put_state++;
                    }
                    break;
                }

                case 21:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        main_put_state++;
                    }
                    break;
                }
                ///////////////***************放置第五个物块EEEEEEEEEEEEEE***************////////
                case 22:
                {
                    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                    {
                        upperflag = PICKINGOUT;
                        vTaskDelay(200);
                        main_put_state++;
                    }
                    break;
                }


                default:
                    break;
                }
            }
#endif
#ifdef TASK2

#ifdef FINALTASK2
main_second_state=17;
#endif
            switch (main_second_state)
            {
           case 0:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise) && qr_code != 0 )
                {
                    CurrentColorLoop =5;
                    PutGoalLoop =5;
                    setYawZero();
                    vTaskDelay(200);
                    GetRankTask(rank_task, qr_code_ptr);
                    vTaskDelay(2000);
                    move_step_distance(0, 0, 3.1415926, 1);
                    main_second_state++;
					vTaskDelay(2000);
                }
                break;
            }
            // 去三那
            case 1:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(200);
                    setYawZero();
                    vTaskDelay(500);
                    move_step_distance(0.6, 0.47, 0, 1);
                    main_second_state++;
                }
                break;
            }
						case 2:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0.2, 0, 0, 1);
                    main_second_state++;
                }
                break;
            }
            case 3:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(-0.0563, 0, 0, 1);
                    main_second_state++;
                }
                break;
            }
            // 抓取第一个物块
            case 4:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    *upperflag_ptr = PICKINGIN;
                    vTaskDelay(500);
                    main_second_state++;
                }
                break;
            }
            //去找2//
            case 5:
            {
                if (*upperflag_ptr == IDLE) // 抓完第一个还是很正的
                {
                    vTaskDelay(1000);
                    move_step_distance(0.1, 0.43, 0, 1);
                    main_second_state++;
                }
                break;
            }
            case 6:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0.2, 0, 0, 1);
                    main_second_state++;
                }
                break;
            }
            case 7:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(-0.0563, 0, 0, 1);
                    main_second_state++;
                }
                break;
            }
            // 抓取第二个物块
            case 8:
            {

                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    *upperflag_ptr = PICKINGIN;
                    vTaskDelay(500);
                    main_second_state++;
                }
                break;
            }
                /// 去找最后
            case 9:
            {
                if (*upperflag_ptr == IDLE) 
                {
                    vTaskDelay(1000);
                    move_step_distance(-0.3, 0.43, 0, 1);
                    main_second_state++;
                }
                break;
            }
            case 10:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0.3, 0, 0, 1);
                    main_second_state++;
                }
                break;
            }
            case 11:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(-0.0563, 0, 0, 1);
                    main_second_state++;
                }
                break;
            }
            // 抓取最后物块
            case 12:
            {

                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    *upperflag_ptr = PICKINGIN;
                    vTaskDelay(500);
                    main_second_state++;
                }
                break;
            }
            // 旋转去冠亚季领奖台
            case 13:
            {
                if (*upperflag_ptr == IDLE)
                {
                    vTaskDelay(1000);
                    move_step_distance(0, 0, PI / 2, 1);
                    main_second_state++;
                }
                break;
            }
            case 14:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(200);
                    setYawZero();
                    vTaskDelay(500);
                    move_step_distance(0.23, 1.74, 0, 1);
                    main_second_state++;
                }
                break;
            }
            case 15:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_vel(0.1, 0, 0);
                    if (real_time_gray_state_side == aim_black)
                    {
                        move_vel(0, 0, 0);
                        main_second_state++;
                    }
                }

                break;
            }
            //找冠圈
            case 16:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0.10, 0, 0, 1);
                    main_second_state++;
                }

                break;
            }
            //放冠圈
            case 17:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
					CurrentColorLoop =5;
                    PutGoalLoop =5;
                    upperflag = PICKINGOUT;
                    vTaskDelay(200);
                    main_second_state++;
                }
                break;
            }
            //找亚圈
            case 18:
            {
                if (*upperflag_ptr == IDLE)
                {
                    vTaskDelay(500);
                    move_step_distance(-0.10, -0.15, 0, 1);
                    main_second_state++;
                }
                break;
            }
            //去亚圈（从十字到圈）
            case 19:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0.10, 0, 0, 1);
                    main_second_state++;
                }
                break;
            }
            //放亚圈
            case 20:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    upperflag = PICKINGOUT;
                    vTaskDelay(200);
                    main_second_state++;
                }
                break;
            }
            //去季圈十字
            case 21:
            {
                if (*upperflag_ptr == IDLE)
                {
                    vTaskDelay(500);
                    move_step_distance(-0.10, 0.30, 0, 1);
                    main_second_state++;
                }
                break;
            }
            //去季圈
            case 22:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0.1, 0, 0, 1);
                    main_second_state++;
                }
                break;
            }
            //放季圈
            case 23:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    upperflag = PICKINGOUT;
                    vTaskDelay(200);
                    main_second_state++;
                }
                break;
            }

            default:
                break;

            }
#endif
            switch (motor_mode)
            {
            case 0:
            {
                Controller_set_vel_target(ChassisControl_ptr, debug_target_vel, false);
                break;
            }
            case 1:
            {
                break;
            }
            default:
                break;
            }
        }
        vTaskDelay(30);
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
    vTaskDelay(1000); // 等待一会
    uint16_t last_tick = xTaskGetTickCount();

    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
        if (safe_guard == 1)
        {
            Controller_KinematicAndControlUpdateWithYaw(ChassisControl_ptr, dt, main_yaw);
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

void usart3_callback(void)
{
    // 处理接收到的数据
    uint8_t *data = uart3.recv_buff;
    for (int i = 0; i <= 3; i++)
    {
        if (data[0] == i) // 检查地址和功能码
        {
            if (data[1] == 0x37 && data[7] == 0x6B)
            {
                // 解析位置误差数据
                int64_t pose_error = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6]; // 解析位置误差数据
                pose_error = (data[2] == 0x0) ? pose_error : -pose_error;                          // 更新目标误差(电机维护方向)
                zdt_stepmotor_ptr[i]->_target_pose_error = pose_error * 360 / 65536.0;
                if (zdt_stepmotor_ptr[i]->_dir != 0)
                {
                    zdt_stepmotor_ptr[i]->_target_pose_error = -zdt_stepmotor_ptr[i]->_target_pose_error; // 如果是反转方向，则取反(控制维护的方向)
                }
            }
        }
    }
}