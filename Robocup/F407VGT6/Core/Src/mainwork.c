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
#include "ch040.h"
#include "gw_color_iic.h"
#include "servo.h"
#include "upper.h"
#define BUZZER_ON HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
#define BUZZER_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
#define get_little_yellow_state HAL_GPIO_ReadPin(little_yellow_GPIO_Port, little_yellow_Pin)
#define abs(x) (x > 0 ? x : (-x))
Servo_t servo[1] = {
    {&htim3, TIM_CHANNEL_4, 83, 0}};
int pick_goods_flag = 0;
int put_goods_flag = 0;
UpperTaskFlag upperflag = IDLE; // 上层机构状态机
UpperTaskFlag *upperflag_ptr = &upperflag;
ThingStore_t plate_things[5] = {0}; // 料盘槽数组

Color_t color_task[5];
Color_t current_color = COLOR_BLACK; // 当前颜色
Color_t *current_color_ptr = &current_color;
int CurrentColorLoop = 0;
int PutGoalLoop = 0;
// 主函数状态机
__IO int main_state = 0;
int motor_mode = 0;
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
int sensor_weights_side[8] = {-7, -5, -4, -2, 2, 4, 5, 7}; // 传感器权重
unsigned char Digtal_gray_side;
unsigned char Anolog_gray_side[8] = {0};
unsigned char Normal_side[8] = {0};
// 调试信息
int debug_pwm = 0;
int close_flag = 0;
int safe_flag = 0;

// 气泵
int get_yellow_flag = 0;
int yellow_state = 0;
// 上升控制
int first_upper_flag = 1;
upper_location now_upper_loacation = up_location;
upper_location target_upper_loacation = up_location;
int upper_flag = 0;
int upper_rotate_pwm = 960;
int pump_flag = 0;

// 陀螺仪数据yaw
float main_yaw = 0.0f;
// 串口接收
USARTInstance uart6 = {0};
USARTInstance uart3 = {0};
USARTInstance uart1 = {0};
// 读陀螺仪
void usart6_callback(void)
{
    if (uart6.recv_buff[0] == 0x5A && uart6.recv_buff[1] == 0xA5)
    {
        main_yaw = ch040_get_data(uart6.recv_buff);
    }
}
USART_Init_Config_s uart6_cfg = {
    .recv_buff_size = 90,
    .usart_handle = &huart6,
    .module_callback = usart6_callback,
};
//上位机通信
void usart1_callback(void)
{
	
	
}
USART_Init_Config_s uart1_cfg = {
    .recv_buff_size = 90,
    .usart_handle = &huart1,
    .module_callback = usart1_callback,
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
    memset(uart6.recv_buff, 0, uart6.recv_buff_size);
    memset(uart3.recv_buff, 0, uart3.recv_buff_size);
	memset(uart1.recv_buff, 0, uart1.recv_buff_size);
    // 注意电机编号如下所示

    //    Step_ZDT_Init(zdt_stepmotor_ptr[0], 1, &huart3, 0, 0.06f, false); // 左上
    //    Step_ZDT_Init(zdt_stepmotor_ptr[1], 2, &huart3, 1, 0.06f, false); // 右上
    //    Step_ZDT_Init(zdt_stepmotor_ptr[2], 4, &huart3, 0, 0.06f, false); // 左下
    //    Step_ZDT_Init(zdt_stepmotor_ptr[3], 3, &huart3, 1, 0.06f, true);  // 右下

    Step_ZDT_Init(zdt_stepmotor_ptr[0], 1, &huart3, 1, 0.08f, false); // 左上
    Step_ZDT_Init(zdt_stepmotor_ptr[1], 2, &huart3, 0, 0.08f, false); // 右上
    Step_ZDT_Init(zdt_stepmotor_ptr[2], 4, &huart3, 1, 0.08f, false); // 左下
    Step_ZDT_Init(zdt_stepmotor_ptr[3], 3, &huart3, 0, 0.08f, true);  // 右下

    ChassisControl_ptr = &ChassisControl_instance;
    kinematic_ptr = &kinematic_instance;
    planner_ptr = &planner_instance;
    Kinematic_init(kinematic_ptr, 0.6, 2, X_shape);
    Controller_Init(ChassisControl_ptr, zdt_stepmotor_ptr, kinematic_ptr);
    Planner_init(planner_ptr, ChassisControl_ptr);

    BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 300, NULL, 3, &Chassic_control_handle);
    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
    BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 200, NULL, 4, &Planner_update_handle);
    BaseType_t ok5 = xTaskCreate(GwGet_color_task, "GwGet_color", 200, NULL, 3, &Get_Color_handle);
    BaseType_t ok6 = xTaskCreate(LCD_Show_task, "LCD_Show_task", 200, NULL, 1, &LCD_Show_handle);
    BaseType_t ok7 = xTaskCreate(UPPER_control_task, "UPPER_control_task", 200, NULL, 1, &LCD_Show_handle);
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

void GwGet_color_task(void *pvParameters)
{
    while (Ping_color())
    {
        vTaskDelay(5);
    }

    while (1)
    {

        if (IIC_Get_HSL(HSL, 3))
        {
            goods_color_HSL = Get_GW_Color_HSL(HSL);
            *current_color_ptr = goods_color_HSL;
        }
        vTaskDelay(200);
    }
}

// 上层结构
void UPPER_control_task(void *pvParameters)
{

    vTaskDelay(1000);
    //__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,2050);
    Servo_SetAngle(servo, CENTER_PICK, 360);
    while (1)
    {
        if (pick_goods_flag == 1)
        {
            *upperflag_ptr = PICKINGIN;
            pick_goods_flag = 0;
        }
        if (put_goods_flag == 1)
        {
            *upperflag_ptr = PICKINGOUT;
            put_goods_flag = 0;
        }
        DistributionLoop(servo, plate_things, current_color_ptr, upperflag_ptr, &CurrentColorLoop);
        PutGoal(color_task, servo, plate_things, upperflag_ptr, &PutGoalLoop);

        vTaskDelay(100);
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
        if (get_yellow_flag)
        {
            yellow_state = get_little_yellow_state;
        }
        else
        {
            yellow_state = -1;
        }

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

        if (gray_data_side_sum >= 3)
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

        // 显示
        //        LCD_ShowString(48, 20, ",", RED, WHITE, 16, 0);
        LCD_ShowFloatNum1(0, 20, HSL[0], 8, RED, WHITE, 16);
        LCD_ShowFloatNum1(0, 40, goods_color_HSL, 8, RED, WHITE, 16);
        //							LCD_ShowFloatNum1(0, 60, HSL[2], 8, RED, WHITE, 16);
        LCD_ShowFloatNum1(0, 60, abs(main_yaw), 8, RED, WHITE, 16);

        vTaskDelay(100);
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
    Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 0.5, &debug_target_erro, clear_odom);
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
                move_step_distance(0.29, 0, 0, 1);
                main_state++;
                break;
            }

            case 1:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_vel(0, 0.18, 0);
                    if (real_time_gray_state_side == aim_black)
                    {
                        move_vel(0, 0, 0);
                        main_state++;
                    }
                }
                break;
            }
            /////**********对十字中********//////
            case 2:
            {
							               while (gray_data_side_sum < 6&&(gray_data_side_middle!=0))
                {
                    if (gray_data_side_middle != 0)
                    {
                        move_step_distance(-gray_data_side_middle, 0, 0, 1);
                    }
										vTaskDelay(20);
                }
							main_state++;
//							if(gray_data_side_sum==8)
//							{
//							main_state++;
//							}							
//              else if (gray_data_side_sum < 6||(gray_data_side_middle!=0))
//                {
//                    if (abs(gray_data_side_middle) >0.01)
//                    {
//                        move_step_distance(-gray_data_side_middle, 0, 0, 1);
//                    }        
//                    vTaskDelay(100);
//                }
//						if(digital_gray_data_side[0]==1||digital_gray_data_side[1]==1)
//						{
//						main_state++;					
//						}							
//							else {
//							
//					           if (abs(gray_data_side_middle) >0.01)
//                    {
//                        move_step_distance(-gray_data_side_middle, 0, 0, 1);
//                    }        
//                    vTaskDelay(100);
//							}

                break;
            }
                //////*********找第一个物块****//////
            case 3:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    move_step_distance(-0.042, 0.46, 0, 1);
                    main_state++;
                }
                break;
            }

            case 4:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0, -0.065, 0, 1);
                    main_state++;
                }
                break;
            }
                //****************抓取第一个*********///////
            case 5:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    pick_goods_flag = 1;

                    vTaskDelay(200);
                    main_state++;
                }
                break;
            }

                // **********抓完第一个，去找第二个物块***********///
                // 第一个到第二个是dx:346.6(mm),dy:341.136(mm)
            case 6:
            {
                if (*upperflag_ptr == IDLE) // 抓完第一个还是很正的
                {
                    vTaskDelay(1000);
                    move_step_distance(0.36, 0.28, 0, 1);
                    main_state++;
                }
                break;
            }
            case 7:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0, 0.20, 0, 1);
                    main_state++;
                }
            }
            case 8:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0, -0.065, 0, 1);
                    main_state++;
                }
                break;
            }

                ////********抓取第二个物块**********/////
            case 9:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {

                    pick_goods_flag = 1;

                    vTaskDelay(200);
                    main_state++;
                }

                break;
            }
                // *************抓第二个完成，去找第三个物块************////
                // 第二 个到第三个是dx:470.109(mm),dy:124.624(mm)
            case 10:
            {
                if (*upperflag_ptr == IDLE)
                {
                    vTaskDelay(1000);
                    move_step_distance(0.50, 0.11, 0, 1);
                    main_state++;
                }
                break;
            }
            case 11:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0, 0.2, 0, 1);
                    main_state++;
                }
            }
            case 12:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0, -0.065, 0, 1);
                    main_state++;
                }
                break;
            }
                // *************抓第三个物块********//
            case 13:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {

                    pick_goods_flag = 1;

                    vTaskDelay(200);
                    main_state++;
                }

                break;
            }

                // *************抓第三个完成，去找第四个物块************////
                // 第三 个到第四个是dx:470.109(mm),dy:124.624(mm)
            case 14:
            {
                if (*upperflag_ptr == IDLE)
                {
                    vTaskDelay(1000);
                    move_step_distance(0.48, -0.4, 0, 1);
                    main_state++;
                }
                break;
            }
            case 15:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0, 0.3, 0, 1);
                    main_state++;
                }
            }
            case 16:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(500);
                    move_step_distance(0, -0.065, 0, 1);
                    main_state++;
                }
                break;
            }
                // *************抓第四个物块********//
            case 17:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {

                    pick_goods_flag = 1;

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
                    move_step_distance(0, -0.63, 0, 1);
                    main_state++;
                }
                break;
            }
						            case 19:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(200);
                    move_step_distance(0.38, 0, 0, 1);
                    main_state++;
                }
            }
						
						
            case 20:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(200);
                    move_step_distance(0, 0.33, 0, 1);
                    main_state++;
                }
            }
            case 21:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {
                    vTaskDelay(200);
                    move_step_distance(0, -0.065, 0, 1);
                    main_state++;
                }
                break;
            }
                // *************抓第五个物块********//
            case 22:
            {
                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
                {

                    pick_goods_flag = 1;

                    vTaskDelay(200);
                    main_state++;
                }

                break;
            }

            default:
                break;
            }

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
    int safe_upper_count = 0;
    uint16_t last_tick = xTaskGetTickCount();
    vTaskDelay(1000);

    while (1)
    {
        safe_upper_count++;
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
        if (safe_guard)
        {
            //            if (safe_upper_count >= 50 && first_upper_flag == 1)
            //            {
            //                upper_move_distance(5, 0, 300, 0.02, 300, 0, 0); // 上升到中间位置
            //                first_upper_flag = 0;
            //            }

            upper_to_target(target_upper_loacation);
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