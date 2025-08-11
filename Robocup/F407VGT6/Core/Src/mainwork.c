/*
 * @Author: Nagisa 2964793117@qq.com
 * @Date: 2025-08-07 15:49:23
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2025-08-11 12:57:17
 * @FilePath: \MDK-ARMd:\project\git\robotcup\Robocup\F407VGT6\Core\Src\mainwork.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
#include "chassislogic.h"
#define BUZZER_ON HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
#define BUZZER_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);

#define DEBUG_UPPER 0
#define DEBUG_CHASSIS 1

#define get_little_yellow_state			HAL_GPIO_ReadPin(little_yellow_GPIO_Port,little_yellow_Pin)

float debug_angle[3] = {0, 0, 0}; // 调试角度
int debug_isOpened = 0;

int debug_speed = 0;
int debug_distance = 0; // 调试开始标志
float debug_chassis_speed[3] = {0};
float debug_chassis_distance[3] = {0};

Servo_t servo[3]= {
    {&htim9, TIM_CHANNEL_2, 0, 0},
    {&htim9, TIM_CHANNEL_1, 0, 0},
    {&htim5, TIM_CHANNEL_3, 0, 0}
};
UpperTaskFlag upperflag = IDLE; // 上层机构状态机
UpperTaskFlag* upperflag_ptr = &upperflag;
ChassisTaskFlag chassisflag = IDLE_CHASSIS; // 底盘状态机
// ThingStore_t plate_things[5] = {0}; // 料盘槽数组
ThingStore_t plate_things[5] = {
    {COLOR_BLACK, 33, 0},
    {COLOR_WHITE, 93, 1},
    {COLOR_RED, 153, 2},
    {COLOR_BLUE, 213, 3},
    {COLOR_GREEN, 273, 4}
};
Color_t color_task[5] = {COLOR_BLACK, COLOR_WHITE, COLOR_RED, COLOR_BLUE, COLOR_GREEN}; // 颜色任务数组
Color_t current_color_RGB = COLOR_BLACK; // 当前颜色
Color_t current_color_HSL = COLOR_BLACK; // 当前颜色
Color_t* current_color_ptr = &current_color_RGB;
int CurrentColorLoop = 0;
int PutGoalLoop = 0; // 目标放置循环
float main_yaw = 0.0f; // imu存取的yaw
int safe_count = 0; // 保护锁

// 主函数状态机
int main_state = 0;
int motor_mode = 0;
// 颜色传感器 
int GET_RGB_FLAG=0;
int GET_HSL_FLAG=0;
int goods_color_RGB=-1;
int goods_color_HSL=-1;
unsigned char RGB[3] = {0};
unsigned char HSL[3] = {0};
// 灰度
gray_state real_time_gray_state = orgin_gray;      // 主灰度状态
gray_state real_time_gray_state_side = orgin_gray; // 侧边灰度
// 前面灰度
float gray_front_p = 0.01f; // 前面灰度传感器的神秘小参数
float gray_data_front_middle = 0;
float gray_data_front_middle_temp = 0;
uint8_t digital_gray_data_front[8];
int sensor_weights_front[8] = {-7, -4, -3, -2, 2, 3, 4, 7}; // 传感器权重
unsigned char Digtal_gray_front;
unsigned char Anolog_gray_front[8] = {0};
unsigned char Normal_front[8] = {0};
// 侧边灰度
float gray_side_p = 0.01f;
float gray_data_side_middle = 0;
float gray_data_side_middle_temp = 0;
uint8_t digital_gray_data_side[8];
int sensor_weights_side[8] = {-7, -4, -3, -2, 2, 3, 4, 7}; // 传感器权重
unsigned char Digtal_gray_side;
unsigned char Anolog_gray_side[8] = {0};
unsigned char Normal_side[8] = {0};
// 调试信息
int debug_pwm = 0;
int close_flag = 0;
int safe_flag = 0;
USARTInstance uart6 = {0};
//气泵
int pump_flag=0;
int get_yellow_flag=0;
int yellow_state=0;
//上升控制
int now_upper_loacation=0;
int target_upper_loacation=0;
float target_distance=0;
float upper_target_vel=0;
int upper_flag=0;


void usart6_callfront(void)
{
    if (uart6.recv_buff[0] == 0x5A && uart6.recv_buff[1] == 0xA5)
    {
        main_yaw = ch040_get_data(uart6.recv_buff);
    }
}
USART_Init_Config_s uart6_cfg = {
    .recv_buff_size = 90,
    .usart_handle = &huart6,
    .module_callback = usart6_callfront,
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

StepMotorZDT_t *upper_stepmotor_ptr[1]=
{
 &upper_stepmotor_instance[0],
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
void IMU_Read_task(void *pvParameters);
void LCD_Show_task(void *pvParameters);
void tcs230_read_task(void *pvParameters);
void gray_read_task(void *pvParameters);
void GwGet_color_task(void *pvParameters);
void main_work(void)
{
    HAL_UART_Receive_IT(&huart4, &RxData, 1);
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");


			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,965);//初始965
	
    USARTRegister(&uart6, &uart6_cfg);
    memset(uart6.recv_buff, 0, uart6.recv_buff_size);
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

    BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 300, NULL, 3, &Chassic_control_handle);
    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
    BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 300, NULL, 4, &Planner_update_handle);
		  BaseType_t ok5 = xTaskCreate(GwGet_color_task, "GwGet_color", 200, NULL, 3, &Get_Color_handle);
    BaseType_t ok6 = xTaskCreate(LCD_Show_task, "LCD_Show_task", 400, NULL, 1, &LCD_Show_handle);
    BaseType_t ok8 = xTaskCreate(gray_read_task, "gray_read_task", 300, NULL, 2, &gray_read_handle);
    if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS||ok5!=pdPASS)
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
    Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 0.5, &debug_target_erro, clear_odom);
}
void GwGet_color_task(void *pvParameters)
{
	while(Ping_color())
	{
	    vTaskDelay(5);
	
	}
	
while(1)
{
	if(IIC_Get_RGB(RGB, 3))
	{
		goods_color_RGB=Get_GW_Color_RGB(RGB);
        current_color_RGB = goods_color_RGB;
	}
	if (IIC_Get_HSL(HSL, 3))
    {
		goods_color_HSL=Get_GW_Color_HSL(HSL);
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
        if ( IIC_Get_Anolog(Normal_side, 8, side))
        {
        }
        IIC_Anolog_Normalize(0xff, side);

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
        if( DEBUG_UPPER==1)
        {
            Servo_SetAngle(&servo[0], debug_angle[0], 270);
            Servo_SetAngle(&servo[1], debug_angle[1], 180);
            Servo_SetAngle(&servo[2], debug_angle[2], 360);
            HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, debug_isOpened);
        }
        DistributionLoop(servo, plate_things, current_color_ptr, upperflag_ptr, &CurrentColorLoop);
        PutGoal(color_task, servo, plate_things, upperflag_ptr, &PutGoalLoop);
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
        //vTaskDelay(100);
    }
}

void Onmaincpp(void *pvParameters)
{
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
                move_step_distance(0, -0.3, 0, 1);
                main_state++;
                break;
            }
        		case 1:
						{
								    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
								{
											   move_step_distance(0.6, 0, 0, 1);
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
				else 
				{
				setYawZero();
				}
//        if (DEBUG_CHASSIS == 1)
//        {
//            if (debug_speed == 1)
//            {
//                move_vel(debug_chassis_speed[0], debug_chassis_speed[1], debug_chassis_speed[2]);
//            }
//           
//        }
//        if (DEBUG_CHASSIS == 1)
//        {
//            if( debug_distance == 1)
//            {
//                if(debug_chassis_distance[0] != 0 || debug_chassis_distance[1] != 0 || debug_chassis_distance[2] != 0)
//                {
//                    safe_count = 1;
//                    move_step_distance(debug_chassis_distance[0], debug_chassis_distance[1], debug_chassis_distance[2], true);
//                    vTaskDelay(10000);
//                    safe_count = 0;
//                    debug_chassis_distance[0] = 0;
//                    debug_chassis_distance[1] = 0;
//                    debug_chassis_distance[2] = 0;
//                }
//                vTaskDelay(10000);
//            }
//        }
//        
//    if(chassisflag == LEAVE_HOME)
//    {
//        move_step_distance(0,-0.28,0, true); // 离开HOME
//        safe_count = 1;
//        // vTaskDelay(6000);
//        // safe_count = 0;
//        chassisflag = FIND_THING; // 状态机转移到寻找灰度
//    }

//    // if(chassisflag == FIND_GRAY)此处还缺灰度纠正的部分

//    if(chassisflag == FIND_THING)
//    {
//        move_step_distance(1,0,0, true); // 前往第一个物块处
//        // safe_count = 1;
//        // vTaskDelay(6000);
//        // safe_count = 0;
//        upperflag = PICKINGIN; // 状态机转移到拾取物块
//        chassisflag = WAITPICK; // 底盘状态机转移到等待拾取
//    }


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
	  vTaskDelay(1000);//等待一会
    uint16_t last_tick = xTaskGetTickCount();

    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
	if(safe_guard == 1)
	{
        Controller_KinematicAndControlUpdateWithYaw(ChassisControl_ptr, dt,main_yaw);
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
