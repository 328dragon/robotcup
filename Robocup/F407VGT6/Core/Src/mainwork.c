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
#define get_little_yellow_state			HAL_GPIO_ReadPin(little_yellow_GPIO_Port,little_yellow_Pin)

Servo_t servo[1]= {
    {&htim3, TIM_CHANNEL_4, 83, 0}
};
int pick_goods_flag=0;
UpperTaskFlag upperflag = IDLE; // 上层机构状态机
UpperTaskFlag* upperflag_ptr = &upperflag;
ThingStore_t plate_things[5] = {0}; // 料盘槽数组
Color_t current_color = COLOR_BLACK; // 当前颜色
Color_t* current_color_ptr = &current_color;
int CurrentColorLoop = 0;
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
int get_yellow_flag=0;
int yellow_state=0;
//上升控制
int first_upper_flag=1;
upper_location now_upper_loacation=0;
upper_location target_upper_loacation=0;
int upper_flag=0;
int upper_rotate_pwm=960;
int pump_flag=0;
//读陀螺仪
void usart6_callfront(void)
{
    if (uart6.recv_buff[0] == 0x5A && uart6.recv_buff[1] == 0xA5)
    {
        ch040_get_data(uart6.recv_buff);
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
    memset(uart6.recv_buff, 0, uart6.recv_buff_size);
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
    if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS||ok5!=pdPASS)
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
	while(Ping_color())
	{
	vTaskDelay(5);
	
	}
	
while(1)
{

	if (IIC_Get_HSL(HSL, 3))
        {
					goods_color_HSL=Get_GW_Color_HSL(HSL);
					*current_color_ptr=goods_color_HSL;
        }
vTaskDelay(200);

}

}
void UPPER_control_task(void *pvParameters)
{

while (1)
{
if(pick_goods_flag==1)
{
*upperflag_ptr=PICKINGIN;
pick_goods_flag=0;
}
    
DistributionLoop(servo,plate_things,current_color_ptr, upperflag_ptr, &CurrentColorLoop);
PutGoal()

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
	if(get_yellow_flag)
	{
	yellow_state=get_little_yellow_state;
	}
	else 
	{
	yellow_state=-1;
	}

	
        // 读取灰度传感器数据
        Digtal_gray_front = IIC_Get_Digtal(front);
        Digtal_gray_side = IIC_Get_Digtal(side);
        for (int i = 0; i < 8; i++)
        {
            digital_gray_data_front[i] = 1 - ((Digtal_gray_front >> i) & 0x01); // 读取后边数字灰度传感器数据
            digital_gray_data_side[i] = 1 - ((Digtal_gray_side >> i) & 0x01);   // 读取侧边数字灰度传感器数据
        }

        // 获取传感器模拟量结果
        if (IIC_Get_Anolog(Anolog_gray_front, 8, front) && IIC_Get_Anolog(Anolog_gray_side, 8, side))
        {
        }

        // 获取传感器归一化结果
        IIC_Anolog_Normalize(0xff, front); // 所有通道归一化都打开
        IIC_Anolog_Normalize(0xff, side);  // 所有通道归一化都打开
        vTaskDelay(10);                    // 设置完，需要等上一会。stm8的运算速度没stm32快，等一下，让传感器把数据刷新一下。
        if (IIC_Get_Anolog(Normal_front, 8, front) && IIC_Get_Anolog(Normal_front, 8, side))
        {
        }
        IIC_Anolog_Normalize(0xff, front); // 为了下一次循环是非归一化，所以清零
        IIC_Anolog_Normalize(0xff, side);
        if (digital_gray_data_front[0] == 1 && digital_gray_data_front[1] == 1 && digital_gray_data_front[2] == 1 && digital_gray_data_front[3] == 1 && digital_gray_data_front[4] == 1 && digital_gray_data_front[5] == 1 && digital_gray_data_front[6] == 1 && digital_gray_data_front[7] == 1)
        {

            real_time_gray_state = all_black;
        }
        else
        {
            real_time_gray_state = orgin_gray;
        }

        if (digital_gray_data_side[1] == 1 && digital_gray_data_side[2] == 1 && digital_gray_data_side[3] == 1 && digital_gray_data_side[4] == 1 && digital_gray_data_side[5] == 1 && digital_gray_data_side[6] == 1)
        {

            real_time_gray_state_side = all_black;
            BUZZER_ON;
        }
        else
        {
            real_time_gray_state_side = orgin_gray;
            BUZZER_OFF;
        }

        for (int i = 0; i < 8; i++)
        {
            gray_data_front_middle_temp += digital_gray_data_front[i] * sensor_weights_front[i] * gray_front_p; // 计算前面灰度传感器的中间值
            gray_data_side_middle_temp += digital_gray_data_side[i] * sensor_weights_side[i] * gray_side_p;
        }
        gray_data_side_middle = gray_data_side_middle_temp;
        gray_data_front_middle = gray_data_front_middle_temp;
        gray_data_front_middle_temp = 0;
        gray_data_side_middle_temp = 0;

        vTaskDelay(10); // 延时10ms
    }
}
void LCD_Show_task(void *pvParameters)
{
    // 屏幕
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
    DistributionLoop(servo, plate_things, current_color_ptr, upperflag_ptr, &CurrentColorLoop);
    while (1)
    {   
       
        //        // 显示
        //        // 陀螺仪
        //        LCD_ShowFloatNum1(0, 20, gyro[0], 4, RED, WHITE, 16);
        //        LCD_ShowString(48, 20, ",", RED, WHITE, 16, 0);
        //        LCD_ShowFloatNum1(58, 20, gyro[1], 4, RED, WHITE, 16);
        //        LCD_ShowString(106, 40, ",", RED, WHITE, 16, 0);
              LCD_ShowFloatNum1(0, 20, HSL[0], 8, RED, WHITE, 16);
							LCD_ShowFloatNum1(0, 40, goods_color_HSL, 8, RED, WHITE, 16);
							LCD_ShowFloatNum1(0, 60, HSL[2], 8, RED, WHITE, 16);
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
        vTaskDelay(100);
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
//            switch (main_state)
//            {

//            case 0:
//            {

//                motor_mode = 1;
//                debug_target_odom = (odom_t){0.3, 0, 0};
//                debug_target_erro = (odom_t){0.01, 0.01, 0.01};
//                Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 0.5, &debug_target_erro, 1);
//                main_state++;

//                break;
//            }

//            case 1:
//            {
//                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
//                {
//                    motor_mode = 0;
//                    debug_target_vel = (cmd_vel_t){0, 0.2, 0};
//                    if (real_time_gray_state_side == all_black)
//                    {
//                        debug_target_vel = (cmd_vel_t){0, 0, 0};
//                        main_state++;
//                    }
//                    break;
//                }
//            }
//            case 2:
//            {


//                break;
//            }
//            case 4:
//            {
//                if (SimpleStatus_t_isResolved(&planner_ptr->promise))
//                {
//                    motor_mode = 0;
//                }

//                break;
//            }
//            default:
//                break;
//            }

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

static void upper_move_distance(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
	
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(&huart3, (uint8_t *)cmd, 13,1000);
	vTaskDelay(10);
}
static void upper_move_location(upper_location now_location,upper_location target_position )
{
//origin-- down--pick_middle--middle--up
int origin_pulse=0;
int down_pulse=300;
int pick_middle_pulse=4400;
int middle_pulse=5000;
int up_pulse=7800;
    switch (now_location)
    {
    case down_location:
        if (target_position == middle_location)
        {
            upper_move_distance(5, 0, 300, 0.02, middle_pulse-down_pulse, 0, 0); // 上升到中间位置
            now_upper_loacation = middle_location;
        }
        else if (target_position == up_location)
        {
            upper_move_distance(5, 0, 300, 0.02, up_pulse-down_pulse, 0, 0); // 上升到最高位置
            now_upper_loacation = up_location;
        }else if(target_position==pick_middle_location)
        {
					upper_move_distance(5, 0, 300, 0.02, pick_middle_pulse-down_pulse, 0, 0); // 上升到分拣位置
            now_upper_loacation = pick_middle_location;
        }
        break;

        case pick_middle_location:
        {
            if(target_position=down_location)
            {
                upper_move_distance(5, 1, 300, 0.02, pick_middle_pulse-down_pulse, 0, 0); // 降落到最低位置
                now_upper_loacation = down_location;
            }
            else if(target_position==middle_location)
            {
							upper_move_distance(5, 0, 300, 0.02, middle_pulse-pick_middle_pulse, 0, 0); // 上升到中间位置
                now_upper_loacation = middle_location;
            }else if(target_position==up_location)
            {
							upper_move_distance(5, 0, 300, 0.02, up_pulse-pick_middle_pulse, 0, 0); // 上升到最高位置
                now_upper_loacation = up_location;
            }

            break;
        }


    case middle_location:
    {
        if (target_position == down_location)
        {
            upper_move_distance(5, 1, 300, 0.02, middle_pulse-down_pulse, 0, 0); // 降落到最低位置
            now_upper_loacation = down_location;
        }
        else if (target_position == up_location)
        {
            upper_move_distance(5, 0, 300, 0.02, up_pulse-middle_pulse, 0, 0); // 上升到最高位置
            now_upper_loacation = up_location;
        }else if(target_position == pick_middle_location)
        {
            upper_move_distance(5, 1, 300, 0.02, middle_pulse-pick_middle_pulse, 0, 0); // 下降到分拣位置
            now_upper_loacation = middle_location;
        }
        break;

    }


    case up_location:
    {
        if (target_position == down_location)
        {
            upper_move_distance(5, 1, 300, 0.02, up_pulse-down_pulse, 0, 0); // 降落到最低位置
            now_upper_loacation = down_location;
        }
        else if (target_position == middle_location)
        {
            upper_move_distance(5, 1, 300, 0.02, up_pulse-middle_pulse, 0, 0); // 降落到中间位置
            now_upper_loacation = middle_location;
        }else if(target_position==pick_middle_location)
        {
            upper_move_distance(5, 1, 300, 0.02, up_pulse-pick_middle_pulse, 0, 0); // 降落到分拣位置
            now_upper_loacation = pick_middle_location;
        }
        break;

    }

    
    default:
        break;
    }
}

static void upper_to_target(upper_location target_position)
{
upper_move_location(now_upper_loacation,target_position);
}

// 底盘更新任务,包括执行层
void OnChassicControl(void *pvParameters)
{
	int safe_upper_count=0;
    uint16_t last_tick = xTaskGetTickCount();

    while (1)
    {
			safe_upper_count++;
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
        if (safe_guard)
        {
					if(safe_upper_count>=20&&first_upper_flag==1)
					{
					upper_move_distance(5, 0, 300, 0.02, 300, 0, 0); // 上升到中间位置
						first_upper_flag=0;
					}
					 
					upper_to_target(target_upper_loacation);		
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
