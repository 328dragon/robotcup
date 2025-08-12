#include "upper.h"
/*
    * @brief 获得颜色任务
*/
void GetColorTask(Color_t* color_task, int* color_task_index)
{
    // 创建颜色映射表，使用Color_t枚举
    static const Color_t colorMap[16][5] = 
    {
        /* 1  */ {COLOR_BLACK, COLOR_WHITE, COLOR_RED, COLOR_GREEN, COLOR_BLUE},
        /* 2  */ {COLOR_WHITE, COLOR_BLACK, COLOR_RED, COLOR_GREEN, COLOR_BLUE},
        /* 3  */ {COLOR_WHITE, COLOR_BLACK, COLOR_GREEN, COLOR_RED, COLOR_BLUE},
        /* 4  */ {COLOR_BLUE, COLOR_WHITE, COLOR_BLACK, COLOR_RED, COLOR_GREEN},
        /* 5  */ {COLOR_WHITE, COLOR_RED, COLOR_BLUE, COLOR_BLACK, COLOR_GREEN},
        /* 6  */ {COLOR_BLACK, COLOR_RED, COLOR_BLUE, COLOR_WHITE, COLOR_GREEN},
        /* 7  */ {COLOR_BLUE, COLOR_GREEN, COLOR_BLACK, COLOR_WHITE, COLOR_RED},
        /* 8  */ {COLOR_GREEN, COLOR_WHITE, COLOR_BLUE, COLOR_BLACK, COLOR_RED},
        /* 9  */ {COLOR_WHITE, COLOR_GREEN, COLOR_BLACK, COLOR_BLUE, COLOR_RED},
        /* 10 */ {COLOR_BLACK, COLOR_RED, COLOR_BLUE, COLOR_GREEN, COLOR_WHITE},
        /* 11 */ {COLOR_RED, COLOR_BLUE, COLOR_GREEN, COLOR_BLACK, COLOR_WHITE},
        /* 12 */ {COLOR_GREEN, COLOR_RED, COLOR_BLACK, COLOR_BLUE, COLOR_WHITE},
        /* 13 */ {COLOR_WHITE, COLOR_RED, COLOR_BLUE, COLOR_GREEN, COLOR_BLACK},
        /* 14 */ {COLOR_RED, COLOR_GREEN, COLOR_WHITE, COLOR_BLUE, COLOR_BLACK},
        /* 15 */ {COLOR_BLUE, COLOR_WHITE, COLOR_GREEN, COLOR_RED, COLOR_BLACK},
        /* 16 */ {COLOR_GREEN, COLOR_BLUE, COLOR_RED, COLOR_WHITE, COLOR_BLACK}
    };



    // 检查输入数字是否有效
    if (*color_task_index >= 1 && *color_task_index <= 16) 
    {
        // 将对应行的颜色复制到输出数组
        for (int i = 0; i < 5; i++) 
        {
            color_task[i] = colorMap[*color_task_index - 1][i];
        }
    } 
}

/*
    * @brief 绑定料盘槽的信息
*/
void DistributionLoop(Servo_t* servos,ThingStore_t* plate_things,Color_t* current_color_ptr, UpperTaskFlag* upperflag, int* CurrentColorLoop)
{
    if(*CurrentColorLoop<5)
    {
        if (*upperflag == PICKINGIN)
        {
			PUMP_ON;
            Servo_SetAngle(&servos[1], UP,180);
            Servo_SetAngle(&servos[0], PICK_LEFT,270);
			vTaskDelay(2000);
            Servo_SetAngle(&servos[1], PICK_DOWN,180);
						
						
			vTaskDelay(2000);
            Servo_SetAngle(&servos[1], UP,180);
			vTaskDelay(2000);
            Servo_SetAngle(&servos[0], COLORTASKHEIGHT,270);
            vTaskDelay(2000);
            *upperflag = GETCOLORIN;
        }
        if (*upperflag == GETCOLORIN)
        {
            plate_things[*CurrentColorLoop]._color = *current_color_ptr;
            plate_things[*CurrentColorLoop]._angle = THING_GIMBAL_ORIGIN_ANGLE + *CurrentColorLoop*THING_GIMBAL_FIXED_DELTA;
            if(plate_things[*CurrentColorLoop]._angle>360)
            {
                plate_things[*CurrentColorLoop]._angle -= 360;
            }
            plate_things[*CurrentColorLoop]._number = *CurrentColorLoop;
            Servo_SetAngle(&servos[2], plate_things[*CurrentColorLoop]._angle,360);
            (*CurrentColorLoop)++;
            *upperflag = PUTINGIN;
            vTaskDelay(2000);
        }
        if (*upperflag == PUTINGIN)
        {   
            Servo_SetAngle(&servos[0], UNMEANING_BIAS,270);
            vTaskDelay(2000);
            Servo_SetAngle(&servos[1], FUCK_LIMIT,180);
            Servo_SetAngle(&servos[0], FIND_PLATE,270);
            vTaskDelay(2000);
            // Servo_SetAngle(&servos[1], PUT_DOWN,180);
            // vTaskDelay(2000);

            PUMP_OFF;
            vTaskDelay(2000);
            Servo_SetAngle(&servos[1], UP,180);
            vTaskDelay(500);
            Servo_SetAngle(&servos[2], plate_things[*CurrentColorLoop-1]._angle+60,360);
            vTaskDelay(2000);
            Servo_SetAngle(&servos[2], plate_things[*CurrentColorLoop-1]._angle,360);
            *upperflag = IDLE;
        }
    }
}

void PutGoal(Color_t* color_task,Servo_t* servos,ThingStore_t* plate_things, UpperTaskFlag* upperflag,int* PutGoalLoop)
{
    if(*PutGoalLoop<=5)
    {   
        if (*upperflag == PICKINGOUT)
         // 按顺序筛选对应颜色任务的料盘
        {   
            Servo_SetAngle(&servos[1], UP,180);
            for (int i = 0; i < 5; i++) 
            {
                if (plate_things[i]._color == color_task[*PutGoalLoop])   
                {
                    Servo_SetAngle(&servos[2], plate_things[i]._angle,360);
                    vTaskDelay(2000);
                    (*PutGoalLoop)++;
                    break;
                }
            }
            Servo_SetAngle(&servos[0], FIND_PLATE,270);
            vTaskDelay(2000);
            Servo_SetAngle(&servos[1], PUT_DOWN,180);
            vTaskDelay(2000); // 等待舵机转动完成，需要实测

            PUMP_ON;
            vTaskDelay(2000);
            Servo_SetAngle(&servos[1], UP,180);
        }

        // 此处还需等待底盘移动到目标位置

        // if(*upperflag == PUTTINGOUT)
        // {
            vTaskDelay(2000);
            Servo_SetAngle(&servos[0], GOAL,270);
            vTaskDelay(700);
            Servo_SetAngle(&servos[1], PICK_DOWN,270);
            vTaskDelay(2000);
            PUMP_OFF;
            vTaskDelay(2000);

        // }
    }
}