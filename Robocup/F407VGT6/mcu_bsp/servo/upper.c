#include "upper.h"
/*
    * @brief 获得颜色任务
*/
void GetColorTask(Color_t* color_task, int* color_task_index)
{
    // 创建颜色映射表，使用Color_t枚举,以放置顺序BADCE来排序
    static const Color_t colorMap[16][5] = 
    {
        /* 1  */ {COLOR_WHITE, COLOR_BLACK, COLOR_GREEN, COLOR_RED, COLOR_BLUE},
        /* 2  */ {COLOR_BLACK, COLOR_WHITE, COLOR_GREEN, COLOR_RED, COLOR_BLUE},
        /* 3  */ {COLOR_BLACK, COLOR_WHITE, COLOR_RED, COLOR_GREEN, COLOR_BLUE},
        /* 4  */ {COLOR_WHITE, COLOR_BLUE, COLOR_RED, COLOR_BLACK, COLOR_GREEN},
        /* 5  */ {COLOR_RED, COLOR_WHITE, COLOR_BLACK, COLOR_BLUE, COLOR_GREEN},
        /* 6  */ {COLOR_RED, COLOR_BLACK, COLOR_WHITE, COLOR_BLUE, COLOR_GREEN},
        /* 7  */ {COLOR_GREEN, COLOR_BLUE, COLOR_WHITE, COLOR_BLACK, COLOR_RED},
        /* 8  */ {COLOR_WHITE, COLOR_GREEN, COLOR_BLACK, COLOR_BLUE, COLOR_RED},
        /* 9  */ {COLOR_GREEN, COLOR_WHITE, COLOR_BLUE, COLOR_BLACK, COLOR_RED},
        /* 10 */ {COLOR_RED, COLOR_BLACK, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE},
        /* 11 */ {COLOR_BLUE, COLOR_RED, COLOR_BLACK, COLOR_GREEN, COLOR_WHITE},
        /* 12 */ {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_BLACK, COLOR_WHITE},
        /* 13 */ {COLOR_RED, COLOR_WHITE, COLOR_GREEN, COLOR_BLUE, COLOR_BLACK},
        /* 14 */ {COLOR_GREEN, COLOR_RED, COLOR_BLUE, COLOR_WHITE, COLOR_BLACK},
        /* 15 */ {COLOR_WHITE, COLOR_BLUE, COLOR_RED, COLOR_GREEN, COLOR_BLACK},
        /* 16 */ {COLOR_BLUE, COLOR_GREEN, COLOR_WHITE, COLOR_RED, COLOR_BLACK}
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
            VALVE_OFF;
            Servo_SetAngle(&servos[1], UP,180);
            Servo_SetAngle(&servos[0], PICK_LEFT,270);
			vTaskDelay(2000);
            Servo_SetAngle(&servos[1], PICK_DOWN+16,180);
						
						
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

            if (*current_color_ptr == 0xFF)
            plate_things[*CurrentColorLoop]._color = COLOR_GREEN;

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

            VALVE_ON;
            PUMP_OFF;

            vTaskDelay(5000);
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
    if(*PutGoalLoop<5)
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
            Servo_SetAngle(&servos[1], PUT_DOWN+5,180);
            vTaskDelay(2000); // 等待舵机转动完成，需要实测

            PUMP_ON;
            VALVE_OFF;
            vTaskDelay(2000);
            Servo_SetAngle(&servos[1], UP,180);
			*upperflag = PUTTINGOUT;
        }

        // 此处还需等待底盘移动到目标位置

        if(*upperflag == PUTTINGOUT)
        {
            vTaskDelay(2000);
            Servo_SetAngle(&servos[0], GOAL,270);
            vTaskDelay(700);
            Servo_SetAngle(&servos[1], PICK_DOWN-8,180);
            vTaskDelay(2000);
            
            VALVE_ON;
            PUMP_OFF;
            

            vTaskDelay(4000);
            Servo_SetAngle(&servos[1], UP,180);
            vTaskDelay(2000);

            *upperflag = IDLE;

        }
    }
}

/*
    * @brief 获取任务二的物块排名数组
*/
void GetRankTask(Rank_t* rank_task, int* rank_task_index)
{
    // 创建排序映射表，使用Rank_t枚举，按照3，2，1顺序来获取物块
    static const Rank_t Rank_map[6][3] = 
    {
        /* 1  */ {BRONZE, SILVER, GOLD},
        /* 2  */ {SILVER, BRONZE, GOLD},
        /* 3  */ {BRONZE, GOLD, SILVER},
        /* 4  */ {GOLD, BRONZE, SILVER},
        /* 5  */ {SILVER, GOLD, BRONZE},
        /* 6  */ {GOLD, SILVER, BRONZE},
    };



    // 检查输入数字是否有效
    if (*rank_task_index >= 1 && *rank_task_index <= 6) 
    {
        // 将对应行的颜色复制到输出数组
        for (int i = 0; i < 3; i++) 
        {
            rank_task[i] = Rank_map[*rank_task_index - 1][i];
        }
    } 
}

/*
    * @brief 抓取任务二物块的单步函数
*/
void DistributionRankLoop(Rank_t* rank_task,Servo_t* servos,ThingStore_Task2_t* plate_task2_things, UpperTaskFlag* upperflag,int* CurrentRankLoop)
{
    if(*CurrentRankLoop<3)
    {
        if (*upperflag == PICKINGIN)
        {
			PUMP_ON;
            VALVE_OFF;
            Servo_SetAngle(&servos[1], UP,180);
            Servo_SetAngle(&servos[0], PICK_LEFT,270);
			vTaskDelay(2000);
            Servo_SetAngle(&servos[1], PICK_DOWN+16,180);
						
						
			vTaskDelay(2000);
            Servo_SetAngle(&servos[1], UP,180);
			vTaskDelay(2000);

            plate_task2_things[*CurrentRankLoop]._rank = rank_task[*CurrentRankLoop];
            plate_task2_things[*CurrentRankLoop]._angle = THING_GIMBAL_ORIGIN_ANGLE + *CurrentRankLoop*THING_GIMBAL_FIXED_DELTA;

            if(plate_task2_things[*CurrentRankLoop]._angle>360)
            {
                plate_task2_things[*CurrentRankLoop]._angle -= 360;
            }
            plate_task2_things[*CurrentRankLoop]._number = *CurrentRankLoop;
            Servo_SetAngle(&servos[2], plate_task2_things[*CurrentRankLoop]._angle,360);

            (*CurrentRankLoop)++;
            *upperflag = PUTINGIN;
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

            VALVE_ON;
            PUMP_OFF;

            vTaskDelay(5000);
            Servo_SetAngle(&servos[1], UP,180);
            vTaskDelay(500);
            Servo_SetAngle(&servos[2], plate_task2_things[*CurrentRankLoop-1]._angle+60,360);
            vTaskDelay(2000);
            Servo_SetAngle(&servos[2], plate_task2_things[*CurrentRankLoop-1]._angle,360);
            *upperflag = IDLE;
        }
    }
}

/*

*/
void PutRank(Servo_t* servos, ThingStore_Task2_t* plate_task2_things, UpperTaskFlag* upperflag, int* PutRankLoop)
{
    if (*PutRankLoop<3)
    {
        if (*upperflag == PICKINGOUT)
        {
            Servo_SetAngle(&servos[1], UP,180);
            Servo_SetAngle(&servos[2], plate_task2_things[*PutRankLoop]._angle,360);

            Servo_SetAngle(&servos[0], FIND_PLATE,270);
            vTaskDelay(4000);
            Servo_SetAngle(&servos[1], PUT_DOWN+5,180);
            vTaskDelay(2000); // 等待舵机转动完成，需要实测

            PUMP_ON;
            VALVE_OFF;
            vTaskDelay(2000);
            Servo_SetAngle(&servos[1], UP,180);
			*upperflag = PUTTINGOUT;
        }
        // 此处等到底盘走到目标点
        if (*upperflag == PUTTINGOUT)
        {
            vTaskDelay(2000);
            Servo_SetAngle(&servos[0], GOAL,270);
            vTaskDelay(700);
            Servo_SetAngle(&servos[1], PICK_DOWN-8,180);
            vTaskDelay(2000);
            
            VALVE_ON;
            PUMP_OFF;
            

            vTaskDelay(4000);
            Servo_SetAngle(&servos[1], UP,180);
            vTaskDelay(2000);
            (*PutRankLoop)++;

            *upperflag = IDLE;
        }
    }
}