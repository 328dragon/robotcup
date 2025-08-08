#include "upper.h"
/*
    * @brief 获得颜色任务
*/
void GetColorTask(Color_t* color_task, int* color_task_index)
{
    // 创建颜色映射表，使用Color_t枚举
    static const Color_t colorMap[16][5] = {
        /* 1  */ {BLACK, WHITE, RED, GREEN, BLUE},
        /* 2  */ {WHITE, BLACK, RED, GREEN, BLUE},
        /* 3  */ {WHITE, BLACK, GREEN, RED, BLUE},
        /* 4  */ {BLUE, WHITE, BLACK, RED, GREEN},
        /* 5  */ {WHITE, RED, BLUE, BLACK, GREEN},
        /* 6  */ {BLACK, RED, BLUE, WHITE, GREEN},
        /* 7  */ {BLUE, GREEN, BLACK, WHITE, RED},
        /* 8  */ {GREEN, WHITE, BLUE, BLACK, RED},
        /* 9  */ {WHITE, GREEN, BLACK, BLUE, RED},
        /* 10 */ {BLACK, RED, BLUE, GREEN, WHITE},
        /* 11 */ {RED, BLUE, GREEN, BLACK, WHITE},
        /* 12 */ {GREEN, RED, BLACK, BLUE, WHITE},
        /* 13 */ {WHITE, RED, BLUE, GREEN, BLACK},
        /* 14 */ {RED, GREEN, WHITE, BLUE, BLACK},
        /* 15 */ {BLUE, WHITE, GREEN, RED, BLACK},
        /* 16 */ {GREEN, BLUE, RED, WHITE, BLACK}
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
    if(*CurrentColorLoop<=5)
        if (*upperflag == PICKINGIN)
        {
            Servo_SetAngle(&servos[0], PICK_LEFT);
            Servo_SetAngle(&servos[1], PICK_DOWN);
            vTaskDelay(1000); // 等待舵机转动完成，需要实测

            // 此处还需加入吸盘启动

            vTaskDelay(500);
            Servo_SetAngle(&servos[0], FIND_PLATE);
            Servo_SetAngle(&servos[1], UP);
            vTaskDelay(500);
            Servo_SetAngle(&servos[1], COLORTASKHEIGHT);
            vTaskDelay(500);
            *upperflag = GETCOLORIN;
        }
        if (*upperflag == GETCOLORIN)
        {
            plate_things[*CurrentColorLoop]._color = *current_color_ptr;
            plate_things[*CurrentColorLoop]._angle = THING_GIMBAL_ORIGIN_ANGLE + *CurrentColorLoop*THING_GIMBAL_FIXED_DELTA;
            plate_things[*CurrentColorLoop]._number = *CurrentColorLoop;
            (*CurrentColorLoop)++;
            *upperflag = PUTINGIN;
        }
        if (*upperflag == PUTINGIN)
        {
            Servo_SetAngle(&servos[1], PUT_DOWN);
            vTaskDelay(500);

            // 此处还需加入吸盘关闭

            *upperflag = IDLE;
        }
}

void PutGoal(Color_t* color_task,Servo_t* servos,ThingStore_t* plate_things, UpperTaskFlag* upperflag,int* PutGoalLoop)
{
    if(*PutGoalLoop<=5)
    {   
        if (*upperflag == PICKINGOUT)
         // 按顺序筛选对应颜色任务的料盘
        {
            for (int i = 0; i < 6; i++) 
            {
                if (plate_things[i]._color == color_task[*PutGoalLoop])   
                {
                    Servo_SetAngle(&servos[2], plate_things[i]._angle);
                }
            }
            Servo_SetAngle(&servos[0], FIND_PLATE);
            Servo_SetAngle(&servos[1], PUT_DOWN);
            vTaskDelay(1000); // 等待舵机转动完成，需要实测

            // 此处还需加入吸盘启动

            Servo_SetAngle(&servos[1], UP);
        }

        // 此处还需等待底盘移动到目标位置

        if(*upperflag == PUTTINGOUT)
        {
            Servo_SetAngle(&servos[0], GOAL);

            // 此处还需加入吸盘关闭

            vTaskDelay(500);
        }

    }
}