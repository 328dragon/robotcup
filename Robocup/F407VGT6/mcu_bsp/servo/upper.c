#include "upper.h"
#include "mainwork.h"
// 1980中间960前面

extern upper_location target_upper_loacation;
/*
 * @brief 获得颜色任务
 */
void GetColorTask(Color_t *color_task, int *color_task_index)
{
    // 创建颜色映射表，使用Color_t枚举
    static const Color_t colorMap[16][5] = {
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
        /* 16 */ {COLOR_GREEN, COLOR_BLUE, COLOR_RED, COLOR_WHITE, COLOR_BLACK}};

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
 * @brief 绑定料盘槽的信息并且抓取
 *dragon:只用一个舵机，一个抬升,servo[0]是云台舵机
 */
void DistributionLoop(Servo_t *servos, ThingStore_t *plate_things, Color_t *current_color_ptr, UpperTaskFlag *upperflag, int *CurrentColorLoop)
{
    plate_things[0]._color = 135;//黑
    plate_things[1]._color = 45;//白
    plate_things[2]._color = 90;
    plate_things[3]._color = 0;
    plate_things[4]._color = 180;
		plate_things[0]._angle=BLACK_PICK;
		plate_things[1]._angle=WHITE_PICK;
		plate_things[2]._angle=RED_PICK;
		plate_things[3]._angle=GREEN_PICK;
		plate_things[4]._angle=BLUE_PICK;
    if (*CurrentColorLoop <= 5)
    {
        if (*upperflag == PICKINGIN)
        {
            PUMP_ON;
            target_upper_loacation = up_location;
            vTaskDelay(2000);
            Servo_SetAngle(&servos[0], FIND_PLATE, 360); // 等待抓取
            vTaskDelay(500);                             // 等待舵机转动完成，需要实测
            target_upper_loacation = down_location;
            vTaskDelay(2000);

            target_upper_loacation = middle_location;
            vTaskDelay(1000);
            target_upper_loacation = up_location;
            vTaskDelay(1000);
            Servo_SetAngle(&servos[0], CENTER_PICK, 360); // 开始颜色识别
            vTaskDelay(2000);
            *upperflag = GETCOLORIN;
        }
        if (*upperflag == GETCOLORIN)
        {
            plate_things[*CurrentColorLoop]._number = *CurrentColorLoop;
            int color_angle = -1;
            switch (*current_color_ptr)
            {
            case 0:
            {
                color_angle = GREEN_PICK;
                break;
            }
            case 45:
            {

                color_angle = WHITE_PICK;
                break;
            }
            case 90:
            {
                color_angle = RED_PICK;
                break;
            }
            case 135:
            {
                color_angle = BLACK_PICK;
                break;
            }
            case 180:
            {
                color_angle = BLUE_PICK;
                break;
            }
            default:
                break;
            }
            Servo_SetAngle(&servos[0], color_angle, 360); // 放到对应颜色料盘正上方
            (*CurrentColorLoop)++;
            *upperflag = PUTINGIN;
        }
        if (*upperflag == PUTINGIN)
        {
            vTaskDelay(500);
            // 此处还需加入吸盘关闭
            target_upper_loacation = middle_location;
            vTaskDelay(500);
            PUMP_OFF;
            vTaskDelay(2000);
            target_upper_loacation = up_location; // 上升到中间防止冲突
					 vTaskDelay(4000);
					  Servo_SetAngle(&servos[0], FIND_PLATE, 360); // 等待抓取
            *upperflag = IDLE;
        }
    }
}
// 拿出去
void PutGoal(Color_t *color_task, Servo_t *servos, ThingStore_t *plate_things, UpperTaskFlag *upperflag, int *PutGoalLoop)
{
    if (*PutGoalLoop <= 5)
    {
        if (*upperflag == PICKINGOUT) // 将物块分拣到对应料盘
                                      //  按顺序筛选对应颜色任务的料盘
        {
            target_upper_loacation = up_location;
            vTaskDelay(200);
            for (int i = 0; i < 6; i++)
            {
                if (plate_things[i]._color == color_task[*PutGoalLoop])
                {
                    Servo_SetAngle(&servos[0], plate_things[i]._angle, 360);
                    break;
                }
            }
            vTaskDelay(500); // 等待舵机转动完成，需要实测
            target_upper_loacation = middle_location;
            vTaskDelay(500); // 等待舵机转动完成，需要实测
            // 此处还需加入吸盘启动
            PUMP_ON;
            vTaskDelay(200);
        }

        // 此处还需等待底盘移动到目标位置
        if (*upperflag == PUTTINGOUT) // 放置物块到目标位置
        {
            target_upper_loacation = up_location;
            vTaskDelay(200);
            Servo_SetAngle(&servos[0], GOAL_PLACE, 360);
            vTaskDelay(500); // 等待舵机转动完成，需要实测
            target_upper_loacation = down_location;
            vTaskDelay(200);
            // 此处还需加入吸盘关闭
            PUMP_OFF;
            vTaskDelay(500);
        }
    }
}