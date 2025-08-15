#include "upper.h"
#include "mainwork.h"
#include "usart.h"
// 1980中间960前面
extern upper_location now_upper_loacation;
extern upper_location target_upper_loacation;
/*
 * @brief 获得颜色任务
 */
void GetColorTask(Color_t *color_task, int *color_task_index)
{
    // 创建颜色映射表，使用Color_t枚举：bacde
    static const Color_t colorMap[16][5] = {
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
 * @brief 绑定料盘槽的信息并且抓取
 *dragon:只用一个舵机，一个抬升,servo[0]是云台舵机
 */
void DistributionLoop(Servo_t *servos, ThingStore_t *plate_things, Color_t *current_color_ptr, UpperTaskFlag *upperflag, int *CurrentColorLoop)
{

    if (*CurrentColorLoop <= 5)
    {
        if (*upperflag == PICKINGIN)
        {
            PUMP_ON;
            target_upper_loacation = up_location;
            vTaskDelay(1000);
            Servo_SetAngle(&servos[0], FIND_PLATE, 360); // 等待抓取
            vTaskDelay(1000);                            // 等待舵机转动完成，需要实测
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
            vTaskDelay(3000);
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
            PUMP_ON;
            vTaskDelay(1000);
            for (int i = 0; i < 6; i++)
            {
                if (plate_things[i]._color == color_task[*PutGoalLoop]) // 找对应放置任务的颜色
                {
                    Servo_SetAngle(&servos[0], plate_things[i]._angle, 360);
                    (*PutGoalLoop)++;
                    break;
                }
            }
            vTaskDelay(1400);
            target_upper_loacation = pick_middle_location;
            *upperflag = PUTTINGOUT;
        }

        // 此处还需等待底盘移动到目标位置
        if (*upperflag == PUTTINGOUT) // 放置物块到目标位置
        {
            vTaskDelay(1000);
            target_upper_loacation = up_location;
            vTaskDelay(2000);
            Servo_SetAngle(&servos[0], GOAL_PLACE, 360);
            vTaskDelay(1000); // 等待舵机转动完成，需要实测
            target_upper_loacation = down_put_lcoation;
            vTaskDelay(500);
            // 此处还需加入吸盘关闭
            PUMP_OFF;
            vTaskDelay(1500);
            target_upper_loacation = up_location;
            *upperflag = IDLE;
            vTaskDelay(1000);
        }
    }
}

/*
 * @brief 获得任务二摆放顺序
 */
void Get_ABC_Task( ThingStore_t *plate_things, int *ranking_task_index)
{
    // 创建名次映射表，使用Ranking_t枚举
    ////顺序3,2,1/////
 static const Ranking_t ABC_ranking_Map[6][3] = {
            /* 1  */ {Bronze_medal, Runner_up, Champion},
            /* 2  */ {Runner_up, Bronze_medal, Champion},
            /* 3  */ {Bronze_medal, Champion, Runner_up},
            /* 4  */ {Champion, Bronze_medal, Runner_up},
            /* 5  */ {Runner_up, Champion, Bronze_medal},
            /* 6  */ {Champion, Runner_up, Bronze_medal},
        };

    // 检查输入数字是否有效
    if (*ranking_task_index >= 1 && *ranking_task_index <= 6)
    {
        // 将对应行的颜色复制到输出数组
        for (int i = 2; i < 5; i++)
        {
            plate_things[i]._rank = ABC_ranking_Map[*ranking_task_index - 1][i-2];
        }
    }
}

/*
 * @brief 绑定料盘槽的信息并且抓取
 *dragon:只用一个舵机，一个抬升,servo[0]是云台舵机
 */
void Distribution_ABC(Servo_t *servos, ThingStore_t *plate_things, UpperTaskFlag *upperflag, int *CurrentrankingLoop)
{

    if (*CurrentrankingLoop <= 3)
    {
        if (*upperflag == PICKINGABC)
        {
            PUMP_ON;
            target_upper_loacation = up_location;
            vTaskDelay(1000);
            Servo_SetAngle(&servos[0], FIND_PLATE, 360); // 等待抓取
            vTaskDelay(1000);                            // 等待舵机转动完成，需要实测
            target_upper_loacation = down_location;
            vTaskDelay(2000);
            target_upper_loacation = middle_location;
            vTaskDelay(1000);
            target_upper_loacation = up_location;
            vTaskDelay(1000);
            *upperflag = GETABCIN;
        }
        if (*upperflag == GETABCIN)
        {
            int ranking_angle = -1;
            switch (*CurrentrankingLoop)
            {
            case 0:
            {
                ranking_angle = THIRD_PLACE;
                break;
            }
            case 1:
            {
                ranking_angle = SECOND_PLACE;
                break;
            }
            case 2:
            {
                ranking_angle = ONCE_PLACE;

                break;
            }
            default:
                break;
            }
            Servo_SetAngle(&servos[0], ranking_angle, 360); // 放到对应任务料盘正上方
            (*CurrentrankingLoop)++;
            *upperflag = PUTINGINABC;
        }
        if (*upperflag == PUTINGINABC)
        {
            vTaskDelay(1000);
            // 此处还需加入吸盘关闭
            target_upper_loacation = middle_location;
            vTaskDelay(500);
            PUMP_OFF;
            vTaskDelay(2000);
            target_upper_loacation = up_location; // 上升到上面防止冲突
            vTaskDelay(3000);
            Servo_SetAngle(&servos[0], FIND_PLATE, 360); // 等待抓取
            *upperflag = IDLE;
        }
    }
}

// 拿出去
void PutABCGoal(Ranking_t *ranking_task, Servo_t *servos, ThingStore_t *plate_things, UpperTaskFlag *upperflag, int *PutGoalLoop)
{
    if (*PutGoalLoop <= 3)
    {
        if (*upperflag == PICKINGABCOUT) // 将物块分拣到对应料盘
                                      //  按顺序筛选对应颜色任务的料盘
        {
            target_upper_loacation = up_location;
            PUMP_ON;
            vTaskDelay(1000);
            for (int i = 2; i < 5; i++)
            {
                if (plate_things[i]._rank == ranking_task[*PutGoalLoop]) // 找对应放置任务的名次
                {
                    Servo_SetAngle(&servos[0], plate_things[i]._angle, 360);
                    (*PutGoalLoop)++;
                    break;
                }
            }
            vTaskDelay(1400);
            target_upper_loacation = pick_middle_location;
            *upperflag = PUTTINGABCOUT;
        }

        // 此处还需等待底盘移动到目标位置
        if (*upperflag == PUTTINGABCOUT) // 放置物块到目标位置
        {
            vTaskDelay(1000);
            target_upper_loacation = up_location;
            vTaskDelay(2000);
            Servo_SetAngle(&servos[0], GOAL_PLACE, 360);
            vTaskDelay(1000); // 等待舵机转动完成，需要实测
            target_upper_loacation = down_put_lcoation;
            vTaskDelay(500);
            // 此处还需加入吸盘关闭
            PUMP_OFF;
            vTaskDelay(1000);
            target_upper_loacation = up_location;
            *upperflag = IDLE;
            vTaskDelay(1000);
        }
    }
}

void upper_move_distance(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                 // 地址
    cmd[1] = 0xFD;                 // 功能码
    cmd[2] = dir;                  // 方向
    cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
    cmd[5] = acc;                  // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )

    cmd[10] = raF;  // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;  // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B; // 校验字节

    // 发送命令
    HAL_UART_Transmit(&huart3, (uint8_t *)cmd, 13, 1000);
    vTaskDelay(10);
}
static void upper_move_location(upper_location now_location, upper_location target_position)
{
    // origin-- down--dowm_put_pulse--pick_middle--middle--up
    int origin_pulse = 0;
    int down_pulse = 200;
    int dowm_put_pulse = 500;
    int pick_middle_pulse = 4600;
    int middle_pulse = 5000;
    int up_pulse = 7800;
    switch (now_location)
    {
    case down_location:
    {
        if (target_position == middle_location)
        {
            upper_move_distance(5, 0, 300, 0.02, middle_pulse - down_pulse, 0, 0); // 上升到中间位置
            now_upper_loacation = middle_location;
        }
        else if (target_position == up_location)
        {
            upper_move_distance(5, 0, 300, 0.02, up_pulse - down_pulse, 0, 0); // 上升到最高位置
            now_upper_loacation = up_location;
        }
        else if (target_position == pick_middle_location)
        {
            upper_move_distance(5, 0, 300, 0.02, pick_middle_pulse - down_pulse, 0, 0); // 上升到分拣位置
            now_upper_loacation = pick_middle_location;
        }
        else if (target_position == down_put_lcoation)
        {
            upper_move_distance(5, 0, 300, 0.02, dowm_put_pulse - down_pulse, 0, 0); // 上升到放置位置
            now_upper_loacation = down_put_lcoation;
        }

        break;
    }

    case down_put_lcoation:
    {
        if (target_position == down_location)
        {
            upper_move_distance(5, 1, 300, 0.02, dowm_put_pulse - down_pulse, 0, 0); // 降落到最低位置
            now_upper_loacation = down_location;
        }
        else if (target_position == middle_location)
        {
            upper_move_distance(5, 0, 300, 0.02, middle_location - dowm_put_pulse, 0, 0); // 上升到中间位置
            now_upper_loacation = middle_location;
        }
        else if (target_position == up_location)
        {
            upper_move_distance(5, 0, 300, 0.02, up_pulse - dowm_put_pulse, 0, 0); // 上升到最高位置
            now_upper_loacation = up_location;
        }
        if (target_position == pick_middle_location)
        {
            upper_move_distance(5, 0, 300, 0.02, pick_middle_pulse - dowm_put_pulse, 0, 0); // 上升到分拣位置
            now_upper_loacation = pick_middle_location;
        }

        break;
    }

    case pick_middle_location:
    {
        if (target_position == down_location)
        {
            upper_move_distance(5, 1, 300, 0.02, pick_middle_pulse - down_pulse, 0, 0); // 降落到最低位置
            now_upper_loacation = down_location;
        }
        else if (target_position == middle_location)
        {
            upper_move_distance(5, 0, 300, 0.02, middle_pulse - pick_middle_pulse, 0, 0); // 上升到中间位置
            now_upper_loacation = middle_location;
        }
        else if (target_position == up_location)
        {
            upper_move_distance(5, 0, 300, 0.02, up_pulse - pick_middle_pulse, 0, 0); // 上升到最高位置
            now_upper_loacation = up_location;
        }
        else if (target_position == down_put_lcoation)
        {
            upper_move_distance(5, 1, 300, 0.02, pick_middle_pulse - dowm_put_pulse, 0, 0); // 下降到放置位置
            now_upper_loacation = down_put_lcoation;
        }

        break;
    }

    case middle_location:
    {
        if (target_position == down_location)
        {
            upper_move_distance(5, 1, 300, 0.02, middle_pulse - down_pulse, 0, 0); // 降落到最低位置
            now_upper_loacation = down_location;
        }
        else if (target_position == up_location)
        {
            upper_move_distance(5, 0, 300, 0.02, up_pulse - middle_pulse, 0, 0); // 上升到最高位置
            now_upper_loacation = up_location;
        }
        else if (target_position == pick_middle_location)
        {
            upper_move_distance(5, 1, 300, 0.02, middle_pulse - pick_middle_pulse, 0, 0); // 下降到分拣位置
            now_upper_loacation = pick_middle_location;
        }
        else if (target_position == down_put_lcoation)
        {
            upper_move_distance(5, 1, 300, 0.02, middle_pulse - dowm_put_pulse, 0, 0); // 下降到放置位置
            now_upper_loacation = down_put_lcoation;
        }
        break;
    }
    case up_location:
    {
        if (target_position == down_location)
        {
            upper_move_distance(5, 1, 300, 0.02, up_pulse - down_pulse, 0, 0); // 降落到最低位置
            now_upper_loacation = down_location;
        }
        else if (target_position == middle_location)
        {
            upper_move_distance(5, 1, 300, 0.02, up_pulse - middle_pulse, 0, 0); // 降落到中间位置
            now_upper_loacation = middle_location;
        }
        else if (target_position == pick_middle_location)
        {
            upper_move_distance(5, 1, 300, 0.02, up_pulse - pick_middle_pulse, 0, 0); // 降落到分拣位置
            now_upper_loacation = pick_middle_location;
        }
        else if (target_position == down_put_lcoation)
        {
            upper_move_distance(5, 1, 300, 0.02, up_pulse - dowm_put_pulse, 0, 0); // 下降到放置位置
            now_upper_loacation = down_put_lcoation;
        }
        break;
    }

    default:
        break;
    }
}

void upper_to_target(upper_location target_position)
{
    upper_move_location(now_upper_loacation, target_position);
}