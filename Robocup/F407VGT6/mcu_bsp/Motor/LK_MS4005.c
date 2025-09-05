/// @file LK_MS4005.c
/// @brief LK_MS4005电机驱动库
/// @author Dragon
/// 手册中好用的就这几种：
/// 读取命令：“3. 读取电机状态 2 命令”、“18. 读取控制参数命令”、“20. 读取电机编码器数据命令”、“22. 读取多圈角度命令”、“23. 读取单圈角度命令”
// 硬件设定命令："5. 电机关闭命令"、“6. 电机运行命令”、“7. 电机停止命令”、“8. 抱闸器控制和状态读取命令”、“21. 设置当前位置到 ROM 作为电机零点命令”、“24. 设置当前位置为任意角度（写入 RAM）”
/// 控制命令：“10. 转矩闭环控制命令（该命令仅在 MF、MH、MG 电机上实现）”、"11. 速度闭环控制命令"
/// “13. 多圈位置闭环控制命令 2”、“15单圈位置闭环控制命令 2”、“17增量位置闭环控制命令 2”、“19. 写入控制参数命令”

/// 建议使用，或者说重要的指令：“3. 读取电机状态 2 命令”、所有硬件控制设定命令、"11. 速度闭环控制命令"、“13. 多圈位置闭环控制命令 2”、“17增量位置闭环控制命令 2”
/// “19. 写入控制参数命令”、“20. 读取电机编码器数据命令”、“22. 读取多圈角度命令”

#include "LK_MS4005.h"

#define DEVICE_STD_ID (0x140)           // 命令报文标识符：DEVICE_STD_ID+ ID(1~32)
#define DEVICE_STD_RV_ID (0x180)        // 回复报文标识符：DEVICE_STD_RV_ID + ID(1~32)
#define DEVICE_STD_BOARDCAST_ID (0x280) // 广播报文标识符

static LK_MS4005_Controller_t *LK_MS4005_instnce[LK_MS4005_NUM] = {NULL}; // dm_j4310实例数组
static int LK_ms4005_idx = 0;                                             // dm_j4310实例索引,每次有新的模块注册会自增

static int32_t float_to_int32(float value, float mini_quantum)
{
    if (mini_quantum != 0)
    {
        float scaled_value = value / mini_quantum;
        int32_t return_int32_t_data = (int32_t)(scaled_value + 0.5f);
        return return_int32_t_data;
    }
    else
    {
        return 0;
    }
}
static int16_t float_to_int16(float value, float mini_quantum)
{
    if (mini_quantum != 0)
    {
        float scaled_value = value / mini_quantum;
        int16_t return_int16_t_data = (int16_t)(scaled_value + 0.5f);
        return return_int16_t_data;
    }
    else
    {
        return 0;
    }
}

/// @brief 电机汇报信息解析
/// @param can_instance
void LK_MS4005_Get_Info(CANInstance *can_instance)
{

    uint8_t data[8] = {0};
    memcpy(data, can_instance->rx_buff, 8);

    //***这里的id是指向那个电机实例的指针***//
    LK_MS4005_Controller_t *_instance = (LK_MS4005_Controller_t *)(can_instance->id);
    // _instance->motor_instnce.get.deg_pos = position;
    // _instance->motor_instnce.get.velocity = vel_raw;
}

/// @brief 注册LK_MS4005电机
/// @param hcan
/// @param protocol_id
/// @param mst_id
/// @param w_mode
/// @return
LK_MS4005_Controller_t *LK_MS4005_Register(CAN_HandleTypeDef *hcan, uint32_t protocol_id, uint16_t w_mode)
{
    LK_MS4005_Controller_t *LK_MS4005_s = (LK_MS4005_Controller_t *)malloc(sizeof(LK_MS4005_Controller_t));
    memset(LK_MS4005_s, 0, sizeof(LK_MS4005_Controller_t));
    LK_MS4005_s->wroking_mode = w_mode;
    LK_MS4005_s->protocol_id = DEVICE_STD_ID + protocol_id;
    // 瓴控电机的can发送id和协议id是一样的，不会因为模式改变而改变发送的id
    LK_MS4005_s->mode_trans_id = LK_MS4005_s->protocol_id;

    CAN_Init_Config_s can_instance_config = {0};
    can_instance_config.can_handle = hcan;
    can_instance_config.tx_id = LK_MS4005_s->mode_trans_id;
    can_instance_config.rx_id = protocol_id + DEVICE_STD_RV_ID;   // 返回帧id
    can_instance_config.can_module_callback = LK_MS4005_Get_Info; // 这里可以设置回调函数,但是目前没有用到
    can_instance_config.id = LK_MS4005_s;

    LK_MS4005_s->can_instance = CANRegister(&can_instance_config); // 注册CAN实例
    LK_MS4005_instnce[LK_ms4005_idx++] = LK_MS4005_s;

    return LK_MS4005_s; // 返回电机实例指针
}

/// @brief 改变LK_MS4005工作模式
/// @param lk_ms4005_instance
/// @param target_mode
static void LK_MS4005_Change_Mode(LK_MS4005_Controller_t *lk_ms4005_instance, LK_MS4005_Working_MODE_e target_mode)
{
    lk_ms4005_instance->wroking_mode = target_mode; // 设置新的工作模式
}

/// @brief 使能LK_MS4005电机:“6. 电机运行命令”
/// @param lk_ms4005_instance
void Enable_LK(LK_MS4005_Controller_t *lk_ms4005_instance)
{
    uint8_t motor_data[8] = {0}; // 发送数据缓存
    motor_data[0] = 0x88;
    motor_data[1] = 0x00;
    motor_data[2] = 0x00;
    motor_data[3] = 0x00;                                             // 使能电机
    motor_data[4] = 0x00;                                             // 使能电机
    motor_data[5] = 0x00;                                             // 使能电机
    motor_data[6] = 0x00;                                             // 使能电机
    motor_data[7] = 0x00;                                             // 使能电机
    memcpy(lk_ms4005_instance->can_instance->tx_buff, motor_data, 8); // 将数据拷贝到CAN实例的发送缓存中
    CANTransmit(lk_ms4005_instance->can_instance);                    // 发送数据
}

/// @brief 停止控制电机，但不清除运行状态："5. 电机关闭命令"
/// @param lk_ms4005_instance
void Disable_LK(LK_MS4005_Controller_t *lk_ms4005_instance)
{
    uint8_t motor_data[8] = {0}; // 发送数据缓存
    motor_data[0] = 0x81;
    motor_data[1] = 0x00;
    motor_data[2] = 0x00;
    motor_data[3] = 0x00;                                             // 关闭电机
    motor_data[4] = 0x00;                                             // 关闭电机
    motor_data[5] = 0x00;                                             // 关闭电机
    motor_data[6] = 0x00;                                             // 关闭电机
    motor_data[7] = 0x00;                                             // 关闭电机
    memcpy(lk_ms4005_instance->can_instance->tx_buff, motor_data, 8); // 将数据拷贝到CAN实例的发送缓存中
    CANTransmit(lk_ms4005_instance->can_instance);                    // 发送数据
}
/// @brief
/// @param lk_ms4005_instance
/// @param stop_way 0x00：抱闸器断电，刹车启动 0x01：抱闸器通电，刹车释放 0x10：读取抱闸器状态
void STOP_LK(LK_MS4005_Controller_t *lk_ms4005_instance, uint8_t stop_way)
{
    uint8_t motor_data[8] = {0}; // 发送数据缓存
    motor_data[0] = 0x8C;
    motor_data[1] = stop_way;
    motor_data[2] = 0x00;
    motor_data[3] = 0x00;                                             // 停止电机
    motor_data[4] = 0x00;                                             // 停止电机
    motor_data[5] = 0x00;                                             // 停止电机
    motor_data[6] = 0x00;                                             // 停止电机
    motor_data[7] = 0x00;                                             // 停止电机
    memcpy(lk_ms4005_instance->can_instance->tx_buff, motor_data, 8); // 将数据拷贝到CAN实例的发送缓存中
    CANTransmit(lk_ms4005_instance->can_instance);                    // 发送数据
}

/// @brief
/// @param lk_ms4005_instance
/// @param iqControl 单位:A -16.5A~16.5A
/// @param target_Angular_velocity  目标单位：rad/s
void Vel_Ctrl_LK(LK_MS4005_Controller_t *lk_ms4005_instance, float iqControl, float target_Angular_velocity)
{
    LK_MS4005_Change_Mode(lk_ms4005_instance, vel_close_loop_mode);
    // 给电流赋值，但不是实际发送信息
    lk_ms4005_instance->motor_instnce.set.velocity = target_Angular_velocity;
    // 实际发送信息
    lk_ms4005_instance->hardware_t.speedCtrl_trans = float_to_int32(lk_ms4005_instance->motor_instnce.set.velocity, 0.01f);
    lk_ms4005_instance->hardware_t.iqCtrl_trans=(int16_t)remap_limit(-16.5f,16.5f,-2048,2048,iqControl);

}

/// 循环调用，实际发送命令
///  @brief 控制LK_MS4005电机
///  @param lk_ms4005_instance
void Control_LK_MS4005(LK_MS4005_Controller_t *lk_ms4005_instance)
{
    int trans_flag = 0;
    uint8_t motor_data[8] = {0}; // 发送数据缓存
//临时变量
    int32_t speedCtrl_=0;
    int16_t iqCtrl_=0;
    int32_t  angle_Ctrl_=0;
    uint16_t maxspeed_=0;
    int32_t angleIncrement_=0;

    switch (lk_ms4005_instance->wroking_mode)
    {
    case vel_close_loop_mode: // 速度闭环式
    {
        motor_data[0] = 0xA2;
        motor_data[1] = 0x00;
        speedCtrl_=lk_ms4005_instance->hardware_t.speedCtrl_trans;
        iqCtrl_=lk_ms4005_instance->hardware_t.iqCtrl_trans;
        motor_data[2] = (uint8_t)(iqCtrl_);
        motor_data[3] = (uint8_t)(iqCtrl_>>8);
        motor_data[4] = (uint8_t)(speedCtrl_);//低八位
        motor_data[5] = (uint8_t)(speedCtrl_>>8);
        motor_data[6] = (uint8_t)(speedCtrl_>>16);
        motor_data[7] = (uint8_t)(speedCtrl_>>24);
        trans_flag = 1;
        break;
    }
    case multiple_position_close_loop_mode: // 多圈位置闭环式
    {
        motor_data[0] = 0xA4;
        motor_data[1] = 0x00;
        
        trans_flag = 1;
        break;
    }
    case incremental_position_close_loop_mode: // 增量位置闭环式
    {
        motor_data[0] = 0xA8;
        motor_data[1] = 0x00;
        trans_flag = 1;
        break;
    }

    default:
        break;
    }

    if (trans_flag == 1)
    {
        memcpy(lk_ms4005_instance->can_instance->tx_buff, motor_data, 8); // 将数据拷贝到CAN实例的发送缓存中
        CANTransmit(lk_ms4005_instance->can_instance);                    // 发送数据
    }
}
