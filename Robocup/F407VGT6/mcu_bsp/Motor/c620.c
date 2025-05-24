/*
 * @Author: skybase
 * @Date: 2025-04-08 18:40:05
 * @LastEditors: skybase
 * @LastEditTime: 2025-04-09 10:02:40
 * @Description:  ᕕ(◠ڼ◠)ᕗ​
 * @FilePath: \2025_RC_JUMP\mcu_bsp\Motor\c620.c
 */
#include "C620.h"

// !C620的ID从1开始
C620_group_Controller_t *C620_Group_instnce[C620_GROUP_NUM] = {NULL}; // c620实例数组
static int idx = 0;                                                   // 由于C620的id从1开始的特性,故而实例从下标1开始,共有第几组3508电机

void C620_Group_Get_Info(CANInstance *can_instance)
{
    int16_t _deg_pos = (can_instance->rx_buff[0] << 8) + can_instance->rx_buff[1];
    int16_t _volocity = ((can_instance->rx_buff[2] << 8) + can_instance->rx_buff[3]);
    int16_t _current = ((can_instance->rx_buff[4] << 8) + can_instance->rx_buff[5]);

    if (can_instance->rx_id >= 0x201 && can_instance->rx_id <= 0x204)
    {
        int flag = ((C620_group_Controller_t *)(can_instance->id))->motor_instnce[can_instance->rx_id - 0x201].motor_cofig.motor_reverse_flag;

        if (flag == MOTOR_DIRECTION_REVERSE)
        {
            _volocity = -_volocity;
            _current = -_current;
        }

        ((C620_group_Controller_t *)(can_instance->id))->motor_instnce[can_instance->rx_id - 0x201].get.deg_pos = (float)_deg_pos * 6.28 / 8191.0;
        ((C620_group_Controller_t *)(can_instance->id))->motor_instnce[can_instance->rx_id - 0x201].get.velocity = (float)_volocity * 0.1047;
        ((C620_group_Controller_t *)(can_instance->id))->motor_instnce[can_instance->rx_id - 0x201].get.current = (float)_current * 0.0012207;
    }
    else if (can_instance->rx_id >= 0x205 && can_instance->rx_id <= 0x208)
    {
        int flag = ((C620_group_Controller_t *)(can_instance->id))->motor_instnce[can_instance->rx_id - 0x205].motor_cofig.motor_reverse_flag;
        if (flag == MOTOR_DIRECTION_REVERSE)
        {
            _volocity = -_volocity;
            _current = -_current;
        }

        ((C620_group_Controller_t *)(can_instance->id))->motor_instnce[can_instance->rx_id - 0x205].get.deg_pos = _deg_pos * 6.28 / 8191.0;
        ((C620_group_Controller_t *)(can_instance->id))->motor_instnce[can_instance->rx_id - 0x205].get.velocity = _volocity * 0.1047;
        ((C620_group_Controller_t *)(can_instance->id))->motor_instnce[can_instance->rx_id - 0x205].get.current = _current * 0.0012207;
    }
}

/**
 * @brief
 *
 * @param hcan ,表示can1还是can2
 * @param group_id, 表示can1的第几组c620电机
 * @param protocol_id 0x200 or 0x1FF
 */
void C620_Group_Register(CAN_HandleTypeDef *hcan, uint8_t group_id, uint32_t protocol_id)
{
    int i;

    C620_group_Controller_t *C620_group_s = (C620_group_Controller_t *)malloc(sizeof(C620_group_Controller_t));
    memset(C620_group_s, 0, sizeof(C620_group_Controller_t));
    
    C620_group_s->group_id = group_id;
    C620_group_s->protocol_id = protocol_id;
    CAN_Init_Config_s can_instance_config = {0};
    can_instance_config.can_handle = hcan;
    can_instance_config.tx_id = protocol_id;
    can_instance_config.can_module_callback = C620_Group_Get_Info;
    can_instance_config.id = C620_group_s;

    // 四个电机注册
    for (i = 0; i <= 3; i++)
    {
        can_instance_config.rx_id = (protocol_id == 0x200) ? 0x201 + i : 0x205 + i;
        C620_group_s->can_instance[i] = CANRegister(&can_instance_config);
    }

    C620_Group_instnce[idx++] = C620_group_s;
}

static void C620_Group_base_current(C620_group_Controller_t *c620_group)
{
    int i;
    for (i = 0; i <= 3; i++)
    {
        if (MOTOR_STOP == c620_group->motor_instnce[i].motor_cofig.motor_enable_flag)
        {
            c620_group->motor_instnce[i].set.current = 0;
        }
        else
        {
            if (MOTOR_DIRECTION_REVERSE == c620_group->motor_instnce[i].motor_cofig.motor_reverse_flag)
            {
                MOTOR_VAR_REVERSE(c620_group->motor_instnce[i].set.current);
            }
        }
    }
}

void C620_Group_Set_Current(C620_group_Controller_t *c620_group)
{
    C620_Group_base_current(c620_group);
    uint8_t motor_current_data[8] = {0};

    for (int i = 0; i <= 7; i += 2)
    {
        if (c620_group->motor_instnce[i / 2].motor_cofig.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        {
            motor_current_data[i] = (int16_t)(-(c620_group->motor_instnce[i / 2].set.current * 819.2)) >> 8;
            motor_current_data[i + 1] = (int16_t)(-(c620_group->motor_instnce[i / 2].set.current * 819.2));
        }
        else
        {
            motor_current_data[i] = (int16_t)(c620_group->motor_instnce[i / 2].set.current * 819.2) >> 8;
            motor_current_data[i + 1] = (int16_t)(c620_group->motor_instnce[i / 2].set.current * 819.2);
        }
    }

    memcpy(c620_group->can_instance[0]->tx_buff, motor_current_data, 8);
    CANTransmit(c620_group->can_instance[0]);
}
