#include "ZDTstepmotor.h"
 
#include "math.h"
/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
static uint8_t Step_Pos_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    // uint8_t cmd[16] = {0};

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
    cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                // 校验字节

    // 返回命令长度
    return (uint8_t)13;
}

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
static uint8_t Step_Vel_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    // uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                // 地址
    cmd[1] = 0xF6;                // 功能码
    cmd[2] = dir;                 // 方向
    cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
    cmd[5] = acc;                 // 加速度，注意：0是直接启动
    cmd[6] = snF;                 // 多机同步运动标志
    cmd[7] = 0x6B;                // 校验字节

    // 发送命令
    return (uint8_t)8;
}

/**
 * @brief    多机同步运动
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
static uint8_t Step_Synchronous_motion(uint8_t *cmd, uint8_t addr)
{
    // uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0xFF; // 功能码
    cmd[2] = 0x66; // 辅助码
    cmd[3] = 0x6B; // 校验字节
    return (4);
}

void set_speed_target(StepMotorZDT_t* zdt_motor,float target)
{
  zdt_motor->motor_controller_t.set.velocity = target;
  zdt_motor->_target_rpm = (int16_t)(target * 60 / (3.14 *( zdt_motor->_wheel_diameter))); // 转化为转速(RPM)
    // 发送数据到电机
    uint8_t len;
    if (zdt_motor->_target_rpm  > 0)
    {
        len = Step_Vel_Control(zdt_motor->_cmd_buffer,  zdt_motor->motor_controller_t.id_protocol, zdt_motor->_dir, (uint16_t)(zdt_motor->_target_rpm), 0, false);
    }
    else
    { // 反转
        int dir_trans = zdt_motor->_dir == 0 ? 1 : 0;
        len = Step_Vel_Control(zdt_motor->_cmd_buffer, zdt_motor->motor_controller_t.id_protocol, dir_trans, (uint16_t)(-zdt_motor->_target_rpm), 0, false);
    }
    HAL_UART_Transmit(zdt_motor->_USART, zdt_motor->_cmd_buffer, len, 1000); // 发送数据到电机
    HAL_Delay(1);                                          // 傻逼电机需要延迟避免重包
    if (zdt_motor->_have_pub_permission)
    {
        // 发布同步信号
        len = Step_Synchronous_motion(zdt_motor->_cmd_buffer, 0);     // 发送数据到电机
        HAL_UART_Transmit(zdt_motor->_USART, zdt_motor->_cmd_buffer, len, 1000); // 发送数据到电机
        HAL_Delay(1);
    }
}



float get_linear_speed(StepMotorZDT_t* zdt_motor)
{
    // 返回电机的线速度
    // return _target_speed * 3.14 * _wheel_diameter / 60; // 转化为米每秒
    return zdt_motor->_target_rpm * 3.14 * zdt_motor->_wheel_diameter / 60; // 转化为米每秒
}