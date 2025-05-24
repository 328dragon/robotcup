/*
 * @Author: skybase
 * @Date: 2025-01-13 17:12:42
 * @LastEditors: skybase
 * @LastEditTime: 2025-04-09 00:39:39
 * @Description:  ᕕ(◠ڼ◠)ᕗ​
 * @FilePath: \2025_RC_JUMP\mcu_bsp\log\bsp_log.c
 */
#include "bsp_log.h"

char _debug[1024];

DISPLAY_MODE dis_mode = DISPLAY_MODE_DEFAULT;       // 显示模式(默认)
FOREGROUND_COLOR fwd_clor = FOREGROUND_COLOR_BLACK; // 字体色（黑色）
BACKGROUND_COLOR bak_clor = BACKGROUND_COLOR_WHITE; // 背景色（白色）

/* USER CODE BEGIN  */
#include "bsp_usart.h"
#include "usart.h"

extern USARTInstance uart6;

/* USER CODE END  */

/**
 * @brief 用户自定义的串口发送
 *
 * @param data
 * @param len
 */
void debug_transmit(uint8_t *data, uint16_t len)
{
  USARTSend(&uart6, data, len, USART_TRANSFER_BLOCKING);
}

// void Float2Str(char *str, float va)
// {
//     int flag = va < 0;
//     int head = (int)va;
//     int point = (int)((va - head) * 1000);
//     head = abs(head);
//     point = abs(point);
//     if (flag)
//         sprintf(str, "-%d.%d", head, point);
//     else
//         sprintf(str, "%d.%d", head, point);
// }
