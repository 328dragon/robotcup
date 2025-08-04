#ifndef __TCS230_H
#define __TCS230_H

#include <stdio.h>
#include "main.h"
#include "usart.h"

#define white_value 45
#define blue_value 180
#define green_value 0
#define red_value 90
#define black_value 135
extern char tcs_string[40];
extern  uint8_t R , G , B ;
extern uint8_t RX_Flag;
extern uint8_t RxData;
extern uint32_t red, blue, green;


enum color_enum
{
green_color =0,
white_color=45,
red_color=90,
black_color=135,
blue_color=180
};


extern uint8_t USARTData;//数据缓冲
extern uint8_t USARTDataBag[6];//数据存储数组

uint8_t Color_Recognize(void);
int Get_Color(void);
uint8_t Get_RxFlag(void);


#endif
