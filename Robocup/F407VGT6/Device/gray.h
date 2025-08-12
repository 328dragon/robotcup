#ifndef GRAY_H
#define GRAY_H
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "gw_grayscale_sensor.h"
enum gray_ordinal
{
front=0,
back=1,
	side=2
};
typedef enum gray_state
{
    orgin_gray=0,
     all_black,//全黑
aim_black=2,
   
} gray_state;
unsigned char Ping(void);
unsigned char IIC_Get_Digtal(int ordinal);
unsigned char IIC_Get_Anolog(unsigned char *Result, unsigned char len, int ordinal);
unsigned char IIC_Get_Single_Anolog(unsigned char Channel, int ordinal);
unsigned char IIC_Anolog_Normalize(uint8_t Normalize_channel, int ordinal);
unsigned short IIC_Get_Offset(int ordinal);

#endif