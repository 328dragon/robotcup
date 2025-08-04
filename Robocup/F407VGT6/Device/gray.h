#ifndef GRAY_H
#define GRAY_H
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "gw_grayscale_sensor.h"
enum gray_ordinal
{
front=0,
back=1,
};
typedef enum gray_state
{
    orgin_gray=0,
     all_black,//全黑
   primary_gray,
    senior_gray,
    undetermined_gray
   
} gray_state;
unsigned char Ping(void);
unsigned char IIC_Get_Digtal();
unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len);
unsigned char IIC_Get_Single_Anolog(unsigned char Channel);
unsigned char IIC_Anolog_Normalize(uint8_t Normalize_channel );
unsigned short IIC_Get_Offset( );
// unsigned char Ping(void);
// unsigned char IIC_Get_Digtal(void);
// unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len);
// unsigned char IIC_Get_Single_Anolog(unsigned char Channel);
// unsigned char IIC_Anolog_Normalize(uint8_t Normalize_channel);
// unsigned short IIC_Get_Offset(void );

#endif
