#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "gw_color_sensor.h"
enum gw_color_enum
{
gw_green_color =0,
gw_white_color=45,
gw_red_color=90,
gw_black_color=135,
gw_blue_color=180
};
unsigned char Ping_color(void);
unsigned char IIC_Get_HSL(unsigned char * Result,unsigned char len);
unsigned char IIC_Get_RGB(unsigned char * Result,unsigned char len);
int Get_GW_Color(unsigned char *RGB);

