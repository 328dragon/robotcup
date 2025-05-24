#ifndef __LCD_INIT_H
#define __LCD_INIT_H

//#include "sys.h"
#include "stdint.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

					 
#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 80
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 80
#endif



//-----------------LCD端口定义---------------- 

#define LCD_SCLK_Clr() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0)//SCL=SCLK
#define LCD_SCLK_Set() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0)//SDA=MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1)

#define LCD_RES_Clr()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1)
 		     
#define LCD_CS_Clr()   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,0)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,1)

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,0)//BLK
#define LCD_BLK_Set()  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,1)




void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序
void LCD_WR_DATA8(uint8_t dat);//写入一个字节
void LCD_WR_DATA(uint16_t dat);//写入两个字节
void LCD_WR_REG(uint8_t dat);//写入一个指令
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




