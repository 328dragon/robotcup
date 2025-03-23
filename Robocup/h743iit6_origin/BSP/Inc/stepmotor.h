#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#include "usart.h"
#include "main.h"
#include "stdbool.h"
 #define		CMD_LEN		255
extern bool rxFrameFlag;
 extern uint8_t rxCmd[CMD_LEN];
extern  uint8_t rxCount ;
void Step_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Step_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Step_Synchronous_motion(uint8_t addr);


#endif
