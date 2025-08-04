#ifndef __TCD1103_H
#define __TCD1103_H
#include "main.h"
#include "adc.h"
#include "i2c.h"
void Dly_us(void);
extern uint16_t ccd_rawdata[1546]; // 储存CCD接收的原始数据?
extern uint16_t ccd_data[128];     // 储存平均后的CCD数据
extern uint16_t avg, _max, _min;


extern uint32_t icg_flag, stop_line_flag;
extern uint8_t l, r;
extern uint8_t l_w, r_w;
void FindLines(uint8_t *l, uint8_t *r, uint16_t *data, uint16_t threshold, uint8_t *l_w, uint8_t *r_w);

#endif
