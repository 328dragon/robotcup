#include "tcd1103.h"

uint16_t ccd_rawdata[1546]; // 储存CCD接收的原始数�???????????
uint16_t ccd_data[128];     // 储存平均后的CCD数据
uint32_t icg_flag, stop_line_flag;
uint8_t l, r;
uint8_t l_w, r_w;
uint16_t avg, _max, _min;

 //延时
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);      
}


// 128点找左右边界
void FindLines(uint8_t *l, uint8_t *r, uint16_t *data, uint16_t threshold, uint8_t *l_w, uint8_t *r_w)
{
    *r = *l = 128;
    for (int i = 0; i < 128 - 6; i++)
    {
        if (data[i + 6] - data[i] > threshold)
        {
            for (int j = i + 1; j < 128 - 6; j++)
                if (data[j] - data[j + 6] > threshold)
                {
                    *l = (i + j + 6) / 2, *l_w = j + 6 - i;
                    break;
                }
            if (*l == 128)
                *l = (i + 127) / 2, *l_w = 127 - i;
            break;
        }
        if(data[i] - data[i + 6] > threshold)
        {
            *l = i  / 2, *l_w = i;
            break;
        }
    }
		
		
	
    for (int i = 127; i >= 0 + 6; i--)
    {
        if (data[i - 6] - data[i] > threshold)
        {
            for (int j = i - 1; j >= 6; j--)
                if (data[j] - data[j - 6] > threshold)
                {
                    *r = (i + j - 6) / 2, *r_w = i - j + 6;
                    break;
                }
            if (*r == 128)
                *r = i / 2, *r_w = i;
            break;
        }
        if(data[i] - data[i - 6] > threshold)
        {
            *r = (i + 127) / 2, *l_w = 127 - i;
            break;
        }
    }
}