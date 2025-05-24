#include "crc8.h"

uint8_t _lut[256];
// 获取CRC8校验余数
void Crc8_init(uint8_t poly)
{
    for (int idx = 0; idx < 256; ++idx)
    {
        uint8_t crc = idx;
        for (int shift = 0; shift < 8; ++shift)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        }
        _lut[idx] = crc & 0xff;
    }
}

// CRC8处理，得到计算的CRC
uint8_t Crc8_calc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc = _lut[crc ^ *data++];
    }
    return crc;
}
