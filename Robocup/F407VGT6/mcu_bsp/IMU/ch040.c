#include "ch040.h"
uint8_t ch040_origin_data[76] = {0};
float ch040_acc[3] = {0};
float ch040_gyr[3] = {0};
float ch040_mag[3] = {0};
float ch040_imu[3] = {0};
float ch040_quat[4] = {0};

float ch040_yaw = 0;

__NOINLINE void ch040_get_data(uint8_t *data)
{
	uint8_t data_length = data[2];
	if (data_length == 76) // HI91浮点型数据输入
	{
		memcpy(ch040_acc, &data[18], 12);
		memcpy(ch040_gyr, &data[30], 12);
		memcpy(ch040_mag, &data[42], 12);
		memcpy(ch040_imu, &data[54], 12);
		memcpy(ch040_quat, &data[66], 16);
	}

	// 处理yaw角度
	if (ch040_imu[2] < 0)
	{
		ch040_yaw = (ch040_imu[2]) + 360;
	}
	else if (ch040_imu[2] > 0)
	{
		ch040_yaw = ch040_imu[2];
	}
}