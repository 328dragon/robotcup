#include "ch040.h"
uint8_t ch040_origin_data[76] = {0};
float ch040_acc[3] = {0};
float ch040_gyr[3] = {0};
float ch040_mag[3] = {0};
float ch040_imu[3] = {0};
float ch040_quat[4] = {0};

float ch040_yaw = 0;
float yaw_raw = 0.0f;  // 用于存储原始yaw角度
float yaw_zero = 0.0f; // 用于存储零点yaw角度
__NOINLINE float ch040_get_data(uint8_t *data)
{
	float yaw = 0.0f; // 用于存储处理后的yaw角度

	uint8_t data_length = data[2];
	if (data_length == 76) // HI91浮点型数据输入
	{
		memcpy(ch040_acc, &data[18], 12);
		memcpy(ch040_gyr, &data[30], 12);
		memcpy(ch040_mag, &data[42], 12);
		memcpy(ch040_imu, &data[54], 12);
		memcpy(ch040_quat, &data[66], 16);
	}

	// 处理yaw角度.0到360度
	if (ch040_imu[2] < 0)
	{
		ch040_yaw = (ch040_imu[2]) + 360;
	}
	else if (ch040_imu[2] > 0)
	{
		ch040_yaw = ch040_imu[2];
	}

	// 处理yaw线转化成弧度
	yaw_raw = ch040_imu[2] * 3.1415926 / 180.0f;
	// 进行归一化
	if (yaw_raw < -3.1415926f)
	{
		yaw_raw += 2 * 3.1415926f;
	}
	else if (yaw_raw > 3.1415926f)
	{
		yaw_raw -= 2 * 3.1415926f;
	}

	yaw = yaw_raw - yaw_zero; // 减去零点偏移
	if (yaw < -3.1415926f)
	{
		yaw += 2 * 3.1415926f;
	}
	else if (yaw > 3.1415926f)
	{
		yaw -= 2 * 3.1415926f;
	}
	return yaw;
}
void setYawZero()
{
	yaw_zero = yaw_raw; // 设置当前yaw为零点
}