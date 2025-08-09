#ifndef CH040_H
#define CH040_H
#include  "main.h"
#include "string.h"
extern uint8_t ch040_origin_data[76];
extern  float ch040_acc[3];
extern  float ch040_gyr[3];
extern float ch040_mag[3];
extern  float ch040_imu[3];
extern float ch040_quat[4] ;
float ch040_get_data(uint8_t *data);
void setYawZero();
extern float ch040_yaw;
#endif
