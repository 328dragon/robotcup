#ifndef SOFT_PWM_H
#define SOFT_PWM_H
#include <stdint.h>

void SoftPwmTimerISR(void);
void SoftPwmSetPeriod(uint16_t period);
void SoftPwmSetHigh(uint16_t high);

#endif
