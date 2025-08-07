#include "StepMotor.h"
#include <stdlib.h>

#define abs(x) ((x) > 0 ? (x) : -(x))

static void StepMotor_getIterationData(StepMotor_t *motor, float *rpm_in, uint32_t *pulse_in) ;
void StepMotor_givePulse(StepMotor_t *motor, uint32_t pulse, uint32_t freq);
void StepMotor_giveOncePulse(StepMotor_t *motor, uint32_t pulse, uint32_t freq, bool clearbuffer);

void StepMotor_Init(StepMotor_t *motor, TIM_HandleTypeDef *tim, uint32_t channel, GPIO_TypeDef *ph_port, uint16_t ph_pin) {
    motor->_tim = tim;
    motor->_ph_port = ph_port;
    motor->_ph_pin = ph_pin;
    motor->_channel = channel;
    motor->_clock_base_frequency = 1000000;
    motor->_resolution = 10;
    motor->_itertration_num = ITERATIONS;
    motor->_iteration_pulse = ITERATIONPULSE;
    motor->_state = IDLE;
    __HAL_TIM_SetAutoreload(motor->_tim, motor->_resolution - 1);
}

void StepMotor_setParam(StepMotor_t *motor, float StepAngle, uint8_t Subdivision, uint16_t target_iteration_num, uint16_t itertion_pulse) {
    motor->_StepAngle = StepAngle;
    motor->_Subdivision = Subdivision;
    motor->_itertration_num = target_iteration_num;
    motor->_iteration_pulse = itertion_pulse;
}

bool StepMotor_isBusy(StepMotor_t *motor) {
    return !(motor->_state == IDLE);
}

void StepMotor_update(StepMotor_t *motor, uint16_t dt) {
    switch (motor->_state) {
        case BUSY:
            return;
        case IDLE:
            return;
        case Start:
            motor->_state = WaitIteration;
            break;
        case WaitIteration:
            if (motor->_iteration == 2 * motor->_itertration_num) {
                motor->_iteration = 0;
                motor->_state = IDLE;
            } else {
                float temp_rpm;
                uint32_t temp_pulse;
                StepMotor_getIterationData(motor, &temp_rpm, &temp_pulse);
                StepMotor_giveRPMPulse(motor, temp_rpm, temp_pulse);
                motor->_iteration++;
            }
            break;
        default:
            break;
    }
}

void StepMotor_giveRPMPulseSoft(StepMotor_t *motor, float rpm, uint32_t pulse, uint16_t target_iteration_num, uint16_t itertion_pulse) {
    if (target_iteration_num != 0 && itertion_pulse != 0) {
        motor->_itertration_num = target_iteration_num;
        motor->_iteration_pulse = itertion_pulse;
    }
    motor->_soft_target_pulse = pulse;
    motor->_soft_target_rpm = rpm;

    if (motor->_state != IDLE || pulse == 0) {
        return;
    } else {
        motor->_state = Start;
    }
}

void StepMotor_giveRPMPulse(StepMotor_t *motor, float rpm, uint32_t pulse) {
    uint32_t freq = motor->_Subdivision * abs(rpm) * (360 / motor->_StepAngle);
    if (pulse == 0) {
        return;
    }
    
    if (rpm > 0) {
        HAL_GPIO_WritePin(motor->_ph_port, motor->_ph_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(motor->_ph_port, motor->_ph_pin, GPIO_PIN_RESET);
    }
    StepMotor_givePulse(motor, pulse, freq);
}

void StepMotor_giveRPMAngle(StepMotor_t *motor, float rpm, float angle) {
    uint32_t freq = motor->_Subdivision * abs(rpm) * (360 / motor->_StepAngle);
    uint32_t pulse = motor->_Subdivision * angle / motor->_StepAngle;
    
    if (rpm > 0) {
        HAL_GPIO_WritePin(motor->_ph_port, motor->_ph_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(motor->_ph_port, motor->_ph_pin, GPIO_PIN_RESET);
    }
    StepMotor_givePulse(motor, pulse, freq);
}

void StepMotor_givePulse(StepMotor_t *motor, uint32_t pulse, uint32_t freq) {
    uint32_t period = motor->_clock_base_frequency / freq / motor->_resolution;
    uint8_t arr = motor->_resolution / 2;
    
    motor->_target_pulse = BUFFER_SIZE / period > pulse ? pulse : BUFFER_SIZE / period;
    motor->_target_number = pulse / motor->_target_pulse + ((pulse % motor->_target_pulse == 0) ? 0 : 1);
    motor->_pulse_mod = pulse - motor->_target_pulse * (motor->_target_number - 1);
    if (motor->_target_number == 1) {
        motor->_pulse_mod = 0;
    }
    motor->_target_freq = freq;
    StepMotor_giveOncePulse(motor, motor->_target_pulse, motor->_target_freq, true);
    motor->_state = BUSY;
}

void StepMotor_giveOncePulse(StepMotor_t *motor, uint32_t pulse, uint32_t freq, bool clearbuffer) {
    uint32_t period = motor->_clock_base_frequency / freq / motor->_resolution;
    uint8_t ccr = motor->_resolution / 2;
    uint16_t max_address = pulse * period;
    
    if (max_address > BUFFER_SIZE) {
        return;
    }
    
    if (clearbuffer) {
        memset(motor->_buffer, 0, BUFFER_SIZE);
        for (uint16_t i = 0; i < pulse; i += 1) {
            uint16_t j = i * period;
            motor->_buffer[j] = ccr;
        }
    }

    HAL_TIM_PWM_Start_DMA(motor->_tim, motor->_channel, (uint32_t *)motor->_buffer, max_address);
}

void StepMotor_dmaCallBack(StepMotor_t *motor, TIM_HandleTypeDef *htim) {
    if (htim->Instance != motor->_tim->Instance) {
        return;
    }
    
    if (motor->_target_number > 2) {
        StepMotor_giveOncePulse(motor, motor->_target_pulse, motor->_target_freq, false);
        motor->_target_number--;
    } else if (motor->_target_number == 2 && motor->_pulse_mod > 0) {
        StepMotor_giveOncePulse(motor, motor->_pulse_mod, motor->_target_freq, true);
        motor->_target_number--;
    } else {
        HAL_TIM_PWM_Stop_DMA(motor->_tim, motor->_channel);
        motor->_state = WaitIteration;
    }
}

uint32_t StepMotor_angleToPulse(StepMotor_t *motor, float angle) {
    return angle / motor->_StepAngle * motor->_Subdivision;
}

static void StepMotor_getIterationData(StepMotor_t *motor, float *rpm_in, uint32_t *pulse_in) {
    if (motor->_iteration < motor->_itertration_num) {
        *rpm_in = motor->_soft_target_rpm * motor->_iteration / motor->_itertration_num;
        *pulse_in = abs(*rpm_in) * motor->_iteration_pulse;
        motor->_soft_target_pulse -= 2 * (*pulse_in);
        if (*pulse_in < 0) {
            *pulse_in = 0;
        }
    } else if (motor->_iteration == motor->_itertration_num) {
        *rpm_in = motor->_soft_target_rpm;
        *pulse_in = motor->_soft_target_pulse;
    } else if (motor->_iteration <= 2 * motor->_itertration_num) {
        *rpm_in = motor->_soft_target_rpm * (motor->_itertration_num * 2 - motor->_iteration) / motor->_itertration_num;
        *pulse_in = abs(*rpm_in) * motor->_iteration_pulse;
    } else {
        *rpm_in = 0;
        *pulse_in = 0;
    }
}