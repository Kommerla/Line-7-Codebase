/*
 * PwmOut.h
 *
 *  Created on: Nov 28, 2025
 *      Author: SE-02
 */

#ifndef PWM_Pwm_H_
#define PWM_Pwm_H_


#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stddef.h>
//#include "tim.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float duty;      // 0.0 to 1.0
    float period_s;  // period in seconds
} PWMOut;

void Pwm_init(PWMOut *obj, TIM_HandleTypeDef *htim, uint32_t channel);

// Duty cycle
void Pwm_write(PWMOut *obj, float value);
float Pwm_read(PWMOut *obj);

// Period setters
void Pwm_period(PWMOut *obj, float period_s);
void Pwm_period_ms(PWMOut *obj, uint32_t period_ms);
void Pwm_period_us(PWMOut *obj, uint32_t period_us);

// Pulse width setters
void Pwm_pulsewidth(PWMOut *obj, float pulse_s);
void Pwm_pulsewidth_ms(PWMOut *obj, uint32_t pulse_ms);
void Pwm_pulsewidth_us(PWMOut *obj, uint32_t pulse_us);

void Pwm_start(PWMOut *obj);
void Pwm_stop(PWMOut *obj);


#endif /* PWM_Pwm_H_ */
