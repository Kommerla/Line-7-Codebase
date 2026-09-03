/*
 * timer_app.c
 *
 *  Created on: 14-Aug-2026
 *      Author: ESE
 */

#include "timer_app.h"
#include "stm32f7xx_hal.h"
#include "tim.h"

void timerAppInit()
{
	HAL_TIM_Base_Start(&htim2);
}

void delay_us(uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while (((__HAL_TIM_GET_COUNTER(&htim2) - start) & 0xFFFFFFFF) < us);
}

void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000);
}

void delay_sec(uint32_t sec)
{
    while (sec--) delay_ms(1000);
}

/* -------------------------------------------------- */
/* Time base (TIM2 configured as free-running counter) */
/* -------------------------------------------------- */
uint32_t micros(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

