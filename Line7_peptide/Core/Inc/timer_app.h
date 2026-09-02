/*
 * timer_app.h
 *
 *  Created on: 14-Aug-2026
 *      Author: ESE
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


#ifndef INC_TIMER_APP_H_
#define INC_TIMER_APP_H_

void timerAppInit();
void delay_us(uint32_t us);
void delay_ms(uint32_t us);
void delay_sec(uint32_t us);
uint32_t micros(void);

#endif /* INC_TIMER_APP_H_ */
