/*
 * uart.h
 *
 *  Created on: 02-Jul-2026
 *      Author: Electronics Dept
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"

void UART_ReadLine(char *buffer, uint16_t length);
void UART_Print(const char *fmt, ...);
void UART_ReadString(char *buffer, uint16_t maxLength);

#endif /* INC_UART_H_ */
