/*
 * SerialComm.h
 *
 *  Created on: Nov 28, 2025
 *      Author: SE-02
 */



#ifndef SERIALCOMM_SERIALCOMM_H_
#define SERIALCOMM_SERIALCOMM_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

/* ==========================================================
   CONFIGURATION
   ========================================================== */

#define UART_RX_BUFFER_SIZE   512
#define UART_TX_TIMEOUT       1000

/* ==========================================================
   TYPE DEFINITIONS
   ========================================================== */

/* User-level RX callback */
typedef void (*SerialRxCallback)(void);

/* Serial Port Structure */
typedef struct
{
    UART_HandleTypeDef *huart;

    volatile size_t head;
    volatile size_t tail;

    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];

    /* Single byte used by HAL interrupt reception */
    uint8_t rxByte;

    /* Optional user callback */
    SerialRxCallback rx_callback;

} SerialPort;

/* ==========================================================
   INITIALIZATION
   ========================================================== */

void SerialInit(SerialPort *sp, UART_HandleTypeDef *huart, uint32_t baud);
SerialPort SerialCreate(UART_HandleTypeDef *huart, uint32_t baud);

/* ==========================================================
   TRANSMIT FUNCTIONS
   ========================================================== */

int  SerialWrite(SerialPort *sp, const void *data, size_t length);
int  SerialPutc(SerialPort *sp, char c);
void SerialPuts(SerialPort *sp, const char *str);
int  SerialPrintf(SerialPort *sp, const char *fmt, ...);

/* ==========================================================
   RECEIVE FUNCTIONS
   ========================================================== */

int  SerialGetc(SerialPort *sp);
int  SerialRead(SerialPort *sp, void *data, size_t length);
int  SerialGets(SerialPort *sp, char *buf, size_t size);

/* ==========================================================
   BUFFER STATE
   ========================================================== */

bool   SerialReadable(SerialPort *sp);
bool   SerialWritable(SerialPort *sp);
size_t SerialAvailable(SerialPort *sp);
void   SerialFlushRx(SerialPort *sp);

/* ==========================================================
   CALLBACK ATTACH (Optional User-Level Callback)
   ========================================================== */

void SerialAttachRxCallback(SerialPort *sp, SerialRxCallback cb);

/* ==========================================================
   HAL ACCESS
   ========================================================== */

UART_HandleTypeDef* SerialGetHandle(SerialPort *sp);

void store_char(SerialPort *sp, uint8_t c);

/* ==========================================================
   UTILITY
   ========================================================== */

void ComputeCRC(uint8_t *buf, int len);


#endif /* SERIALCOMM_SERIALCOMM_H_ */
