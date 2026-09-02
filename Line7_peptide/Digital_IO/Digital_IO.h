/*
 * Digital_IO.h
 *
 *  Created on: Nov 28, 2025
 *      Author: SE-02
 */

#ifndef DIGITAL_IO_DIGITAL_IO_H_
#define DIGITAL_IO_DIGITAL_IO_H_



#include "stm32f7xx_hal.h"

#define ON_SET 0x01
#define OFF_RESET 0x00

/* ---------------- DigitalOut ---------------- */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} DigitalOut;

void DigitalOut_Init(DigitalOut *obj, GPIO_TypeDef *port, uint16_t pin);
void DigitalOut_Write(DigitalOut *obj, int value);
int  DigitalOut_Read(DigitalOut *obj);


/* ---------------- DigitalIn ---------------- */

typedef enum {
    PullNone = GPIO_NOPULL,
    PullUp = GPIO_PULLUP,
    PullDown = GPIO_PULLDOWN
} PinMode;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} DigitalIn;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} EXTInterrupt;


void DigitalIn_Init(DigitalIn *obj, GPIO_TypeDef *port, uint16_t pin, PinMode mode);
int  DigitalIn_Read(DigitalIn *obj);
void DigitalIn_SetMode(DigitalIn *obj, PinMode mode);

#endif /* DIGITAL_IO_DIGITAL_IO_H_ */
