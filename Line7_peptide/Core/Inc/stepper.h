/*
 * stepper.h
 *
 *  Created on: 02-Jul-2026
 *      Author: Electronics Dept
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "gpio.h"
#include "usart.h"
#include "main.h"

extern UART_HandleTypeDef huart3;

void stepperData(int steps, int direction, int speed);
void clock_rot();
void clock_antirot();
void stop();

void GPIO_Init(void);

void SetOutputs(uint8_t A1, uint8_t A2, uint8_t A3, uint8_t A4);

/* Motor 2 (PUL1A / PUL1B / DIR1A / DIR1B) -- twin of the motor-1 functions
 * above, same steps/direction/speed contract. */
void stepperData2(int steps, int direction, int speed);
void clock_rot2(void);
void clock_antirot2(void);
void stop2(void);
void SetOutputs2(uint8_t A1, uint8_t A2, uint8_t A3, uint8_t A4);

void drainStepperLimit(void);
int  drainStepperAtLimit(void);

void stepper_proximity_sensing(void);

/* Non-interactive two-phase proximity dispense, for use inside a sequence.
 * Phase 1 runs at fast_speed until the proximity sensor trips; phase 2 then
 * creeps `steps` further at the fixed slow speed. Returns the number of steps
 * phase 1 completed (0 if the sensor was already tripped). */
uint32_t stepperProximityDispense(int steps, int dir, int fast_speed);

/* Same, but with independent counts: approach_steps caps the phase-1 hunt for
 * the sensor, dose_steps is the phase-2 push that governs how much is
 * delivered. Use this when the dose must scale (e.g. with an entered weight)
 * while the approach distance stays fixed. */
uint32_t stepperProximityDispenseSplit(int approach_steps, int dose_steps,
                                       int dir, int fast_speed);

/* =========================================================
 STEPPER LIBRARY (moved here from new_stepper.h)
 =========================================================
 * A struct-based 4-line stepper driver that uses the TIM2 microsecond timer
 * (timer_app.c) for its per-sub-state delays, with an acceleration ramp. One
 * StepperMotor object per motor; pass its address to the functions. Used by
 * the "Operate Stepper" option in the machine menu. */

#define ON                   1
#define OFF                  0
#define MICRO_STEP_INTERVAL  100
#define MICRO_SPEED          1500

typedef struct {
    GPIO_TypeDef *A0_port;
    uint16_t      A0_pin;

    GPIO_TypeDef *A1_port;
    uint16_t      A1_pin;

    GPIO_TypeDef *A2_port;
    uint16_t      A2_pin;

    GPIO_TypeDef *A3_port;
    uint16_t      A3_pin;
} StepperMotor;

void Stepper_Init(StepperMotor *m,
                  GPIO_TypeDef *A0_port, uint16_t A0_pin,
                  GPIO_TypeDef *A1_port, uint16_t A1_pin,
                  GPIO_TypeDef *A2_port, uint16_t A2_pin,
                  GPIO_TypeDef *A3_port, uint16_t A3_pin);

void Stepper_Off(StepperMotor *m);
void Stepper_Clockwise(StepperMotor *m);
void Stepper_Anticlockwise(StepperMotor *m);
void Stepper_Step(StepperMotor *m, int num_steps, int direction, int speed);

/* Motor 3 on PUL2A/PUL2B/DIR2A/DIR2B, through the same Stepper.h engine as
 * "Operate Stepper" uses for motor 1. Same steps/direction/speed contract:
 * direction 0 = clockwise, 1 = anticlockwise; speed is the per-sub-state
 * delay in microseconds (smaller = faster). Blocks until the move is done and
 * de-energises the coils on the way out. */
void stepper_3(int steps, int direction, int speed);

#endif /* INC_STEPPER_H_ */
