/*
 * stepper.c
 *
 *  Created on: 02-Jul-2026
 *      Author: Electronics Dept
 */

#include "stepper.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "uart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "timer_app.h"   /* delay_us() for the StepperMotor library at the end */

/* TIM7 basic timer (configured in CubeMX: PSC=215 -> 1 MHz tick, IT enabled).
 * Used ONLY for speed>100 so the fast stepping runs in the background and the
 * calling task sleeps instead of busy-waiting. */
extern TIM_HandleTypeDef htim7;

/* Proximity sensor input used by stepper_proximity_sensing().
 * BEST: assign the wired pin a User Label "Proximity_sensing" in CubeMX so
 * main.h generates these two defines. The #ifndef fallback below only exists
 * so the file still COMPILES -- change the placeholder pin/port to your real
 * wiring if you have not set the label in CubeMX. */
#ifndef Proximity_sensing_Pin
#define Proximity_sensing_Pin        GPIO_PIN_2      /* <-- CHANGE to your pin */
#define Proximity_sensing_GPIO_Port  GPIOG           /* <-- CHANGE to your port */
#endif

int i = 0;
char rxBuffer[40];
char msg[50];
int steps = 0;
char direction;
int d = 0;

int MotorSpeed=0;

/* ===================== TIM7 background stepping (speed>100) =============== *
 * The two 8-sub-state coil tables below are exactly the sequences played by
 * clock_rot() (dir 0) and clock_antirot() (dir 1). For fast moves TIM7 fires
 * one interrupt per sub-state; the ISR writes that sub-state and counts a full
 * step every 8 sub-states. The task blocks on stepSem (fully yields) until the
 * ISR reports the move complete -- so TCP/servo tasks keep running.           */
static const uint8_t stepSeq[2][8][4] = {
    /* dir 0 == clock_rot(), played i = 7..0 */
    { {1,0,0,1},{1,0,0,0},{1,1,0,0},{0,1,0,0},{0,1,1,0},{0,0,1,0},{0,0,1,1},{0,0,0,1} },
    /* dir 1 == clock_antirot(), played i = 0..7 */
    { {0,0,1,0},{0,0,0,0},{0,1,0,1},{0,1,0,1},{0,1,1,1},{1,1,1,1},{1,0,1,1},{1,0,1,0} }
};

static volatile uint8_t  tmrDir         = 0;
static volatile uint8_t  tmrSubIdx      = 0;
static volatile uint8_t  tmrActive      = 0;
static volatile uint32_t tmrStepsDone   = 0;
static volatile uint32_t tmrStepsTarget = 0;

static StaticSemaphore_t stepSemBuf;
static SemaphoreHandle_t stepSem = NULL;

/* ===================== Motor 2 background stepping (TIM10) ================= *
 * Motor 2 (PUL1A/PUL1B/DIR1A/DIR1B) now runs on its OWN timer, TIM10, entirely
 * independent of motor 1 on TIM7 -- separate state, separate semaphore. That
 * means both motors can step in the background at the same time (each blocking
 * call is on its own timer/semaphore). SetOutputs2() is forward-declared here
 * so stepperTimerTick2() below can reach it; the definition is further down. */
extern TIM_HandleTypeDef htim10;

void SetOutputs2(uint8_t A1, uint8_t A2, uint8_t A3, uint8_t A4);   /* fwd */

static volatile uint8_t  tmr2Dir         = 0;
static volatile uint8_t  tmr2SubIdx      = 0;
static volatile uint8_t  tmr2Active      = 0;
static volatile uint32_t tmr2StepsDone   = 0;
static volatile uint32_t tmr2StepsTarget = 0;

static StaticSemaphore_t stepSem2Buf;
static SemaphoreHandle_t stepSem2 = NULL;

/* Called once per TIM7 update (see HAL_TIM_PeriodElapsedCallback below).
 * MOTOR 1 (PULA/PULB/DIRA/DIRB). */
void stepperTimerTick(void)
{
    if (!tmrActive) return;

    const uint8_t *o = stepSeq[tmrDir][tmrSubIdx];
    SetOutputs(o[0], o[1], o[2], o[3]);

    if (++tmrSubIdx >= 8) {
        tmrSubIdx = 0;
        if (++tmrStepsDone >= tmrStepsTarget) {
            HAL_TIM_Base_Stop_IT(&htim7);
            tmrActive = 0;
            BaseType_t hpw = pdFALSE;
            xSemaphoreGiveFromISR(stepSem, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

/* Called once per TIM10 update (routed from HAL_TIM_PeriodElapsedCallback).
 * MOTOR 2 (PUL1A/PUL1B/DIR1A/DIR1B) -- an exact twin of stepperTimerTick(),
 * on its own TIM10 + tmr2* state so it runs independently of motor 1. */
void stepperTimerTick2(void)
{
    if (!tmr2Active) return;

    const uint8_t *o = stepSeq[tmr2Dir][tmr2SubIdx];
    SetOutputs2(o[0], o[1], o[2], o[3]);

    if (++tmr2SubIdx >= 8) {
        tmr2SubIdx = 0;
        if (++tmr2StepsDone >= tmr2StepsTarget) {
            HAL_TIM_Base_Stop_IT(&htim10);
            tmr2Active = 0;
            BaseType_t hpw = pdFALSE;
            xSemaphoreGiveFromISR(stepSem2, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

/* Run `steps` full steps at `speed` (>100) via TIM7; blocks (yields) till done.
 * MOTOR 1. */
static void stepperTimerRun(uint32_t steps, int direction, int speed)
{
    if (steps == 0) return;

    if (stepSem == NULL) {
        stepSem = xSemaphoreCreateBinaryStatic(&stepSemBuf);
    }
    (void) xSemaphoreTake(stepSem, 0);   /* drain any stale signal */

    /* Per-sub-state period in us. TIM7 ticks at 1 MHz so ARR = period_us-1.
     *   speed 200 -> 500 us,  speed 500 -> 200 us,  speed 1000 -> 100 us.
     * Clamped so ARR stays in 16-bit range and the ISR rate stays sane. */
    uint32_t period_us = 100000U / (uint32_t) speed;
    if (period_us < 20U)    period_us = 20U;
    if (period_us > 60000U) period_us = 60000U;

    tmrDir         = (direction == 1) ? 1U : 0U;
    tmrSubIdx      = 0;
    tmrStepsDone   = 0;
    tmrStepsTarget = steps;
    tmrActive      = 1;

    __HAL_TIM_SET_AUTORELOAD(&htim7, (uint16_t)(period_us - 1U));
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    HAL_TIM_Base_Start_IT(&htim7);

    /* Sleep until the ISR finishes. Timeout ~2x the estimated move time so a
     * lost interrupt cannot hang the task forever. */
    uint32_t est_ms = (steps * 8U * period_us) / 1000U;
    uint32_t to_ms  = est_ms * 2U + 2000U;
    if (xSemaphoreTake(stepSem, pdMS_TO_TICKS(to_ms)) != pdTRUE) {
        HAL_TIM_Base_Stop_IT(&htim7);
        tmrActive = 0;
        UART_Print("[STEPPER] TIM7 move timed out.\r\n");
    }
}

/* Run `steps` full steps at `speed` (>100) via TIM10; blocks (yields) till done.
 * MOTOR 2 -- a twin of stepperTimerRun() on TIM10 with its own tmr2 state and
 * stepSem2, so motor 2 steps in the background independently of motor 1 on TIM7.
 *
 * NOTE ON TIM10 TICK RATE: TIM10 is on APB2 (216 MHz) and was generated with
 * Prescaler = 431, so it ticks at 216 MHz / 432 = 500 kHz -- the SAME rate as
 * TIM7 (108 MHz / 216). So the period_us math below matches motor 1 exactly. */
static void stepperTimerRun2(uint32_t steps, int direction, int speed)
{
    if (steps == 0) return;

    if (stepSem2 == NULL) {
        stepSem2 = xSemaphoreCreateBinaryStatic(&stepSem2Buf);
    }
    (void) xSemaphoreTake(stepSem2, 0);   /* drain any stale signal */

    uint32_t period_us = 100000U / (uint32_t) speed;
    if (period_us < 20U)    period_us = 20U;
    if (period_us > 60000U) period_us = 60000U;

    tmr2Dir         = (direction == 1) ? 1U : 0U;
    tmr2SubIdx      = 0;
    tmr2StepsDone   = 0;
    tmr2StepsTarget = steps;
    tmr2Active      = 1;

    __HAL_TIM_SET_AUTORELOAD(&htim10, (uint16_t)(period_us - 1U));
    __HAL_TIM_SET_COUNTER(&htim10, 0);
    HAL_TIM_Base_Start_IT(&htim10);

    uint32_t est_ms = (steps * 8U * period_us) / 1000U;
    uint32_t to_ms  = est_ms * 2U + 2000U;
    if (xSemaphoreTake(stepSem2, pdMS_TO_TICKS(to_ms)) != pdTRUE) {
        HAL_TIM_Base_Stop_IT(&htim10);
        tmr2Active = 0;
        UART_Print("[STEPPER2] TIM10 move timed out.\r\n");
    }
}

/* NOTE: the TIM7 update interrupt calls stepperTimerTick() from within
 * HAL_TIM_PeriodElapsedCallback() in main.c (that callback already exists for
 * the TIM6 HAL time base -- we only add a TIM7 branch there, not a 2nd copy). */

void GPIO_Init(void)
{
    HAL_GPIO_WritePin(PULA_Pin_GPIO_Port, PULA_Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PULB_Pin_GPIO_Port, PULB_Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIRA_Pin_GPIO_Port, DIRA_Pin_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIRB_Pin_GPIO_Port, DIRB_Pin_Pin, GPIO_PIN_SET);
}

void SetOutputs(uint8_t A1, uint8_t A2, uint8_t A3,uint8_t A4)
{
    HAL_GPIO_WritePin(PULA_Pin_GPIO_Port, PULA_Pin_Pin,
                      A1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(PULB_Pin_GPIO_Port, PULB_Pin_Pin,
                      A2 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(DIRA_Pin_GPIO_Port, DIRA_Pin_Pin,
                      A3 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(DIRB_Pin_GPIO_Port, DIRB_Pin_Pin,
                      A4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void stop(){
	SetOutputs(0,0,0,0);
}

void clock_antirot(){
	for(int i =0; i < 8; i++){
		switch(i){
		case 0:{
			SetOutputs(0,0,1,0);
		}break;
		case 1: {
			SetOutputs(0,0,0,0);
		}break;
		case 2:{
			SetOutputs(0,1,0,1);
		}break;
		case 3:{
			SetOutputs(0,1,0,1);
		}break;
		case 4:{
			SetOutputs(0,1,1,1);
		}break;
		case 5:{
			SetOutputs(1,1,1,1);
		}break;
		case 6:{
			SetOutputs(1,0,1,1);
		}break;
		case 7:{
			SetOutputs(1,0,1,0);
		}break;
		}
     osDelay(MotorSpeed);
	}
}

void clock_rot(){
	for(i = 7; i >=0; i--){
		switch (i) {
		case 0: {
			SetOutputs(0,0,0,1);
		}break;
		case 1: {
			SetOutputs(0,0,1,1);
		}break;
		case 2: {
			SetOutputs(0,0,1,0);
		}break;
		case 3: {
			SetOutputs(0,1,1,0);
		}break;
		case 4: {
			SetOutputs(0,1,0,0);
		}break;
		case 5: {
			SetOutputs(1,1,0,0);
		}break;
		case 6: {
			SetOutputs(1,0,0,0);
		}break;
		case 7: {
			SetOutputs(1,0,0,1);
		}break;
		}
		osDelay(MotorSpeed);
	}
}


void stepperData(int steps, int direction, int speed){
   int count = 0;

   if (speed < 1) speed = 1;
   if (steps < 1) { stop(); return; }

   if (direction != 0 && direction != 1) { stop(); return; }

   if (speed <= 100) {
       /* Millisecond path: blocks but YIELDS via osDelay -> no RTOS stall. */
       MotorSpeed = 101 - speed;
       if (direction == 0){
           do{ clock_rot();     count++; }while(count<steps);
       } else {
           do{ clock_antirot(); count++; }while(count<steps);
       }
   } else {
       /* Fast path: TIM7 drives the sub-states in the background; the task
        * sleeps on a semaphore, so other RTOS tasks keep running. */
       stepperTimerRun((uint32_t) steps, direction, speed);   /* motor 1, TIM7 */
   }
}

/* =========================================================================
 * MOTOR 2  --  PUL1A / PUL1B / DIR1A / DIR1B
 * -------------------------------------------------------------------------
 * An exact twin of the motor-1 code above, driving the second set of pins.
 * SetOutputs2()/stop2()/clock_rot2()/clock_antirot2()/stepperData2() mirror
 * SetOutputs()/stop()/clock_rot()/clock_antirot()/stepperData() one-for-one --
 * same 8 sub-state coil sequences, same millisecond/timer split at speed 100,
 * same steps/direction/speed contract. The ONE difference: the fast path runs
 * on TIM10 (stepperTimerRun2 / stepperTimerTick2) instead of TIM7, so motor 2
 * steps independently of -- and simultaneously with -- motor 1. MotorSpeed2 is
 * separate from MotorSpeed so the two never clobber each other's per-step
 * delay in the millisecond path.
 * ========================================================================= */

int MotorSpeed2 = 0;

void SetOutputs2(uint8_t A1, uint8_t A2, uint8_t A3, uint8_t A4)
{
    HAL_GPIO_WritePin(PUL1A_Pin_GPIO_Port, PUL1A_Pin_Pin,
                      A1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(PUL1B_Pin_GPIO_Port, PUL1B_Pin_Pin,
                      A2 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(DIR1A_Pin_GPIO_Port, DIR1A_Pin_Pin,
                      A3 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(DIR1B_Pin_GPIO_Port, DIR1B_Pin_Pin,
                      A4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void stop2(void)
{
    SetOutputs2(0, 0, 0, 0);
}

void clock_antirot2(void)
{
    for (int j = 0; j < 8; j++) {
        switch (j) {
        case 0: { SetOutputs2(0, 0, 1, 0); } break;
        case 1: { SetOutputs2(0, 0, 0, 0); } break;
        case 2: { SetOutputs2(0, 1, 0, 1); } break;
        case 3: { SetOutputs2(0, 1, 0, 1); } break;
        case 4: { SetOutputs2(0, 1, 1, 1); } break;
        case 5: { SetOutputs2(1, 1, 1, 1); } break;
        case 6: { SetOutputs2(1, 0, 1, 1); } break;
        case 7: { SetOutputs2(1, 0, 1, 0); } break;
        }
        osDelay(MotorSpeed2);
    }
}

void clock_rot2(void)
{
    for (int j = 7; j >= 0; j--) {
        switch (j) {
        case 0: { SetOutputs2(0, 0, 0, 1); } break;
        case 1: { SetOutputs2(0, 0, 1, 1); } break;
        case 2: { SetOutputs2(0, 0, 1, 0); } break;
        case 3: { SetOutputs2(0, 1, 1, 0); } break;
        case 4: { SetOutputs2(0, 1, 0, 0); } break;
        case 5: { SetOutputs2(1, 1, 0, 0); } break;
        case 6: { SetOutputs2(1, 0, 0, 0); } break;
        case 7: { SetOutputs2(1, 0, 0, 1); } break;
        }
        osDelay(MotorSpeed2);
    }
}

void stepperData2(int steps, int direction, int speed)
{
    int count = 0;

    if (speed < 1) speed = 1;
    if (steps < 1) { stop2(); return; }

    if (direction != 0 && direction != 1) { stop2(); return; }

    if (speed <= 100) {
        /* Millisecond path: blocks but YIELDS via osDelay -> no RTOS stall. */
        MotorSpeed2 = 101 - speed;
        if (direction == 0) {
            do { clock_rot2();     count++; } while (count < steps);
        } else {
            do { clock_antirot2(); count++; } while (count < steps);
        }
    } else {
        /* Fast path: TIM10 drives the sub-states in the background, motor 2 --
         * its own timer, so it can run at the same time as motor 1 on TIM7. */
        stepperTimerRun2((uint32_t) steps, direction, speed);
    }
}

/* =========================================================================
 * DROP-IN REPLACEMENT for stepper.c proximity section
 * -------------------------------------------------------------------------
 * Replaces: stepper_proximity_sensing()
 * Adds:     proxIsHigh(), proxPinConfigure(), stepperMoveUntilProxLow()
 * Modifies: stepperTimerTick()  (adds abort support -- see PATCH 3 below)
 * ========================================================================= */


/* ---------------- PATCH 1: proximity pin configuration ------------------ *
 * The existing #ifndef fallback silently compiles with GPIOG/PIN_2 if the
 * CubeMX User Label was never set. That pin may not be an input at all, in
 * which case HAL_GPIO_ReadPin() returns the output data register forever.
 * Set the label in CubeMX, or fix these two defines to your real wiring.
 *
 * Set PROX_PULL to match your sensor:
 *   GPIO_PULLDOWN -> 3-wire PNP sensor (drives HIGH when triggered, floats otherwise)
 *   GPIO_PULLUP   -> 3-wire NPN / sinking sensor, or a dry-contact switch
 *   GPIO_NOPULL   -> sensor actively drives BOTH states (push-pull output)
 */
#define PROX_PULL                 GPIO_PULLDOWN
#define PROX_SETTLE_MS            200    /* let the input settle after config  */
#define PROX_CHUNK_TARGET_MS      50     /* max motion time between pin checks */
#define PROX_DEBOUNCE_SAMPLES     3      /* consecutive agreeing reads needed  */
#define PROX_DEBOUNCE_GAP_MS      1

/* Minimum speed the proximity scan will run at.
 *
 * THIS IS WHY THE SCAN USED TO CRAWL. stepperData() contains two completely
 * different engines and the speed selects between them:
 *
 *   speed <= 100 -> millisecond path: 8 x osDelay(101 - speed) per step.
 *                   speed 10  =  8 x 91 ms  =  728 ms PER STEP  (1.4 steps/s)
 *   speed  > 100 -> TIM7 path: period_us = 100000/speed, 8 sub-states/step.
 *                   speed 1000 = 8 x 100 us =  0.8 ms per step  (1250 steps/s)
 *
 * A factor of about 900. Typing a "slow" speed at the prompt silently picked
 * the millisecond path, and proxChunkSize() then clamped the chunk to a single
 * step -- so the sensor was read once every 728 ms and the motor inched along.
 *
 * The scan speed is therefore clamped UP to this value. Detection is unchanged:
 * the pin is still read between chunks, just far more often in wall-clock terms.
 * Enter a HIGHER number at the prompt to go faster still; the clamp only ever
 * raises the speed, never lowers it. */
#define PROX_FAST_SPEED           1000

/* Phase 2 speed -- the deliberate slow creep after the sensor has tripped.
 * Fast approach, then this. Was a bare 1000 in the code. */
#define PROX_PHASE2_SPEED         1000

/* Above this, more speed buys nothing. stepperTimerRun() computes
 * period_us = 100000/speed and then floors it at 20 us, so 100000/20 = 5000 is
 * the highest speed that still changes anything: 5000, 10000 and 50000 all end
 * up at 20 us per sub-state = 160 us per full step = ~6250 steps/s. Entering
 * 10000 is not wrong, it just lands on the same rate as 5000. */
#define PROX_SPEED_CEILING        5000

/* Hard cap on chunk size, in STEPS.
 *
 * THIS IS WHY THE while() LOOP DOES NOT BREAK WHEN THE SENSOR SWITCHES.
 *
 * proxChunkSize() sizes a chunk by TIME -- PROX_CHUNK_TARGET_MS of motion --
 * so the faster the scan, the MORE steps land in one chunk. At speed 10000 a
 * chunk is 312 steps. The pin is only read BETWEEN chunks, and stepperData()
 * blocks for the whole chunk, so:
 *
 *   - if the requested move fits inside one chunk (e.g. 300 steps at speed
 *     10000), the sensor is sampled exactly ONCE, before the motor starts.
 *     Switching it mid-move changes nothing: the loop then ends because
 *     `remaining` reached 0, not because the sensor tripped;
 *   - even on longer moves the motor can run up to a full chunk past the point
 *     where the sensor went low.
 *
 * Making the scan fast is what made this visible: the higher the speed, the
 * coarser the polling. Capping the chunk in STEPS as well as in time keeps the
 * pin polled at least this often whatever the speed. 50 steps at 160 us/step is
 * 8 ms of motion against ~2 ms of sampling, so the motor still moves ~80% of
 * the time. */
#define PROX_MAX_CHUNK_STEPS      50

/* The level the sensor presents while the machine should KEEP MOVING; the scan
 * stops as soon as the pin leaves it.
 *
 * The other reason a loop like this "never breaks" is polarity. If the motor
 * runs when it should stop and stops when it should run, flip this one define
 * -- do not rewrite the loop. GPIO_PIN_SET suits a sensor that holds the line
 * HIGH until it detects; GPIO_PIN_RESET suits one that pulls LOW on detect. */
#define PROX_RUN_LEVEL            GPIO_PIN_SET

static void proxPinConfigure(void)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = Proximity_sensing_Pin;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Pull  = PROX_PULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Proximity_sensing_GPIO_Port, &gi);
    osDelay(PROX_SETTLE_MS);
}

/* Debounced read. Returns 1 if HIGH (keep moving), 0 if LOW (stop).
 * A stepper driver right next to a capacitive sensor is a serious EMI
 * source -- a single-sample read WILL give you false transitions. */
static int proxIsHigh(void)
{
    int run = 0, stopv = 0;
    for (int n = 0; n < PROX_DEBOUNCE_SAMPLES; n++) {
        if (HAL_GPIO_ReadPin(Proximity_sensing_GPIO_Port,
                             Proximity_sensing_Pin) == PROX_RUN_LEVEL) run++;
        else                                                           stopv++;
        if (n < PROX_DEBOUNCE_SAMPLES - 1) osDelay(PROX_DEBOUNCE_GAP_MS);
    }
    return (run > stopv) ? 1 : 0;
}


/* ---------------- PATCH 2: chunked motion with per-chunk check ----------- *
 * THE ACTUAL BUG. Your while() re-evaluates the pin only when stepperData()
 * RETURNS. Work out how long one call actually takes:
 *
 *   speed <= 100 path:  MotorSpeed = 101 - speed, and clock_rot() does
 *                       8 x osDelay(MotorSpeed) per step.
 *                       speed 10  -> 8 x 91ms  = 728 ms PER STEP
 *                       600 steps -> 437 SECONDS in one call
 *
 *   speed >  100 path:  stepperTimerRun() blocks on stepSem until ALL steps
 *                       are done. speed 1000 -> 800 us/step;
 *                       600 steps -> 0.48 s per call.
 *
 * So on the slow path you are sampling the sensor roughly once every SEVEN
 * MINUTES. It is not that the loop never exits -- you are never looking.
 *
 * (Side note: the header comment claims speed 10 -> 80 ms/step, but the code
 * computes 101 - speed, giving 728 ms/step. The comment and the code
 * disagree; worth reconciling before you tune anything else.)
 *
 * Fix: break the move into chunks small enough that each stepperData() call
 * returns within ~PROX_CHUNK_TARGET_MS, and re-check the pin between chunks.
 */
/* Worked in MICROSECONDS, not milliseconds.
 *
 * The old version computed ms_per_step = (8 * period_us) / 1000 with integer
 * division: on the fast path a step takes 800 us, so that produced 0, was
 * clamped to 1, and the chunk came out ~62 steps by luck rather than by
 * arithmetic. At speed 2000 it would still have said 1 ms and sized the chunk
 * for a step twice as long as reality. Microsecond maths keeps the chunk
 * honest at any speed. */
/* How long ONE full step actually takes, in microseconds, mirroring exactly
 * what stepperData() will do with this speed -- including stepperTimerRun()'s
 * 20 us floor on the sub-state period.
 *
 * That floor is why speeds above PROX_SPEED_CEILING buy nothing: 100000/speed
 * is clamped up to 20 us, so speed 5000, 10000 and 50000 all run at the same
 * 160 us per step. Reporting the requested speed without applying the clamp
 * would claim a rate the hardware never delivers. */
static uint32_t proxUsPerStep(int speed)
{
    uint32_t us_per_step;

    if (speed <= 100) {
        uint32_t motor_delay = (uint32_t)(101 - speed);      /* as in stepperData */
        if (motor_delay < 1U) motor_delay = 1U;
        us_per_step = 8U * motor_delay * 1000U;
    } else {
        uint32_t period_us = 100000U / (uint32_t) speed;      /* as in stepperTimerRun */
        if (period_us < 20U)    period_us = 20U;
        if (period_us > 60000U) period_us = 60000U;
        us_per_step = 8U * period_us;
    }

    if (us_per_step < 1U) us_per_step = 1U;

    return us_per_step;
}

static uint32_t proxChunkSize(int speed)
{
    uint32_t chunk = ((uint32_t) PROX_CHUNK_TARGET_MS * 1000U)
                     / proxUsPerStep(speed);

    if (chunk < 1U) chunk = 1U;      /* one step is the finest granularity */

    /* Cap in steps as well as in time -- see PROX_MAX_CHUNK_STEPS. Without
     * this a fast scan puts the whole move in one chunk and the pin is read
     * once, before the motor starts. */
    if (chunk > (uint32_t) PROX_MAX_CHUNK_STEPS) {
        chunk = (uint32_t) PROX_MAX_CHUNK_STEPS;
    }

    return chunk;
}

/* Moves up to `total_steps`, checking the proximity pin between chunks.
 * Returns the number of steps actually completed. Stops early on LOW.
 *
 * OVERSHOOT IS THE PRICE OF SPEED, and it is worth knowing the number. The pin
 * is only read BETWEEN chunks, so the motor can run up to one whole chunk past
 * the point where the sensor tripped:
 *
 *   speed 1000, PROX_CHUNK_TARGET_MS 50  ->  chunk 62 steps  -> up to 62 late
 *   speed 1000, PROX_CHUNK_TARGET_MS 10  ->  chunk 12 steps  -> up to 12 late
 *
 * The old crawl overshot by about one step simply because it was 900x slower,
 * not because it was better engineered. If the extra travel matters, lower
 * PROX_CHUNK_TARGET_MS -- each read costs roughly
 * PROX_DEBOUNCE_SAMPLES x PROX_DEBOUNCE_GAP_MS (~2 ms), so 10 ms chunks still
 * leave the motor moving ~80% of the time. Going below that trades real speed
 * for precision, and at some point a fast approach plus a slow re-approach is
 * the better answer. */
static uint32_t stepperMoveUntilProxLow(int total_steps, int dir, int speed)
{
    uint32_t chunk     = proxChunkSize(speed);
    uint32_t remaining = (total_steps > 0) ? (uint32_t) total_steps : 0U;
    uint32_t done      = 0U;

    while (remaining > 0U) {
        uint32_t this_chunk;

        if (!proxIsHigh()) {
            stop();
            break;
        }

        this_chunk = (remaining > chunk) ? chunk : remaining;
        stepperData((int) this_chunk, dir, speed);

        remaining -= this_chunk;
        done      += this_chunk;
    }
    return done;
}


/* ---------------- PATCH 3: abort the TIM7 fast path mid-move ------------- *
 * Optional but recommended. Without this, a fast-path chunk still runs to
 * completion before the pin is re-read. With chunking that is only ~50 ms,
 * which is usually fine -- but if you need a hard stop the instant the sensor
 * trips, add this.
 *
 * In stepper.c, next to the other tmr* statics, add:
 *
 *     volatile uint8_t tmrAbort = 0;
 *
 * Then in stepperTimerTick(), insert at the very top of the function body,
 * immediately after `if (!tmrActive) return;`:
 *
 *     if (tmrAbort) {
 *         HAL_TIM_Base_Stop_IT(&htim7);
 *         tmrActive = 0;
 *         tmrAbort  = 0;
 *         SetOutputs(0, 0, 0, 0);
 *         BaseType_t hpw = pdFALSE;
 *         xSemaphoreGiveFromISR(stepSem, &hpw);
 *         portYIELD_FROM_ISR(hpw);
 *         return;
 *     }
 *
 * Do NOT call HAL_GPIO_ReadPin() from inside the ISR to decide this -- set
 * tmrAbort from an EXTI callback on the proximity pin instead, so the ISR
 * stays short.
 */


/* =========================================================
 PROXIMITY DISPENSE -- NON-INTERACTIVE
 =========================================================
 * The mechanical half of stepper_proximity_sensing(), with the console prompts
 * taken out so a sequence can call it. The menu entry below now just collects
 * the three numbers and calls this, so there is ONE implementation of the
 * two-phase move and the menu and the sequence cannot drift apart.
 *
 *   Phase 1: run at fast_speed until the proximity sensor leaves
 *            PROX_RUN_LEVEL, polling between chunks.
 *   Phase 2: continue `steps` further at PROX_PHASE2_SPEED -- the deliberate
 *            slow creep once the sensor has tripped.
 *
 * Returns the number of steps phase 1 completed before the sensor tripped; 0
 * means the sensor was already tripped and phase 1 was skipped. Phase 2 runs
 * either way.
 ========================================================= */
/* Split form: the two phases get INDEPENDENT step counts.
 *
 *   approach_steps -- phase 1 CAP: the furthest the fast approach will travel
 *                     looking for the sensor. Position, not dose. Make it
 *                     generous; the move stops the instant the sensor trips, so
 *                     an over-large cap costs nothing.
 *   dose_steps     -- phase 2 travel: the plunger push AFTER the sensor trips.
 *                     THIS is the amount delivered, so this is the one that
 *                     should scale with the requested weight.
 *
 * The single-argument stepperProximityDispense() below passes the same value
 * for both, which is exactly the old behaviour -- the menu is unchanged. */
uint32_t stepperProximityDispenseSplit(int approach_steps, int dose_steps,
                                       int dir, int fast_speed)
{
    uint32_t moved = 0U;

    if (approach_steps < 1) {
        UART_Print("\r\n[PROX] Approach steps must be >= 1 (got %d). Aborting.\r\n",
                   approach_steps);
        return 0U;
    }
    if (dose_steps < 1) {
        UART_Print("\r\n[PROX] Dose steps must be >= 1 (got %d). Aborting.\r\n",
                   dose_steps);
        return 0U;
    }
    if (dir != 0 && dir != 1) {
        UART_Print("\r\n[PROX] Direction must be 0 or 1 (got %d). Aborting.\r\n", dir);
        return 0U;
    }
    if (fast_speed < 1) {
        UART_Print("\r\n[PROX] Speed must be >= 1 (got %d). Aborting.\r\n", fast_speed);
        return 0U;
    }

    /* ---- force the fast (TIM7) path -- see PROX_FAST_SPEED ---- *
     * Clamped here rather than inside the move so every number reported below
     * is the one actually used. */
    if (fast_speed < PROX_FAST_SPEED) {
        if (fast_speed <= 100) {
            UART_Print("\r\n[PROX] Speed %d selects the millisecond path "
                       "(~%lu ms per step -- this is what made the scan "
                       "crawl).\r\n",
                       fast_speed,
                       (unsigned long) (8U * (101U - (uint32_t) fast_speed)));
        }

        UART_Print("[PROX] Raising scan speed %d -> %d (timer path).\r\n",
                   fast_speed, PROX_FAST_SPEED);

        fast_speed = PROX_FAST_SPEED;
    }

    /* ---- configure and REPORT the pin before moving anything ---- */
    proxPinConfigure();

    UART_Print("[PROX] Proximity pin reads %s\r\n",
               proxIsHigh() ? "HIGH (will move)" : "LOW (will NOT move)");

    if (fast_speed > PROX_SPEED_CEILING) {
        UART_Print("[PROX] NOTE: speed %d is above the %d ceiling. "
                   "stepperTimerRun() floors the sub-state period at 20 us, so "
                   "anything past %d runs at the same rate -- not faster.\r\n",
                   fast_speed, PROX_SPEED_CEILING, PROX_SPEED_CEILING);
    }

    UART_Print("[PROX] Scan speed %d -> %lu us/step, chunk = %lu steps "
               "(~%d ms of motion between sensor reads).\r\n",
               fast_speed,
               (unsigned long) proxUsPerStep(fast_speed),
               (unsigned long) proxChunkSize(fast_speed),
               PROX_CHUNK_TARGET_MS);

    /* ---- Phase 1 : approach, capped at approach_steps ---- */
    if (!proxIsHigh()) {
        UART_Print("[PROX] Already LOW -- skipping phase 1.\r\n"
                   "  If nothing is in front of the sensor, the polarity is\r\n"
                   "  inverted: change PROX_RUN_LEVEL or PROX_PULL, or check\r\n"
                   "  whether the sensor is taught NC instead of NO.\r\n");
    } else {
        UART_Print("[PROX] Phase 1: rotating until proximity goes LOW "
                   "(cap %d steps)...\r\n", approach_steps);
        moved = stepperMoveUntilProxLow(approach_steps, dir, fast_speed);
        UART_Print("[PROX] Phase 1 ended after %lu of %d steps.\r\n",
                   (unsigned long) moved, approach_steps);
    }

    /* ---- Phase 2 : the dose, dose_steps at the slow creep speed ---- */
    UART_Print("[PROX] Phase 2: dosing %d steps at speed %d.\r\n",
               dose_steps, PROX_PHASE2_SPEED);
    stepperData(dose_steps, dir, PROX_PHASE2_SPEED);
    stop();

    return moved;
}

/* Single-count form -- both phases use `steps`. Kept so the stepper menu's
 * proximity option is byte-for-byte unchanged. */
uint32_t stepperProximityDispense(int steps, int dir, int fast_speed)
{
    return stepperProximityDispenseSplit(steps, steps, dir, fast_speed);
}

/* ---------------- PATCH 4: the rewritten entry point --------------------- */
void stepper_proximity_sensing(void)
{
    int ps_steps = 0, ps_dir = 0, ps_speed = 0;

    /* ---- read the data, same pattern as menu option 1 ---- */
    strcpy(msg, "Enter Steps of Motor: ");
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    UART_ReadString(rxBuffer, sizeof(rxBuffer));
    ps_steps = atoi(rxBuffer);

    strcpy(msg, "Enter direction of Motor: ");
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    UART_ReadString(rxBuffer, sizeof(rxBuffer));
    ps_dir = atoi(rxBuffer);

    strcpy(msg, "Enter speed of Motor: ");
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    UART_ReadString(rxBuffer, sizeof(rxBuffer));
    ps_speed = atoi(rxBuffer);

    /* Everything mechanical -- validation, the speed clamp, the pin report and
     * both phases -- now lives in stepperProximityDispense(), so the sequence
     * gets identical behaviour without the prompts and there is only one copy
     * of the two-phase move to maintain. */
    (void) stepperProximityDispense(ps_steps, ps_dir, ps_speed);
}

/* =========================================================
 HOME THE DRAIN STEPPER TO ITS UPPER LIMIT
 =========================================================
 * History of this function, so the tuning is not re-broken:
 *
 *  - Original: stepperData(10000, 1, 0.5). `speed` is an int parameter, so 0.5
 *    truncated to 0 -> osDelay(0) between coil states -> all 8 phases written
 *    back-to-back at CPU speed. No stepper can follow that: it buzzes and
 *    stalls without rotating.
 *
 *  - First fix used speed = 1. Still too fast: 8 micro-steps x 1 ms = 8 ms per
 *    step = 125 steps/s, while every other stepperData() call in this project
 *    uses 10 or 50 (12.5 and 2.5 steps/s). The motor stalled again.
 *
 *  - Now speed = 10, matching stepperData(600, 0, 10) which is known to work
 *    on this machine. If homing is too slow, lower it a little at a time and
 *    stop as soon as the motor starts missing steps.
 *
 * Timing: one step = 8 x osDelay(SPEED) ms.
 *     speed 10 ->  80 ms/step -> 12.5 steps/s   (known good)
 *     speed  5 ->  40 ms/step -> 25   steps/s
 *     speed  1 ->   8 ms/step -> 125  steps/s   (stalls -- do not use)
 */

#define STEPPER_LIMIT_CHUNK        100      /* steps between switch checks       */
#define STEPPER_LIMIT_SPEED        10000      /* ms per micro-step -- known good   */
#define STEPPER_LIMIT_DIR           1      /* 1 = toward the limit; flip if wrong */
#define STEPPER_LIMIT_TIMEOUT_MS  400000u  /* give up after 2 min               */

/* 1 = force a pull-up on the limit input (matches the Mbed reference, which
 * does drainUpDownLimit.mode(PullUp)). Set to 0 if the switch already has an
 * external pull-up/pull-down and this fights it. */
#define STEPPER_LIMIT_USE_PULLUP    1

/* Returns 1 if the drain stepper is ALREADY at its upper limit, else 0.
 * Same electrical sense as drainStepperLimit(): LOW = at the limit. Lets the
 * caller skip homing and go straight to plunging when already parked. */
int drainStepperAtLimit(void)
{
#if STEPPER_LIMIT_USE_PULLUP
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin   = Drain_up_down_limit_Pin;
        gi.Mode  = GPIO_MODE_INPUT;
        gi.Pull  = GPIO_PULLUP;
        gi.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Drain_up_down_limit_GPIO_Port, &gi);
        osDelay(5);
    }
#endif
    return (HAL_GPIO_ReadPin(Drain_up_down_limit_GPIO_Port,
                             Drain_up_down_limit_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}

void drainStepperLimit(void)
{
    uint32_t start;
    uint32_t moved = 0;
    GPIO_PinState level;

#if STEPPER_LIMIT_USE_PULLUP
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin   = Drain_up_down_limit_Pin;
        gi.Mode  = GPIO_MODE_INPUT;
        gi.Pull  = GPIO_PULLUP;
        gi.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Drain_up_down_limit_GPIO_Port, &gi);
    }
#endif

    osDelay(200);   /* let the input settle before the first read */

    level = HAL_GPIO_ReadPin(Drain_up_down_limit_GPIO_Port,
                             Drain_up_down_limit_Pin);

    /* Always report the raw level: if homing does nothing, this single line
     * says whether the switch is the problem or the motor is. */
    UART_Print("[STEPPER] Limit pin reads %s\r\n",
               (level == GPIO_PIN_SET) ? "HIGH (not at limit -> will move)"
                                       : "LOW (already at limit -> will NOT move)");

    if (level != GPIO_PIN_SET) {
        UART_Print("[STEPPER] Already at limit -- no move needed.\r\n"
                   "  If the axis is NOT actually at the limit, the switch\r\n"
                   "  polarity is inverted: set STEPPER_LIMIT_USE_PULLUP to 0,\r\n"
                   "  or invert the comparison in this function.\r\n");
        return;
    }

    UART_Print("[STEPPER] Homing to upper limit (speed %d, dir %d)...\r\n",
               STEPPER_LIMIT_SPEED, STEPPER_LIMIT_DIR);

    start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(Drain_up_down_limit_GPIO_Port,
                            Drain_up_down_limit_Pin) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) > STEPPER_LIMIT_TIMEOUT_MS) {
            stop();
            UART_Print("[STEPPER] LIMIT TIMEOUT after %lu steps.\r\n"
                       "  Motor turning but switch never closed -> check switch wiring.\r\n"
                       "  Motor NOT turning                     -> raise STEPPER_LIMIT_SPEED.\r\n"
                       "  Motor turning the WRONG WAY           -> set STEPPER_LIMIT_DIR to 0.\r\n",
                       (unsigned long) moved);
            return;
        }

        stepperData(STEPPER_LIMIT_CHUNK, STEPPER_LIMIT_DIR, STEPPER_LIMIT_SPEED);
        moved += STEPPER_LIMIT_CHUNK;
    }

    UART_Print("[STEPPER] Homed at limit after %lu steps.\r\n",
               (unsigned long) moved);
}

/* =========================================================================
 * STEPPER LIBRARY  (moved here from new_stepper.c)
 * -------------------------------------------------------------------------
 * Struct-based 4-line stepper driven off the TIM2 microsecond timer
 * (delay_us from timer_app.c), with an acceleration ramp. The declarations and
 * the StepperMotor type live in stepper.h now. Names differ from the motor-1/2
 * code above (Stepper_* vs stepperData*), so nothing collides -- except the
 * private stepSeq table and the speed var, which are renamed stepSeqLib /
 * libMotorSpeed here to avoid the existing stepSeq[2][8][4] and MotorSpeed.
 * ========================================================================= */

/* Per-sub-state delay in microseconds, updated by the ramp in Stepper_Step. */
static int libMotorSpeed;

/* ---- Acceleration ramp ----
 * A stepper can RUN much faster than it can START from a standstill. Commanding
 * the fast target delay immediately (e.g. speed = 1) just stalls it. So START
 * at a larger (slower) per-sub-state delay the motor can begin at, then ramp
 * DOWN to the requested `speed` over the first STEPPER_ACCEL_STEPS steps.
 *   STEPPER_START_DELAY_US : delay (us) at the first step -- raise if it stalls
 *                            at the start, lower if ramp-up feels sluggish.
 *   STEPPER_ACCEL_STEPS    : steps to spread the ramp over -- longer = gentler
 *                            = higher reachable top speed. */
#define STEPPER_START_DELAY_US   12
#define STEPPER_ACCEL_STEPS      800

static const uint8_t stepSeqLib[8][4] = {
    {0, 0, 0, 1},
    {0, 0, 1, 1},
    {0, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 0},
    {1, 1, 0, 0},
    {1, 0, 0, 0},
    {1, 0, 0, 1},
};

void Stepper_Init(StepperMotor *m,
                  GPIO_TypeDef *A0_port, uint16_t A0_pin,
                  GPIO_TypeDef *A1_port, uint16_t A1_pin,
                  GPIO_TypeDef *A2_port, uint16_t A2_pin,
                  GPIO_TypeDef *A3_port, uint16_t A3_pin)
{
    m->A0_port = A0_port;  m->A0_pin = A0_pin;
    m->A1_port = A1_port;  m->A1_pin = A1_pin;
    m->A2_port = A2_port;  m->A2_pin = A2_pin;
    m->A3_port = A3_port;  m->A3_pin = A3_pin;

    Stepper_Off(m);
}

void Stepper_Off(StepperMotor *m)
{
    HAL_GPIO_WritePin(m->A0_port, m->A0_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->A1_port, m->A1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->A2_port, m->A2_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->A3_port, m->A3_pin, GPIO_PIN_RESET);
}

/* Write the four coil lines in one shot each via BSRR. */
static inline void Stepper_SetCoils(StepperMotor *m,
                                    uint8_t a0, uint8_t a1,
                                    uint8_t a2, uint8_t a3)
{
    m->A0_port->BSRR = a0 ? m->A0_pin : ((uint32_t) m->A0_pin << 16);
    m->A1_port->BSRR = a1 ? m->A1_pin : ((uint32_t) m->A1_pin << 16);
    m->A2_port->BSRR = a2 ? m->A2_pin : ((uint32_t) m->A2_pin << 16);
    m->A3_port->BSRR = a3 ? m->A3_pin : ((uint32_t) m->A3_pin << 16);
}

void Stepper_Clockwise(StepperMotor *m)
{
    for (int i = 7; i >= 0; i--) {
        Stepper_SetCoils(m, stepSeqLib[i][0], stepSeqLib[i][1],
                            stepSeqLib[i][2], stepSeqLib[i][3]);
        delay_us(libMotorSpeed);
    }
}

void Stepper_Anticlockwise(StepperMotor *m)
{
    for (int i = 7; i >= 0; i--) {
        Stepper_SetCoils(m,
                         !stepSeqLib[i][0],
                         !stepSeqLib[i][1],
                          stepSeqLib[i][2],
                          stepSeqLib[i][3]);
        delay_us(libMotorSpeed);
    }
}

void Stepper_Step(StepperMotor *m, int num_steps, int direction, int speed)
{
    if (speed < 1) speed = 1;

    int accelSteps = STEPPER_ACCEL_STEPS;
    if (accelSteps > num_steps) accelSteps = num_steps;

    int startDelay = STEPPER_START_DELAY_US;
    if (startDelay < speed) startDelay = speed;   /* never start faster than target */

    for (int i = 0; i < num_steps; i++) {

        /* Linear ramp from startDelay down to speed over the first accelSteps
         * steps, then hold at speed. */
        if (i < accelSteps && accelSteps > 0) {
            libMotorSpeed = startDelay
                          - (int)(((long)(startDelay - speed) * i) / accelSteps);
        } else {
            libMotorSpeed = speed;
        }

        if (direction == 0)
            Stepper_Clockwise(m);
        else
            Stepper_Anticlockwise(m);
    }

    Stepper_Off(m);
}

/* =========================================================================
 * MOTOR 3  --  PUL2A / PUL2B / DIR2A / DIR2B
 * -------------------------------------------------------------------------
 * The third stepper, driven by the SAME Stepper.h library engine (Stepper_Init
 * + Stepper_Step) that the stepper menu's "Operate Stepper" option uses for
 * motor 1 on PUL1A/PUL1B/DIR1A/DIR1B -- so it gets the same acceleration ramp
 * and the same steps/direction/speed contract, just on the second set of pins:
 *
 *      PUL2A = PD10   ->  A0
 *      PUL2B = PF2    ->  A1
 *      DIR2A = PE15   ->  A2
 *      DIR2B = PC3    ->  A3
 *
 * All four are already push-pull outputs driven low by MX_GPIO_Init(), so
 * nothing needs configuring here.
 *
 * The motor object is created ONCE and initialised on the first call, exactly
 * like the menu's libMotor, so repeated calls do not re-home or re-latch it.
 * Blocks until every step is done; Stepper_Step() de-energises the coils on
 * the way out.
 * ========================================================================= */

static StepperMotor motor3;
static int          motor3Ready = 0;

void stepper_3(int steps, int direction, int speed)
{
    if (!motor3Ready) {
        Stepper_Init(&motor3,
                     PUL2A_Pin_GPIO_Port, PUL2A_Pin_Pin,   /* A0 */
                     PUL2B_Pin_GPIO_Port, PUL2B_Pin_Pin,   /* A1 */
                     DIR2A_Pin_GPIO_Port, DIR2A_Pin_Pin,   /* A2 */
                     DIR2B_Pin_GPIO_Port, DIR2B_Pin_Pin);  /* A3 */
        motor3Ready = 1;
    }

    Stepper_Step(&motor3, steps, direction, speed);
}
