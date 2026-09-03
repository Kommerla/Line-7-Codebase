/*
 * Load.h
 *
 *  Created on: 04-Jul-2026
 *      Author: Electronics Dept
 */

#ifndef INC_LOAD_H_
#define INC_LOAD_H_

/*
 * Load.h
 *
 *  Created on: 04-Jul-2026
 *      Author: Electronics Dept
 *
 */
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* =========================================================
   MODBUS CRC
   ========================================================= */

#define CRC_LSB_MASK               0x0001
#define CRC_POLYNOMIAL             0xA001

/* =========================================================
   MODBUS FUNCTION CODES
   ========================================================= */

#define MODBUS_READ_HOLDING        0x03
#define MODBUS_WRITE_SINGLE        0x06
#define MODBUS_WRITE_MULTIPLE      0x10

/* =========================================================
   MODBUS EXCEPTION CODES
   ========================================================= */

#define MODBUS_EX_ILLEGAL_FUNC     0x01
#define MODBUS_EX_ILLEGAL_ADDR     0x02
#define MODBUS_EX_ILLEGAL_VALUE    0x03
#define MODBUS_EX_SLAVE_FAILURE    0x04

/* =========================================================
   LOADCELL REGISTERS
   ========================================================= */

#define LC_REG_WEIGHT              0x0000

#define LC_REG_COARSE              0x0004
#define LC_REG_FINE                0x0006
#define LC_REG_SUPER_FINE          0x0008

/* =========================================================
   LOADCELL CONTROL
   ========================================================= */

#define LC_START_PULSE_MS          500
#define LC_STOP_PULSE_MS           500

/* =========================================================
   COMMUNICATION
   ========================================================= */

#define LOADCELL_MAX_RETRY         3
#define LOADCELL_RESPONSE_TIMEOUT  500

#define LC_FLOW_TIMEOUT_MS     8000
#define LC_MIN_WEIGHT_CHANGE   2

/* =========================================================
   LOADCELL HANDLE
   ========================================================= */

typedef struct
{
    UART_HandleTypeDef *huart;

    /* Start / stop pulse outputs and feedback input, wired straight to the
       pins already defined in main.h (Load_cell_start_Pin, Load_cell_stop_Pin,
       Load_cell_feedback_Pin). feedbackPort/Pin MUST be configured as an
       input in MX_GPIO_Init() -- see gpio.c fix. */
    GPIO_TypeDef *startPort;
    uint16_t      startPin;

    GPIO_TypeDef *stopPort;
    uint16_t      stopPin;

    GPIO_TypeDef *feedbackPort;
    uint16_t      feedbackPin;

    uint8_t slave_id;

    /* Diagnostics */
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t crc_error_count;
    uint32_t exception_count;
    uint32_t retry_count;

    bool communication_ok;

} LoadCellHandle;

typedef enum
{
    LC_COLLECTION_ERROR = 0,
    LC_COLLECTION_TIMEOUT,
    LC_COLLECTION_TARGET_REACHED

} LoadCellCollectionStatus;

/* =========================================================
   INITIALIZATION
   ========================================================= */

void LoadCell_Init(
    LoadCellHandle *loadcell,
    UART_HandleTypeDef *huart,
    GPIO_TypeDef *startPort, uint16_t startPin,
    GPIO_TypeDef *stopPort,  uint16_t stopPin,
    GPIO_TypeDef *feedbackPort, uint16_t feedbackPin,
    uint8_t slave_id
);

/* =========================================================
   LOW LEVEL FUNCTIONS
   ========================================================= */

void LoadCellFlushUART(LoadCellHandle *loadcell);

void LoadCellSendCmd(LoadCellHandle *loadcell, uint8_t *cmd, int len);

bool LoadCellReadResponse(LoadCellHandle *loadcell, uint8_t *resp, int resp_len);

bool LoadCellValidateCRC(uint8_t *resp, int len);

bool LoadCellValidateException(LoadCellHandle *loadcell, uint8_t *resp);

/* =========================================================
   READ FUNCTIONS
   ========================================================= */

bool LoadCellReadRegisters(LoadCellHandle *loadcell, uint16_t reg, uint16_t count, uint16_t *out);

uint16_t LoadCellRead16(LoadCellHandle *loadcell, uint16_t reg);

bool LoadCellSafeRead16(LoadCellHandle *loadcell, uint16_t reg, uint16_t *value);

/* =========================================================
   WRITE FUNCTIONS
   ========================================================= */

bool LoadCellWriteMultipleAck(LoadCellHandle *loadcell, uint16_t reg, uint16_t value);

bool LoadCellWriteMultipleVerified(LoadCellHandle *loadcell, uint16_t reg, uint16_t value);

/* =========================================================
   DOSING FUNCTIONS
   ========================================================= */

bool LoadCellSetCoarse(LoadCellHandle *loadcell, uint16_t value);
bool LoadCellSetFine(LoadCellHandle *loadcell, uint16_t value);
bool LoadCellSetSuperFine(LoadCellHandle *loadcell, uint16_t value);

/* =========================================================
   WEIGHT
   ========================================================= */

bool LoadCellReadWeight(LoadCellHandle *loadcell, uint16_t *weight);

/* =========================================================
   COLLECTION
   ========================================================= */

void LoadCellStartCollection(LoadCellHandle *loadcell);
void LoadCellStopCollection(LoadCellHandle *loadcell);

bool LoadCellWaitFeedback(LoadCellHandle *loadcell, uint32_t timeout_ms);

/* =========================================================
   PROCESS
   ========================================================= */

bool LoadCellLoadCheck(LoadCellHandle *loadcell, uint16_t targetWeight, int coarseP, int fineP);

/* Faithful port of the proven-working main.cpp Load_Check(): sets the
 * feeder's coarse/fine/superfine setpoints (fire-and-forget, no ACK --
 * matches the working reference exactly), then jogs the STEPPER while
 * polling the scale, breaking on target-reached OR the feedback pin going
 * high. This is what "Dispense by Weight" should actually call. */
bool LoadCellLoadCheckWithStepper(LoadCellHandle *loadcell, uint16_t targetWeight,
        int stepDirection, int stepSpeed);

bool LoadCellStepperDispense(LoadCellHandle *loadcell, uint16_t targetWeight,
        int direction, int stepSpeed, int stepIncrement, uint32_t timeout_ms);

/* =========================================================
   DIAGNOSTICS
   ========================================================= */

void LoadCellResetDiagnostics(LoadCellHandle *loadcell);
bool LoadCellCommunicationOK(LoadCellHandle *loadcell);

#endif /* INC_LOAD_H_ */
