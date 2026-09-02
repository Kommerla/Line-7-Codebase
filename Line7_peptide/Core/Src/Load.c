/*
 * Load.c
 *
 *  Created on: 04-Jul-2026
 *      Author: Electronics Dept
 *
 *  STM32 HAL Modbus/RS485 driver for the load cell / vibratory feeder
 *  controller (USART6). LoadCellStepperDispense() is a direct, faithful
 *  port of main.cpp's Load_Check() (the confirmed-working Mbed reference,
 *  menu case 24): same reset-pulse pair, same coarse/fine/super-fine
 *  fire-and-forget setpoint writes, same Start pulse, same "loop while
 *  feedback pin is LOW: read weight, jog stepper one step" real-time loop,
 *  same unconditional Stop pulse at the end -- just with direction/speed
 *  taken as parameters instead of hardcoded, since the menu (Machinemenu.c
 *  case 3) prompts the operator for both.
 */

#include "Load.h"
#include "uart.h"
#include "gpio.h"
#include "main.h"
#include "stepper.h"
#include "cmsis_os.h"

/* =========================================================
 INITIALIZATION
 ========================================================= */

void LoadCell_Init(LoadCellHandle *loadcell, UART_HandleTypeDef *huart,
		GPIO_TypeDef *startPort, uint16_t startPin,
		GPIO_TypeDef *stopPort, uint16_t stopPin,
		GPIO_TypeDef *feedbackPort, uint16_t feedbackPin,
		uint8_t slave_id) {

	loadcell->huart = huart;

	loadcell->startPort = startPort;
	loadcell->startPin  = startPin;

	loadcell->stopPort = stopPort;
	loadcell->stopPin  = stopPin;

	loadcell->feedbackPort = feedbackPort;
	loadcell->feedbackPin  = feedbackPin;

	loadcell->slave_id = slave_id;

	LoadCellResetDiagnostics(loadcell);
}

/* =========================================================
 UART FLUSH
 ========================================================= */

void LoadCellFlushUART(LoadCellHandle *loadcell) {
	/* Matches the proven-working LoadCellReadTest() (load_cell.c) exactly:
	 * clear the overrun flag and drain the data register, nothing more. */
	__HAL_UART_CLEAR_OREFLAG(loadcell->huart);
	__HAL_UART_FLUSH_DRREGISTER(loadcell->huart);
}

/* =========================================================
 SEND COMMAND
 ========================================================= */

void LoadCellSendCmd(LoadCellHandle *loadcell, uint8_t *cmd, int len) {
	LoadCellFlushUART(loadcell);

	UART_Print("[LOADCELL TX]");
	for (int i = 0; i < len; i++)
		UART_Print(" %02X", cmd[i]);
	UART_Print("\r\n");

	/* Matches LoadCellReadTest(): bounded 1000ms transmit, no extra settle
	 * delay before reading -- LoadCellReadResponse()'s own per-byte timeout
	 * now gives the transceiver/load cell plenty of turnaround room. */
	HAL_UART_Transmit(loadcell->huart, cmd, len, 1000);

	loadcell->tx_count++;
}

/* =========================================================
 READ RESPONSE
 ========================================================= */

bool LoadCellReadResponse(LoadCellHandle *loadcell, uint8_t *resp, int resp_len) {
	HAL_StatusTypeDef status;

	memset(resp, 0, resp_len);

	/* Matches the proven-working LoadCellReadTest() (load_cell.c): a full
	 * 1000ms timeout PER BYTE, not a tight one. The earlier 10ms-per-byte
	 * version was consistently coming back with 0 bytes -- the load cell
	 * (plus RS485 transceiver turnaround) apparently takes longer than
	 * 10ms to start replying, so every attempt expired before the first
	 * byte ever arrived, no matter how many retries ran on top of it. */
	for (int i = 0; i < resp_len; i++) {
		status = HAL_UART_Receive(loadcell->huart, &resp[i], 1, 1000);

		if (status != HAL_OK) {
			UART_Print("[LOADCELL RX] Timeout at byte %d/%d\r\n", i, resp_len);
			UART_Print("[LOADCELL RX] UART ISR = 0x%08lX\r\n",
					(unsigned long) loadcell->huart->Instance->ISR);
			loadcell->timeout_count++;
			return false;
		}
	}

	UART_Print("[LOADCELL RX] got %d/%d bytes: ", resp_len, resp_len);
	for (int i = 0; i < resp_len; i++) {
		UART_Print("%02X ", resp[i]);
	}
	UART_Print("\r\n");

	return true;
}

/* =========================================================
 CRC VALIDATION
 ========================================================= */

bool LoadCellValidateCRC(uint8_t *resp, int len) {
	uint16_t crc = 0xFFFF;
	uint16_t rx_crc;

	for (int j = 0; j < (len - 2); j++) {
		crc ^= (uint16_t) resp[j];

		for (int i = 0; i < 8; i++) {
			if (crc & CRC_LSB_MASK) {
				crc >>= 1;
				crc ^= CRC_POLYNOMIAL;
			} else {
				crc >>= 1;
			}
		}
	}

	rx_crc = ((uint16_t) resp[len - 1] << 8) | (uint16_t) resp[len - 2];

	if (crc != rx_crc) {
		UART_Print("[LOADCELL ERROR] CRC Failed\r\n");
		return false;
	}

	return true;
}

/* =========================================================
 MODBUS EXCEPTION
 ========================================================= */

bool LoadCellValidateException(LoadCellHandle *loadcell, uint8_t *resp) {
	if (resp[1] & 0x80) {
		loadcell->exception_count++;
		UART_Print("[LOADCELL EXCEPTION] Code : %02X\r\n", resp[2]);
		return false;
	}

	return true;
}

/* Internal helper: append CRC16 to a Modbus command buffer at buf[len],buf[len+1] */
static void LoadCellComputeCRC(uint8_t *buf, uint16_t len) {
	uint16_t crc = 0xFFFF;

	for (uint16_t pos = 0; pos < len; pos++) {
		crc ^= buf[pos];

		for (uint8_t i = 0; i < 8; i++) {
			if (crc & CRC_LSB_MASK) {
				crc >>= 1;
				crc ^= CRC_POLYNOMIAL;
			} else {
				crc >>= 1;
			}
		}
	}

	buf[len]     = crc & 0xFF;
	buf[len + 1] = (crc >> 8) & 0xFF;
}

/* =========================================================
 READ REGISTERS
 ========================================================= */




bool LoadCellReadRegisters(LoadCellHandle *loadcell, uint16_t reg,uint16_t count, uint16_t *out) {
	uint8_t cmd[8];
	uint8_t resp[64];

	cmd[0] = loadcell->slave_id;
	cmd[1] = MODBUS_READ_HOLDING;

	cmd[2] = reg >> 8;
	cmd[3] = reg & 0xFF;

	cmd[4] = count >> 8;
	cmd[5] = count & 0xFF;

	LoadCellComputeCRC(cmd, 6);

	//for(int i=0;i<8;i++){
		//UART_Print("%x\n\r",cmd[i]);
	//}

	for (int retry = 0; retry < LOADCELL_MAX_RETRY; retry++) {
		LoadCellSendCmd(loadcell, cmd, 8);

		if (!LoadCellReadResponse(loadcell, resp, (5 + (count * 2)))) {
			loadcell->retry_count++;
			continue;
		}

		if (!LoadCellValidateCRC(resp, (5 + (count * 2)))) {
			loadcell->crc_error_count++;
			continue;
		}

		if (!LoadCellValidateException(loadcell, resp)) {
			continue;
		}

		for (uint16_t i = 0; i < count; i++) {
			out[i] = ((uint16_t) resp[3 + (2 * i)] << 8)
					| (uint16_t) resp[4 + (2 * i)];
		}

		loadcell->communication_ok = true;

		return true;
	}

	loadcell->communication_ok = false;

	UART_Print("[LOADCELL ERROR] Read Register Failed REG=0x%04X\r\n", reg);

	return false;
}

/* =========================================================
 READ 16
 ========================================================= */

uint16_t LoadCellRead16(LoadCellHandle *loadcell, uint16_t reg) {
	uint16_t value = 0;

	LoadCellReadRegisters(loadcell, reg, 1, &value);

	return value;
}

/* =========================================================
 SAFE READ 16
 ========================================================= */

bool LoadCellSafeRead16(LoadCellHandle *loadcell, uint16_t reg, uint16_t *value) {
	return LoadCellReadRegisters(loadcell, reg, 1, value);
}

/* =========================================================
 WRITE MULTIPLE REGISTERS (device-specific: always 3 regs / 6 data bytes)
 ========================================================= */

bool LoadCellWriteMultipleAck(LoadCellHandle *loadcell, uint16_t reg,
		uint16_t value) {
	uint8_t cmd[15];
	uint8_t resp[8];

	cmd[0] = loadcell->slave_id;
	cmd[1] = MODBUS_WRITE_MULTIPLE;

	cmd[2] = (reg >> 8) & 0xFF;
	cmd[3] = reg & 0xFF;

	cmd[4] = 0x00;
	cmd[5] = 0x03;

	cmd[6] = 0x06;

	cmd[7] = (value >> 8) & 0xFF;
	cmd[8] = value & 0xFF;

	cmd[9]  = 0x00;
	cmd[10] = 0x00;

	cmd[11] = 0x00;
	cmd[12] = 0x00;

	LoadCellComputeCRC(cmd, 13);

	LoadCellSendCmd(loadcell, cmd, 15);

	if (!LoadCellReadResponse(loadcell, resp, 8)) {
		UART_Print("[LOADCELL ERROR] Multi Write ACK Timeout\r\n");
		return false;
	}

	if (!LoadCellValidateCRC(resp, 8)) {
		UART_Print("[LOADCELL ERROR] Multi Write CRC Failed\r\n");
		return false;
	}

	if (!LoadCellValidateException(loadcell, resp)) {
		return false;
	}

	return true;
}

/* =========================================================
 VERIFIED MULTI WRITE
 ========================================================= */

bool LoadCellWriteMultipleVerified(LoadCellHandle *loadcell, uint16_t reg,
		uint16_t value) {
	uint16_t readback;

	for (int retry = 0; retry < LOADCELL_MAX_RETRY; retry++) {
		if (!LoadCellWriteMultipleAck(loadcell, reg, value)) {
			continue;
		}

		osDelay(100);

		if (!LoadCellSafeRead16(loadcell, reg, &readback)) {
			UART_Print("[LOADCELL ERROR] Readback Failed\r\n");
			continue;
		}

		if (readback == value) {
			return true;
		}

		UART_Print("[LOADCELL ERROR] Verify Failed W=%u R=%u\r\n", value, readback);
	}

	UART_Print("[LOADCELL ERROR] Multi Verified Write Failed\r\n");

	return false;
}

/* =========================================================
 DOSING
 ========================================================= */

bool LoadCellSetCoarse(LoadCellHandle *loadcell, uint16_t value) {
	return LoadCellWriteMultipleVerified(loadcell, LC_REG_COARSE, value);
}

bool LoadCellSetFine(LoadCellHandle *loadcell, uint16_t value) {
	return LoadCellWriteMultipleVerified(loadcell, LC_REG_FINE, value);
}

bool LoadCellSetSuperFine(LoadCellHandle *loadcell, uint16_t value) {
	return LoadCellWriteMultipleVerified(loadcell, LC_REG_SUPER_FINE, value);
}

/* =========================================================
 WEIGHT
 ========================================================= */

bool LoadCellReadWeight(LoadCellHandle *loadcell, uint16_t *weight) {
	if (!LoadCellSafeRead16(loadcell, LC_REG_WEIGHT, weight)) {
		UART_Print("[LOADCELL ERROR] Weight Read Failed\r\n");
		return false;
	}

	return true;
}

/* =========================================================
 COLLECTION (Start/Stop pulse helpers -- SET then RESET, matching
 main.cpp's "Grv_Start=1; sleep(...); Grv_Start=0;" pattern)
 ========================================================= */

void LoadCellStartCollection(LoadCellHandle *loadcell) {
	HAL_GPIO_WritePin(loadcell->startPort, loadcell->startPin, GPIO_PIN_SET);
	osDelay(LC_START_PULSE_MS);
	HAL_GPIO_WritePin(loadcell->startPort, loadcell->startPin, GPIO_PIN_RESET);
}

void LoadCellStopCollection(LoadCellHandle *loadcell) {
	HAL_GPIO_WritePin(loadcell->stopPort, loadcell->stopPin, GPIO_PIN_SET);
	osDelay(LC_STOP_PULSE_MS);
	HAL_GPIO_WritePin(loadcell->stopPort, loadcell->stopPin, GPIO_PIN_RESET);

	UART_Print("\r\nCollection Stopped\r\n");
}

/* =========================================================
 WAIT FEEDBACK
 ========================================================= */

bool LoadCellWaitFeedback(LoadCellHandle *loadcell, uint32_t timeout_ms) {
	uint32_t start = HAL_GetTick();

	/* timeout_ms == 0 means wait forever -- no cap, keep waiting until the
	 * feedback pin actually goes HIGH (i.e. until the target weight is
	 * genuinely collected), instead of giving up on an arbitrary clock. */

	/* Feedback pin idles LOW (GPIO_PIN_RESET); wait for it to go HIGH,
	 * same convention as the real-time loop in LoadCellStepperDispense(). */
	while (HAL_GPIO_ReadPin(loadcell->feedbackPort, loadcell->feedbackPin) == GPIO_PIN_RESET) {
		if ((timeout_ms != 0) && ((HAL_GetTick() - start) >= timeout_ms)) {
			UART_Print("[LOADCELL ERROR] Feedback Timeout\r\n");
			return false;
		}
		osDelay(100);
	}

	return true;
}

/* =========================================================
 LOAD CHECK (vibratory feeder only, no stepper -- used by
 Machinemenu.c's menu case 2)
 ========================================================= */

bool LoadCellLoadCheck(LoadCellHandle *loadcell, uint16_t targetWeight, int coarseP, int fineP) {
	uint16_t coarse, fine, sfine;

	if (targetWeight == 0) {
		UART_Print("[LOADCELL ERROR] Invalid Target Weight\r\n");
		return false;
	}

	coarse = (targetWeight * coarseP) / 100;
	fine   = (targetWeight * fineP) / 100;
	sfine  = targetWeight;

	UART_Print("COARSE : %u\r\n", coarse);
	UART_Print("FINE   : %u\r\n", fine);
	UART_Print("SFINE  : %u\r\n", sfine);

	/* Matches main.cpp's proven-working Load_Check() exactly: coarse/fine/
	 * superfine are sent FIRE-AND-FORGET. The reference does
	 * "Weighing_scale.write(Temp_set_25, sizeof(Temp_set_25));" for each of
	 * these three registers with NO .read() anywhere afterward -- it never
	 * expects or waits for a reply to these writes at all. This used to go
	 * through LoadCellSetCoarse/Fine/SuperFine() -> LoadCellWriteMultiple
	 * Verified(), which waits for an 8-byte ACK -- so every write here was
	 * timing out waiting for a reply the device was never going to send,
	 * not because of any UART/hardware fault (that's what "Multi Write ACK
	 * Timeout" / "Multi Verified Write Failed" actually were). Same raw
	 * send already used correctly in LoadCellStepperDispense() below. */
	{
		uint8_t cmd[15] = { loadcell->slave_id, 0x10,
				0x00, 0x20, 0x00, 0x03, 0x06,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
		uint16_t decimal;

		cmd[3] = LC_REG_COARSE & 0xFF;
		decimal = coarse;
		cmd[7] = (decimal >> 8) & 0xFF;
		cmd[8] = decimal & 0xFF;
		LoadCellComputeCRC(cmd, 13);
		LoadCellSendCmd(loadcell, cmd, 15);
		osDelay(100);

		cmd[3] = LC_REG_FINE & 0xFF;
		decimal = fine;
		cmd[7] = (decimal >> 8) & 0xFF;
		cmd[8] = decimal & 0xFF;
		LoadCellComputeCRC(cmd, 13);
		LoadCellSendCmd(loadcell, cmd, 15);
		osDelay(100);

		cmd[3] = LC_REG_SUPER_FINE & 0xFF;
		decimal = sfine;
		cmd[7] = (decimal >> 8) & 0xFF;
		cmd[8] = decimal & 0xFF;
		LoadCellComputeCRC(cmd, 13);
		LoadCellSendCmd(loadcell, cmd, 15);
		osDelay(100);
	}

	LoadCellStartCollection(loadcell);
	osDelay(500);

	/* 0 = wait forever -- don't give up until the target weight is
	 * actually collected (was a hard 30s cap before). */
	if (!LoadCellWaitFeedback(loadcell, 0)) {
		LoadCellStopCollection(loadcell);
		return false;
	}
	osDelay(500);

	LoadCellStopCollection(loadcell);

	return true;
}

/* =========================================================
 LOAD CELL + STEPPER DISPENSE
 (faithful port of main.cpp's Load_Check(), menu case 24 --
 the confirmed-working reference)

 Sequence, matching the reference exactly:
   1. Reset pulse pair: pulse Start then pulse Stop, once each.
   2. Write coarse(30%)/fine(60%)/superfine(100%) setpoints to the feeder
      as FIRE-AND-FORGET Modbus writes -- no ACK read, no verification,
      same as the reference (it never reads a response after these).
   3. Pulse Start to begin the feeder.
   4. Loop while the feedback pin is LOW: read the scale, jog the stepper
      one step, repeat -- break out as soon as either the target weight
      is reached or the feedback pin goes HIGH.
   5. Pulse Stop once, unconditionally, at the end.

 direction/stepSpeed are operator-entered (Machinemenu.c case 3 prompts
 for both) -- the reference hardcodes these (drainUpDown.step(1,0,5)),
 everything else matches exactly. stepIncrement/timeout_ms are accepted
 only for call-site compatibility; the reference always jogs 1 step per
 check and has no timeout of its own -- it runs until the target weight
 is hit or the feedback pin goes HIGH.
 ========================================================= */
bool LoadCellStepperDispense(LoadCellHandle *loadcell, uint16_t targetWeight,
		int direction, int stepSpeed, int stepIncrement, uint32_t timeout_ms) {
	(void) stepIncrement;
	(void) timeout_ms;

	if (targetWeight == 0) {
		UART_Print("[LOADCELL ERROR] Invalid Target Weight\r\n");
		return false;
	}

	/* 1. reset pulse pair */
	LoadCellStartCollection(loadcell);
	osDelay(1000);

	LoadCellStopCollection(loadcell);
	osDelay(1000);

	/* 2. coarse/fine/superfine setpoints -- fire-and-forget */
	uint8_t cmd[15] = { loadcell->slave_id, 0x10,
			0x00, 0x20, 0x00, 0x03, 0x06,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint16_t decimal;

	/* coarse = 30% of target, register LC_REG_COARSE */
	cmd[3] = LC_REG_COARSE & 0xFF;
	decimal = (uint16_t) (((uint32_t) targetWeight * 30) / 100);
	cmd[7] = (decimal >> 8) & 0xFF;
	cmd[8] = decimal & 0xFF;
	LoadCellComputeCRC(cmd, 13);
	LoadCellSendCmd(loadcell, cmd, 15);
	osDelay(1000);

	/* fine = 60% of target, register LC_REG_FINE */
	cmd[3] = LC_REG_FINE & 0xFF;
	decimal = (uint16_t) (((uint32_t) targetWeight * 60) / 100);
	cmd[7] = (decimal >> 8) & 0xFF;
	cmd[8] = decimal & 0xFF;
	LoadCellComputeCRC(cmd, 13);
	LoadCellSendCmd(loadcell, cmd, 15);
	osDelay(1000);

	/* superfine = target, register LC_REG_SUPER_FINE */
	cmd[3] = LC_REG_SUPER_FINE & 0xFF;
	decimal = targetWeight;
	cmd[7] = (decimal >> 8) & 0xFF;
	cmd[8] = decimal & 0xFF;
	LoadCellComputeCRC(cmd, 13);
	LoadCellSendCmd(loadcell, cmd, 15);
	osDelay(1000);

	/* 3. begin */
	LoadCellStartCollection(loadcell);
	osDelay(1000);

	/* 4. real-time weight + stepper loop */
	uint16_t currentWeight = 0;
	bool targetReached = false;

	while (HAL_GPIO_ReadPin(loadcell->feedbackPort, loadcell->feedbackPin) == GPIO_PIN_RESET) {
		if (!LoadCellReadWeight(loadcell, &currentWeight)) {
			/* Match the reference's tolerance: a single bad read doesn't
			 * abort the dispense, it just keeps jogging and retries the
			 * read next loop. */
			UART_Print("[LOADCELL ERROR] Weight read failed, retrying\r\n");
		}

		UART_Print("\r\nWeight = %u\r\n", currentWeight);

		if (currentWeight >= targetWeight) {
			UART_Print("\r\nTarget reached\r\n");

			targetReached = true;
			break;
		}

		stepperData(1000, direction, stepSpeed);
	}

	/* 5. stop, unconditionally */
	LoadCellStopCollection(loadcell);
	osDelay(1000);

	UART_Print("\r\nCollection Done\r\n");

	return targetReached;
}

/* =========================================================
 DIAGNOSTICS
 ========================================================= */

void LoadCellResetDiagnostics(LoadCellHandle *loadcell) {
	loadcell->tx_count = 0;
	loadcell->rx_count = 0;
	loadcell->timeout_count = 0;
	loadcell->crc_error_count = 0;
	loadcell->exception_count = 0;
	loadcell->retry_count = 0;
	loadcell->communication_ok = true;
}

bool LoadCellCommunicationOK(LoadCellHandle *loadcell) {
	return loadcell->communication_ok;
}
