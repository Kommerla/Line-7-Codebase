/*
 * servo.c
 *
 *  Created on: 29-Jun-2026
 *      Author: Electronics Dept
 */

#include "timer_app.h"
#include "servo.h"
#include "stdbool.h"
#include "string.h"
#include "main.h"
#include "uart.h"
#include <math.h>
#include "cmsis_os.h"

/* Interrupt-driven RX ring buffer for the servo bus (USART6).
 *
 * Implemented in freertos.c (the USART6 ISR fills the ring; these accessors
 * drain it). This replaces the old vTaskSuspendAll() approach: instead of
 * freezing the scheduler so a polled read can't be preempted, the ISR now
 * captures every byte the instant it arrives -- exactly what mbed's buffered
 * UARTSerial did -- so reads are immune to preemption without stalling LWIP
 * or any other task. */
void ServoRxRingInit(void);              /* enable USART6 RX interrupt + NVIC */
void ServoRxRingFlush(void);             /* discard buffered bytes, clear ORE  */
int  ServoRxRingGet(uint8_t *out_byte);  /* 1 = byte popped, 0 = ring empty    */


/* =========================================================
 INITIALIZATION
 ========================================================= */

void ServoInit(ServoHandle *servo, UART_HandleTypeDef *huart) {
	servo->huart = huart;

	servo->tx_count = 0;
	servo->rx_count = 0;

	servo->timeout_count = 0;
	servo->crc_error_count = 0;
	servo->exception_count = 0;

	servo->retry_count = 0;

	servo->communication_ok = true;

	/* Turn on interrupt-driven RX capture for the servo UART. Safe to call
	 * here (before the RTOS starts): the ISR only fills a plain ring buffer,
	 * it makes no RTOS calls. MX_USART6_UART_Init() has already run in main(),
	 * so the receiver is enabled. */
	ServoRxRingInit();
}

void ComputeCRC(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];

        for(uint8_t i = 0; i < 8; i++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    buf[len]     = crc & 0xFF;         // CRC Low byte
    buf[len + 1] = (crc >> 8) & 0xFF;  // CRC High byte
}


/* =========================================================
 UART FLUSH
 ========================================================= */

void ServoFlushUART(ServoHandle *servo)
{
	(void)servo;

	/* Drop anything the ISR has buffered so far (stale bytes / any TX echo)
	 * and clear a stale overrun. We must NOT read RDR here ourselves -- the
	 * ISR owns that register now; racing it would lose bytes. */
	ServoRxRingFlush();
}

/* =========================================================
 SEND COMMAND
 ========================================================= */

/* Pure transmit -- NO trailing delay.
 *
 * The old version did osDelay(10) right here, and THAT is the bug that made
 * every read fail with a CRC error. On the read path the sequence was:
 *
 *     flush -> transmit -> osDelay(10) -> HAL_UART_Receive(whole frame)
 *
 * At 9600 baud one byte takes ~1.04 ms, so a 9-byte reply takes ~9.4 ms. The
 * servo starts answering almost immediately after the frame leaves the wire,
 * so during that osDelay(10) the ENTIRE reply arrives -- but nobody is reading
 * yet. The STM32 has a single-byte RX register (no FIFO in this config): byte 1
 * latches (RXNE), byte 2 lands on top of it -> overrun (ORE), and bytes 2..N
 * are dropped. By the time HAL_UART_Receive() finally runs, only one stale byte
 * is left, so it times out / returns a corrupt partial frame -> CRC error,
 * every single time.
 *
 * mbed's UARTSerial never hit this because it is interrupt-buffered: an ISR
 * stashes every byte in a ring buffer the instant it arrives, so the 10 ms gap
 * before the app reads is harmless. To get the same robustness here we must
 * (a) not sit idle between transmit and receive, and (b) pull bytes out the
 * moment RXNE latches -- see ServoReadResponse().
 *
 * The inter-frame gap that fire-and-forget writes legitimately wanted now
 * lives in ServoWrite16()/ServoWrite32() instead of here, so it can't poison
 * the read path. */
void ServoSendCmd(ServoHandle *servo, uint8_t *cmd, int len) {
	HAL_UART_Transmit(servo->huart,
	                  cmd,
	                  len,
	                  HAL_MAX_DELAY);

	servo->tx_count++;
}

/* =========================================================
 CRC VALIDATION
 ========================================================= */

bool ServoValidateCRC(uint8_t *resp, int len) {
	uint16_t received_crc;
	uint16_t calculated_crc;

	uint8_t temp[64];

	memcpy(temp, resp, len - 2);

	ComputeCRC(temp, len - 2);

	received_crc = resp[len - 2] | (resp[len - 1] << 8);

	calculated_crc = temp[len - 2] | (temp[len - 1] << 8);

	return (received_crc == calculated_crc);
}

/* =========================================================
 READ RESPONSE
 ========================================================= */

/* Pop up to resp_len bytes from the RX ring buffer. Returns the count actually
 * received. The USART6 ISR (freertos.c) has already captured the bytes off the
 * wire, so this just drains them with an inter-byte idle timeout -- no risk of
 * overrun no matter what else the scheduler is doing, and no need to suspend
 * anything. */
static int ServoCaptureFrame(ServoHandle *servo, uint8_t *resp, int resp_len) {
	(void)servo;

	int idx = 0;
	uint32_t last_byte = HAL_GetTick();

	memset(resp, 0, resp_len);

	while (idx < resp_len &&
	       (HAL_GetTick() - last_byte) < (uint32_t) SERVO_RESPONSE_TIMEOUT) {

		uint8_t b;

		if (ServoRxRingGet(&b)) {
			resp[idx++] = b;
			last_byte = HAL_GetTick();   /* reset idle timeout on every byte */
		}
	}

	return idx;
}

bool ServoReadResponse(ServoHandle *servo, uint8_t *resp, int resp_len) {
	int got;

	/* Bytes are captured by the USART6 ISR into the ring buffer, so we can
	 * just drain them -- no scheduler suspension needed. Kept for any caller
	 * that transmits separately; the normal paths go through ServoTxRx. */
	got = ServoCaptureFrame(servo, resp, resp_len);

	UART_Print("[SERVO RX] got %d/%d: ", got, resp_len);
	for (int i = 0; i < got; i++) {
		UART_Print("%02X ", resp[i]);
	}
	UART_Print("\r\n");

	if (got != resp_len) {
		servo->timeout_count++;
		servo->communication_ok = false;

		return false;
	}

	if (!ServoValidateCRC(resp, resp_len)) {
		servo->crc_error_count++;
		servo->communication_ok = false;

		UART_Print("CRC Error\r\n");

		return false;
	}

	servo->rx_count++;
	servo->communication_ok = true;

	return true;
}

/* =========================================================
 MODBUS TRANSACTION
 =========================================================
 * Flush -> transmit -> receive.
 *
 * The earlier intermittent 0/9 (whole reply missed), 8/9 (one byte dropped)
 * and single-corrupt-byte reads all came from a context switch landing in the
 * middle of a transaction: this MCU's UART has a one-byte RX register (no FIFO
 * here), so any task stealing the CPU for even ~1 ms mid-reply overran it.
 *
 * That is now handled at the source: the USART6 ISR (freertos.c) copies every
 * incoming byte into a ring buffer the instant it arrives, so it no longer
 * matters which task is running or for how long. We flush the ring, transmit,
 * then drain the reply. No vTaskSuspendAll(), so LWIP and the other tasks keep
 * running normally. */
static bool ServoTxRx(ServoHandle *servo, uint8_t *cmd, int cmd_len,
		uint8_t *resp, int resp_len) {
	int got;

	ServoFlushUART(servo);
	ServoSendCmd(servo, cmd, cmd_len);
	got = ServoCaptureFrame(servo, resp, resp_len);

	UART_Print("[SERVO RX] got %d/%d: ", got, resp_len);
	for (int i = 0; i < got; i++) {
		UART_Print("%02X ", resp[i]);
	}
	UART_Print("\r\n");

	if (got != resp_len) {
		servo->timeout_count++;
		servo->communication_ok = false;

		return false;
	}

	if (!ServoValidateCRC(resp, resp_len)) {
		servo->crc_error_count++;
		servo->communication_ok = false;

		UART_Print("CRC Error\r\n");

		return false;
	}

	servo->rx_count++;
	servo->communication_ok = true;

	return true;
}

/* =========================================================
 READ REGISTERS
 ========================================================= */

bool ServoReadRegisters(ServoHandle *servo, uint8_t id, uint16_t reg,
		uint16_t count, uint16_t *out) {
	uint8_t cmd[8];

	uint8_t resp[64];

	int expected_len;

	for (int retry = 0; retry < SERVO_MAX_RETRY; retry++) {
		servo->retry_count++;

		cmd[0] = id;
		cmd[1] = MODBUS_READ_HOLDING;

		cmd[2] = (reg >> 8) & 0xFF;
		cmd[3] = reg & 0xFF;

		cmd[4] = (count >> 8) & 0xFF;
		cmd[5] = count & 0xFF;

		ComputeCRC(cmd, 6);

		expected_len = 5 + (2 * count);

		if (!ServoTxRx(servo, cmd, 8, resp, expected_len)) {
			UART_Print("Read Timeout/CRC Failed\r\n");

			continue;
		}

		/* MODBUS EXCEPTION */

		if (resp[1] & 0x80) {
			servo->exception_count++;

			UART_Print("Modbus Exception : %02X\r\n", resp[2]);

			return false;
		}

		/* HEADER VALIDATION */

		if ((resp[0] != id) || (resp[1] != MODBUS_READ_HOLDING)
				|| (resp[2] != (2 * count))) {
			UART_Print("Header Validation Failed\r\n");

			continue;
		}

		/* DATA EXTRACTION */

		for (int i = 0; i < count; i++) {
			out[i] = (resp[3 + (2 * i)] << 8) | resp[4 + (2 * i)];
		}

		return true;
	}

	return false;
}

/* =========================================================
 READ 16-BIT
 ========================================================= */

uint16_t ServoRead16(ServoHandle *servo, uint8_t id, uint16_t reg) {
	uint16_t data = 0;

	if (!ServoReadRegisters(servo, id, reg, 1, &data)) {
		UART_Print("Read16 Failed\r\n");

		return 0;
	}

	return data;
}

/* =========================================================
 READ 32-BIT
 ========================================================= */

int32_t ServoRead32(ServoHandle *servo, uint8_t id, uint16_t reg) {
	uint16_t data[2];

	if (!ServoReadRegisters(servo, id, reg, 2, data)) {
		UART_Print("Read32 Failed\r\n");

		return 0;
	}

	return (((int32_t) data[1]) << 16) | data[0];
}

/* =========================================================
 SAFE READ 16-BIT
 ========================================================= */

bool ServoSafeRead16(ServoHandle *servo, uint8_t id, uint16_t reg,
		uint16_t *value) {
	uint16_t data;

	for (int retry = 0; retry < SERVO_MAX_RETRY; retry++) {
		if (ServoReadRegisters(servo, id, reg, 1, &data)) {
			*value = data;

			return true;
		}

		osDelay(10);
	}

	return false;
}

/* =========================================================
 SAFE READ 32-BIT
 ========================================================= */

bool ServoSafeRead32(ServoHandle *servo, uint8_t id, uint16_t reg,
		int32_t *value) {
	uint16_t data[2];

	for (int retry = 0; retry < SERVO_MAX_RETRY; retry++) {
		if (ServoReadRegisters(servo, id, reg, 2, data)) {
			*value = (((int32_t) data[1]) << 16) | data[0];

			return true;
		}

		osDelay(10);
	}

	return false;
}

/* =========================================================
 NORMAL WRITE 16-BIT
 ========================================================= */

void ServoWrite16(ServoHandle *servo, uint8_t id, uint16_t reg, uint16_t value) {
	uint8_t cmd[8];

	cmd[0] = id;
	cmd[1] = MODBUS_WRITE_SINGLE;

	cmd[2] = (reg >> 8) & 0xFF;
	cmd[3] = reg & 0xFF;

	cmd[4] = (value >> 8) & 0xFF;
	cmd[5] = value & 0xFF;

	ComputeCRC(cmd, 6);

	ServoSendCmd(servo, cmd, 8);

	/* Inter-frame gap. This is fire-and-forget (no reply is read), so the
	 * delay is safe here -- unlike inside ServoSendCmd, where it used to sit
	 * between a read command and its reply and cause the overrun. Gives the
	 * servo time to process before the next frame on the shared RS485 bus. */
	osDelay(10);
}

/* =========================================================
 ACK WRITE 16-BIT
 ========================================================= */

bool ServoWrite16Ack(ServoHandle *servo, uint8_t id, uint16_t reg,
		uint16_t value) {
	uint8_t cmd[8];

	uint8_t resp[8];

	for (int retry = 0; retry < SERVO_MAX_RETRY; retry++) {
		servo->retry_count++;

		cmd[0] = id;
		cmd[1] = MODBUS_WRITE_SINGLE;

		cmd[2] = (reg >> 8) & 0xFF;
		cmd[3] = reg & 0xFF;

		cmd[4] = (value >> 8) & 0xFF;
		cmd[5] = value & 0xFF;

		ComputeCRC(cmd, 6);

		if (!ServoTxRx(servo, cmd, 8, resp, 8)) {
			UART_Print("ACK Timeout\r\n");

			continue;
		}

		if (resp[1] & 0x80) {
			servo->exception_count++;

			UART_Print("Modbus Exception : %02X\r\n", resp[2]);

			continue;
		}

		if (memcmp(cmd, resp, 8) != 0) {
			UART_Print("ACK Validation Failed\r\n");

			continue;
		}

		return true;
	}

	return false;
}

/* =========================================================
 VERIFIED WRITE 16-BIT
 ========================================================= */

bool ServoWrite16Verified(ServoHandle *servo, uint8_t id, uint16_t reg,
		uint16_t value) {
	uint8_t cmd[8];

	uint8_t resp[8];

	uint16_t readback;

	for (int retry = 0; retry < SERVO_MAX_RETRY; retry++) {
		servo->retry_count++;

		cmd[0] = id;
		cmd[1] = MODBUS_WRITE_SINGLE;

		cmd[2] = (reg >> 8) & 0xFF;
		cmd[3] = reg & 0xFF;

		cmd[4] = (value >> 8) & 0xFF;
		cmd[5] = value & 0xFF;

		ComputeCRC(cmd, 6);

		if (!ServoTxRx(servo, cmd, 8, resp, 8)) {
			UART_Print("ACK Timeout\r\n");

			continue;
		}

		if (resp[1] & 0x80) {
			servo->exception_count++;

			UART_Print("Modbus Exception : %02X\r\n", resp[2]);

			continue;
		}

		if (memcmp(cmd, resp, 8) != 0) {
			UART_Print("ACK Validation Failed\r\n");

			continue;
		}

		if (ServoSafeRead16(servo, id, reg, &readback)) {
			if (readback == value) {
				return true;
			}
		}

		UART_Print("Readback Failed\r\n");

		osDelay(20);
	}

	return false;
}

/* =========================================================
 NORMAL WRITE 32-BIT
 ========================================================= */

void ServoWrite32(ServoHandle *servo, uint8_t id, uint16_t reg, int32_t value) {
	uint8_t cmd[13];

	uint16_t low;
	uint16_t high;

	cmd[0] = id;
	cmd[1] = MODBUS_WRITE_MULTIPLE;

	cmd[2] = (reg >> 8) & 0xFF;
	cmd[3] = reg & 0xFF;

	cmd[4] = 0x00;
	cmd[5] = 0x02;

	cmd[6] = 0x04;

	low = value & 0xFFFF;
	high = (value >> 16) & 0xFFFF;

	cmd[7] = (low >> 8) & 0xFF;
	cmd[8] = low & 0xFF;

	cmd[9] = (high >> 8) & 0xFF;
	cmd[10] = high & 0xFF;

	ComputeCRC(cmd, 11);

	ServoSendCmd(servo, cmd, 13);

	/* Inter-frame gap -- safe here (fire-and-forget, no reply read). */
	osDelay(10);
}

/* =========================================================
 ACK WRITE 32-BIT
 =========================================================
 * Like ServoWrite32() but confirms the drive accepted the frame, retrying on
 * loss. A correct Write-Multiple echo (id, 0x10, reg, count) is only returned
 * when the servo received the whole frame with a valid CRC -- which includes
 * the data bytes -- so the echo alone proves the target value arrived intact.
 * No register read-back needed (and safer than one: SERVO_POSITION is a
 * write-to-trigger register whose read-back value isn't guaranteed to match).
 * Use this for the target position, where a silently dropped frame would send
 * the servo to the wrong place. */
bool ServoWrite32Ack(ServoHandle *servo, uint8_t id, uint16_t reg,
		int32_t value) {
	uint8_t cmd[13];

	uint8_t resp[8];

	uint16_t low;
	uint16_t high;

	for (int retry = 0; retry < SERVO_MAX_RETRY; retry++) {
		servo->retry_count++;

		cmd[0] = id;
		cmd[1] = MODBUS_WRITE_MULTIPLE;

		cmd[2] = (reg >> 8) & 0xFF;
		cmd[3] = reg & 0xFF;

		cmd[4] = 0x00;
		cmd[5] = 0x02;

		cmd[6] = 0x04;

		low = value & 0xFFFF;
		high = (value >> 16) & 0xFFFF;

		cmd[7] = (low >> 8) & 0xFF;
		cmd[8] = low & 0xFF;

		cmd[9] = (high >> 8) & 0xFF;
		cmd[10] = high & 0xFF;

		ComputeCRC(cmd, 11);

		if (!ServoTxRx(servo, cmd, 13, resp, 8)) {
			UART_Print("ACK Timeout\r\n");

			continue;
		}

		if (resp[1] & 0x80) {
			servo->exception_count++;

			UART_Print("Modbus Exception : %02X\r\n", resp[2]);

			continue;
		}

		if ((resp[0] != id) || (resp[1] != MODBUS_WRITE_MULTIPLE)
				|| (((uint16_t) resp[2] << 8) | resp[3]) != reg
				|| (((uint16_t) resp[4] << 8) | resp[5]) != 2) {
			UART_Print("ACK Validation Failed\r\n");

			continue;
		}

		return true;
	}

	return false;
}

/* =========================================================
 VERIFIED WRITE 32-BIT
 ========================================================= */

bool ServoWrite32Verified(ServoHandle *servo, uint8_t id, uint16_t reg,
		int32_t value) {
	uint8_t cmd[13];

	uint8_t resp[8];

	uint16_t low;
	uint16_t high;

	int32_t readback;

	for (int retry = 0; retry < SERVO_MAX_RETRY; retry++) {
		servo->retry_count++;

		cmd[0] = id;
		cmd[1] = MODBUS_WRITE_MULTIPLE;

		cmd[2] = (reg >> 8) & 0xFF;
		cmd[3] = reg & 0xFF;

		cmd[4] = 0x00;
		cmd[5] = 0x02;

		cmd[6] = 0x04;

		low = value & 0xFFFF;
		high = (value >> 16) & 0xFFFF;

		cmd[7] = (low >> 8) & 0xFF;
		cmd[8] = low & 0xFF;

		cmd[9] = (high >> 8) & 0xFF;
		cmd[10] = high & 0xFF;

		ComputeCRC(cmd, 11);

		if (!ServoTxRx(servo, cmd, 13, resp, 8)) {
			UART_Print("ACK Timeout\r\n");

			continue;
		}

		if (resp[1] & 0x80) {
			servo->exception_count++;

			UART_Print("Modbus Exception : %02X\r\n", resp[2]);

			continue;
		}

		if ((resp[0] != id) || (resp[1] != MODBUS_WRITE_MULTIPLE)
				|| (((uint16_t) resp[2] << 8) | resp[3]) != reg
				|| (((uint16_t) resp[4] << 8) | resp[5]) != 2) {
			UART_Print("ACK Validation Failed\r\n");

			continue;
		}

		if (ServoSafeRead32(servo, id, reg, &readback)) {
			if (readback == value) {
				return true;
			}
		}

		UART_Print("Readback Failed\r\n");

		osDelay(20);
	}

	return false;
}

/* =========================================================
 SAFE POSITION READ
 ========================================================= */

bool ServoSafeReadPosition(ServoHandle *servo, uint8_t id, uint16_t reg,
		int32_t *pos) {
	return ServoSafeRead32(servo, id, reg, pos);
}

/* =========================================================
 WAIT MOTION COMPLETE
 ========================================================= */

bool ServoWaitMotionComplete(ServoHandle *servo, uint8_t id, uint16_t posReg,
		int timeout_ms) {
	int32_t prev = 0;
	int32_t current = 0;

	uint32_t start = HAL_GetTick();

	if (!ServoSafeReadPosition(servo, id, posReg, &prev)) {
		UART_Print("Initial Position Read Failed\r\n");

		return false;
	}

	while ((HAL_GetTick() - start) < timeout_ms) {
		osDelay(100);

		if (!ServoSafeReadPosition(servo, id, posReg, &current)) {
			continue;
		}

		if (current == prev) {
			UART_Print("Motion Complete\r\n");

			return true;
		}

		prev = current;
	}

	UART_Print("Motion Timeout\r\n");

	return false;
}

/* =========================================================
 DEGREE TO PULSE
 ========================================================= */

int32_t DegreeToPulse(float deg) {
	float pulses;

	pulses = (SERVO_PPR * deg) / 360.0f;

	return (int32_t)roundf(pulses);
}

/* =========================================================
 RESET DIAGNOSTICS
 ========================================================= */

void ServoResetDiagnostics(ServoHandle *servo) {
	servo->tx_count = 0;
	servo->rx_count = 0;

	servo->timeout_count = 0;
	servo->crc_error_count = 0;
	servo->exception_count = 0;

	servo->retry_count = 0;
}

/* =========================================================
 COMMUNICATION STATUS
 ========================================================= */

bool ServoIsCommunicationOK(ServoHandle *servo) {
	return servo->communication_ok;
}

/* =========================================================
 POSITION CONFIG
 ========================================================= */

void ServoPositionConfig(ServoHandle *servo, uint8_t id) {
	ServoWrite16(servo, id, FAULT_RESET,        SET_HIGH);
    osDelay(50);

    ServoWrite16(servo, id, SERVO_POS_MODE,     INCREMENTAL);
    osDelay(50);

    ServoWrite16(servo, id, SERVO_POS_ACC_TIME, SERVO_SPEED_ACCEL);
    osDelay(50);

    ServoWrite16(servo, id, SERVO_POS_DEC_TIME, SERVO_SPEED_DECEL);
    osDelay(50);

    ServoWrite16(servo, id, SERVO_POS_SPEED,    5);
    osDelay(50);

    ServoWrite16(servo, id, ORGIN_SPEED_CMD,    10);
    osDelay(50);
}

/* =========================================================
 REACTION MOVE
 =========================================================
 *
 * Sequence
 * --------
 *  1. ServoPositionConfig  — reset fault, set incremental mode,
 *                            acc/dec, default speed.
 *  2. Advance +offset_deg  — move to starting position.
 *  3. Oscillate swing_deg  — alternate reverse/forward for
 *                            the requested number of cycles.
 *  4. Return  -offset_deg  — restore original position.
 *
 * Parameters
 * ----------
 *  servo          : ServoHandle already initialised with ServoInit()
 *  id             : Modbus slave ID of the mixer servo (MIXER_ID)
 *  offset_deg     : Initial offset in degrees  (e.g. 90)
 *  swing_deg      : Travel of ONE stroke in degrees (e.g. 360). The shaft
 *                   rocks half this angle either side of where it started.
 *                   0 is rejected -- it would just stand still.
 *  speed          : SERVO_POS_SPEED value during oscillation
 *  cycles         : Number of full rocks (one RIGHT + one LEFT each), e.g. 10
 *  cycle_delay_ms : osDelay between each stroke in ms
 ========================================================= */

/* --- WHY THE PENDULUM NEEDS ITS OWN WAIT ---------------------------------
 *
 * ServoWaitMotionComplete() calls a move finished as soon as TWO reads 100 ms
 * apart are equal. That is also true in the moment BEFORE the drive has begun
 * moving: the direction command has been sent, the shaft has not started yet,
 * so two reads come back identical and the move is declared complete -- and
 * ReactionStep() then STOPs the drive before it ever turned. That is exactly
 * the "one motion and then it stops" behaviour: the offset move runs (it has
 * the config writes in front of it to cover the drive's start-up latency) and
 * every swing after it is killed during that latency.
 *
 * So the pendulum waits in TWO phases: first for the shaft to actually LEAVE
 * where it started, and only then for it to settle. ServoWaitMotionComplete()
 * itself is left alone -- other callers depend on its behaviour. */
/* Drive counts per degree of mixer rotation = DegreeToPulse(deg) * this. It was
 * a bare 7.5 sitting in ReactionStep(); named here because the speed-mode
 * pendulum has to convert degrees the SAME way or the two functions would
 * disagree about what swing_deg means. (Dispenser's equivalent is
 * DISPENSER_POS_SCALE = 4.5 in servo.h.) */
#define REACTION_POS_SCALE       7.5f

#define REACTION_START_TIMEOUT   3000   /* ms allowed for the shaft to move off */
#define REACTION_STABLE_READS    3      /* equal reads that mean "stopped"      */
#define REACTION_POS_DEADBAND    5      /* encoder counts of read jitter        */

static bool ReactionWaitMotion(ServoHandle *servo, uint8_t id, int timeout_ms)
{
    int32_t  start_pos = 0, pos = 0, prev = 0, delta;
    uint32_t t0     = HAL_GetTick();
    int      stable = 0;
    bool     moving = false;

    if (!ServoSafeReadPosition(servo, id, READ_CURRENT_POS, &start_pos)) {
        UART_Print("Reaction : initial position read failed\r\n");
        return false;
    }
    prev = start_pos;

    while ((HAL_GetTick() - t0) < (uint32_t) timeout_ms) {
        osDelay(50);

        if (!ServoSafeReadPosition(servo, id, READ_CURRENT_POS, &pos))
            continue;   /* a dropped/CRC-failed frame is not "stopped" */

        if (!moving) {
            delta = pos - start_pos;
            if (delta < 0) delta = -delta;

            if (delta > REACTION_POS_DEADBAND) {
                moving = true;      /* the shaft is under way */
                prev   = pos;
            } else if ((HAL_GetTick() - t0) > REACTION_START_TIMEOUT) {
                UART_Print("Reaction : shaft never started moving\r\n");
                return false;
            }
            continue;
        }

        delta = pos - prev;
        if (delta < 0) delta = -delta;

        if (delta <= REACTION_POS_DEADBAND) {
            if (++stable >= REACTION_STABLE_READS) {
                UART_Print("Motion Complete\r\n");
                return true;
            }
        } else {
            stable = 0;
        }
        prev = pos;
    }

    UART_Print("Motion Timeout\r\n");
    return false;
}

/* One INCREMENTAL move of `deg` degrees on the reaction servo, blocking until
 * the drive reports it has finished, then stopping it.
 *
 * Factored out because the pendulum needs one of these per stroke and every
 * one of them must follow the same safe order: stop -> set target -> start ->
 * wait for completion -> stop. Skipping the trailing stop is what let the
 * motor keep coasting into the next step. */
static bool ReactionStep(ServoHandle *servo, uint8_t id, int deg,
                         const char *label, int cycle)
{
    int32_t position = DegreeToPulse((float) deg);
    bool ok;

    if (deg == 0)
        return true;            /* nothing to do -- and never silently no-op */

    /* Stop first so a new target is never stacked on a move in progress. */
    ServoWrite16(servo, id, SERVO_MOTION_CMD, SERVO_STOP);
    osDelay(100);

    ServoWrite32(servo, id, SERVO_POSITION, position * REACTION_POS_SCALE);
    osDelay(100);

    if (position >= 0)
        ServoWrite16(servo, id, SERVO_MOTION_CMD, SERVO_FORWARD);
    else
        ServoWrite16(servo, id, SERVO_MOTION_CMD, SERVO_REVERSE);

    ok = ReactionWaitMotion(servo, id, REACTION_MOTION_TIMEOUT);
    if (!ok)
        UART_Print("Reaction : Cycle %d  %s did not complete\r\n", cycle, label);

    /* Stop before returning so the shaft is genuinely at rest. */
    ServoWrite16(servo, id, SERVO_MOTION_CMD, SERVO_STOP);
    osDelay(100);

    return ok;
}

void ReactionMove(ServoHandle *servo, uint8_t id, int offset_deg,
        int swing_deg, uint16_t speed, int cycles, int cycle_delay_ms) {

    /* A zero swing makes every stroke a no-op -- the mixer would do the offset
     * move and then sit there looking broken. Say so and do nothing. */
    if (swing_deg == 0) {
        UART_Print("Reaction : swing = 0 deg -- nothing to swing, "
                   "aborting.\r\n");
        return;
    }

    if (cycles <= 0) cycles = 1;

    /* ---- 1. Configure servo ---- */

    ServoPositionConfig(servo, id);

    /* Speed goes in ONCE, before the first move; the drive keeps it for every
     * stroke that follows. */
    ServoWrite16(servo, id, SERVO_POS_SPEED, speed);
    osDelay(50);

    /* ---- 2. Initial offset move ----
     * Same step the strokes use, so it gets the same "wait until the shaft has
     * really started and really stopped" treatment, and an offset of 0 is a
     * clean no-op instead of a 15 s wait for a move that never happens. */

    ReactionStep(servo, id, offset_deg, "OFFSET", 0);

    /* ---- 3. Pendulum cycles ----
     *
     * Moves are INCREMENTAL (SERVO_POS_MODE = INCREMENTAL), i.e. each command
     * moves BY the given angle from wherever the shaft currently is -- it does
     * not move TO an absolute angle.
     *
     * EVERY stroke reverses direction: anticlockwise, clockwise, anticlockwise,
     * ... which is the rocking the mixer is supposed to do, and is exactly what
     * works when a clockwise and an anticlockwise move are run by hand from the
     * menu. The previous four-step version (LEFT, CENTRE, RIGHT, CENTRE) sent
     * TWO moves in the same direction back to back in the middle of each cycle,
     * so it read as one long lopsided sweep rather than a pendulum.
     *
     * To stay centred on where the mixer started, the shaft is first PRIMED
     * half a swing to one side; every stroke after that is a full swing_deg
     * from extreme to extreme, and a closing half swing brings it back to
     * centre. So swing_deg is the total travel of each stroke:
     *
     *   centre -half-> LEFT  -full-> RIGHT -full-> LEFT ... -half-> centre
     */

        /* Prime: half a swing to the LEFT so the rocking is centred. */
        UART_Print("Reaction : priming %+d deg to the swing extreme\r\n",
                   -swing_deg / 2);
        ReactionStep(servo, id, -swing_deg / 2, "PRIME", 0);
        if (cycle_delay_ms > 0) osDelay(cycle_delay_ms);

        for (int i = 0; i < cycles; i++) {

            UART_Print("Reaction : Cycle %d  RIGHT (%+d deg)\r\n", i, swing_deg);
            ReactionStep(servo, id,  swing_deg, "RIGHT", i);
            if (cycle_delay_ms > 0) osDelay(cycle_delay_ms);

            UART_Print("Reaction : Cycle %d  LEFT  (%+d deg)\r\n", i, -swing_deg);
            ReactionStep(servo, id, -swing_deg, "LEFT", i);
            if (cycle_delay_ms > 0) osDelay(cycle_delay_ms);
        }

        /* Closing half swing back to the centre it started from. */
        UART_Print("Reaction : recentring %+d deg\r\n", swing_deg / 2);
        ReactionStep(servo, id, swing_deg / 2, "RECENTRE", cycles);

        /* ---- 4. FINAL STOP ----
         * Guarantees the servo is not still turning when this function
         * returns. */
        ServoWrite16(servo, id, SERVO_MOTION_CMD, SERVO_STOP);
        osDelay(100);
}

/* =========================================================
 REACTION MOVE — CONTINUOUS (no stop between swings)
 =========================================================
 * The SAME pendulum as ReactionMove() -- prime half a swing to one side, then
 * alternate full swing_deg strokes, then a closing half swing back to centre --
 * but with no STOP and no "wait for the position to go quiet" between strokes:
 * each stroke ends on ENCODER FEEDBACK, the moment the shaft has covered the
 * requested degrees, and the next stroke starts straight away.
 *
 * THIS FUNCTION USED TO CLAIM SPEED (VELOCITY) MODE. IT NEVER GOT ONE.
 * Writing SERVO_CONTROL_MODE (500) = SPEED_CONTROL does not put this drive into
 * velocity control. The encoder trace proves it: with 360 deg strokes commanded,
 * every stroke travelled ~18760 counts and then the drive STOPPED BY ITSELF and
 * sat there until the 15 s timeout. 18750 counts is DegreeToPulse(180) * 7.5 --
 * the target a PREVIOUS position-mode run had left sitting in SERVO_POSITION.
 * A drive in velocity mode never stops on its own; this one was still executing
 * bounded incremental position moves against a stale target, so every stroke
 * came out 180 deg regardless of what was asked for, in both directions.
 *
 * So the drive is put explicitly into INCREMENTAL POSITION mode -- the mode it
 * demonstrably obeys -- and each stroke ARMS ITS OWN TARGET before starting.
 * Nothing is inherited from whatever ran last. The encoder watch stays as the
 * stroke terminator (and as a backstop if a drive ever does honour velocity
 * mode, where the target write would simply be ignored).
 *
 * Parameters are identical to ReactionMove(); cycle_delay_ms is again just a
 * pause between strokes (0 = none, which is the continuous case).
 ========================================================= */

/* THE 34000 -> 35000 CLIFF.
 * SERVO_POS_SPEED is a 16-bit register the drive treats as unsigned only up to
 * its own internal max (~34000 in this unit). Command MORE than that and the
 * drive wraps/clamps to a crawl -- which is exactly the "35000 suddenly very
 * slow" you saw. So the speed is capped just below that edge; going past it
 * makes the servo SLOWER, not faster. */
#define REACTION_SPEED_MAX        34000

/* THE REAL SPEED LEVER -- ACCELERATION / DECELERATION TIME.
 * ServoPositionConfig() leaves SERVO_POS_ACC_TIME / DEC_TIME at 15000, a very
 * long ramp: the shaft spends the whole sweep still accelerating and never
 * reaches the commanded speed, so it crawls even at speed 34000. These are now
 * set HARD DOWN so the shaft snaps up to speed almost immediately.
 *
 * TUNING (do this on the bench -- I cannot measure your drive):
 *   - If SMALLER made it faster, keep lowering toward 20-50 for max speed.
 *   - If SMALLER made it SLOWER or no different, this register is an accel
 *     RATE, not a time -- then go the OTHER way: set both to a LARGE value
 *     (e.g. 30000-60000) so the rate is high. One of the two directions is
 *     your fast setting; there is no way to know which without the drive's
 *     manual, so try the big value if this small one does not help. */
#define REACTION_FAST_ACC_TIME    50
#define REACTION_FAST_DEC_TIME    50

/* How often the encoder is sampled while a stroke is running. This is the
 * resolution of the turnaround: the shaft keeps moving for up to one poll
 * interval past the target, so a SMALLER number turns around closer to the
 * requested angle at the cost of more Modbus traffic. 10 ms is about as fast as
 * a 9-byte read/reply pair goes. */
#define REACTION_SPEED_POLL_MS    10

/* Counts of slack allowed when deciding the stroke has arrived. The drive lands
 * a count or two either side of its own target, and rounding in DegreeToPulse()
 * can leave our figure marginally higher than what it actually travels -- with
 * no slack that costs a full REACTION_MOTION_TIMEOUT of waiting for a move that
 * has already finished. */
#define REACTION_ARRIVE_SLACK     20

/* Polls with the shaft standing still that mean the drive has finished this
 * stroke early (it hit its own target, or it refused the move). Ends the stroke
 * instead of stalling until the timeout. */
#define REACTION_QUIET_POLLS      40    /* x POLL_MS = 400 ms of stillness */

/* One stroke: arm the target for THIS stroke, start it, and end the stroke when
 * the encoder says the shaft has covered |deg| degrees.
 *
 * ARMING MATTERS. The direction command re-runs whatever incremental target is
 * currently in SERVO_POSITION, so a stroke that does not write its own target
 * inherits the last one written -- by an earlier call, or an earlier RUN. That
 * is what capped every 360 deg stroke at the 180 deg left over from a previous
 * position-mode run.
 *
 * The counts are worked out exactly as ReactionStep() does it --
 * DegreeToPulse(deg) * REACTION_POS_SCALE -- so a given swing_deg produces the
 * same physical rotation in both functions. Travel is measured from where THIS
 * stroke started, so overshoot at a turnaround is not carried into the next
 * stroke and the pendulum does not walk away from its centre.
 *
 * No STOP is sent at the end: the drive has reached its own target by then and
 * the caller flips straight into the opposite direction. */
static bool ReactionSpeedStroke(ServoHandle *servo, uint8_t id, int deg,
                                const char *label, int cycle)
{
    int32_t  start = 0, pos = 0, prev = 0, travelled, delta;
    int32_t  counts, arrive;
    uint32_t t0;
    int      quiet = 0;

    if (deg == 0)
        return true;                    /* nothing to do */

    counts = (int32_t) (DegreeToPulse((float) deg) * REACTION_POS_SCALE);

    arrive = counts;
    if (arrive < 0) arrive = -arrive;
    arrive -= REACTION_ARRIVE_SLACK;
    if (arrive < 1) arrive = 1;

    if (!ServoSafeReadPosition(servo, id, READ_CURRENT_POS, &start)) {
        UART_Print("Reaction : %s -- start position read failed\r\n", label);
        return false;
    }
    prev = start;

    /* Arm THIS stroke's target, then start it. */
    ServoWrite32(servo, id, SERVO_POSITION, counts);
    osDelay(20);

    ServoWrite16(servo, id, SERVO_MOTION_CMD,
                 (deg >= 0) ? SERVO_FORWARD : SERVO_REVERSE);

    t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < (uint32_t) REACTION_MOTION_TIMEOUT) {
        osDelay(REACTION_SPEED_POLL_MS);

        if (!ServoSafeReadPosition(servo, id, READ_CURRENT_POS, &pos))
            continue;                   /* a dropped frame is not an arrival */

        travelled = pos - start;
        if (travelled < 0) travelled = -travelled;

        if (travelled >= arrive)
            return true;                /* stroke done -- caller reverses */

        /* Stopped short? Do not sit here for the whole timeout. */
        delta = pos - prev;
        if (delta < 0) delta = -delta;
        prev = pos;

        if (delta <= REACTION_POS_DEADBAND) {
            if (++quiet >= REACTION_QUIET_POLLS) {
                UART_Print("Reaction : Cycle %d  %s stopped short -- "
                           "%ld of %ld counts\r\n",
                           cycle, label, (long) travelled, (long) arrive);
                return false;
            }
        } else {
            quiet = 0;
        }
    }

    UART_Print("Reaction : Cycle %d  %s TIMEOUT -- wanted %ld counts\r\n",
               cycle, label, (long) arrive);
    return false;
}

void ReactionMoveContinuous(ServoHandle *servo, uint8_t id, int offset_deg,
        int swing_deg, uint16_t speed, int cycles, int cycle_delay_ms) {

    uint16_t v = speed;

    /* Same guards as ReactionMove(): a zero swing would spin forever waiting to
     * travel 0 counts, which in speed mode means it never stops. */
    if (swing_deg == 0) {
        UART_Print("Reaction : swing = 0 deg -- nothing to swing, "
                   "aborting.\r\n");
        return;
    }

    if (cycles <= 0) cycles = 1;

    if (v > REACTION_SPEED_MAX) {
        UART_Print("Reaction(Continuous) : speed %u past the %d cliff -- "
                   "capping.\r\n", (unsigned) v, REACTION_SPEED_MAX);
        v = REACTION_SPEED_MAX;
    }

    /* ---- Put the drive in a KNOWN state: incremental position control. ----
     * This used to write SERVO_CONTROL_MODE = SPEED_CONTROL. That does not give
     * this drive velocity control (see the note above the function): it stayed
     * in position mode and quietly re-ran whatever target SERVO_POSITION still
     * held, so strokes came out the wrong size. POSITION_CONTROL + INCREMENTAL
     * is what it actually honours, and every stroke arms its own target. */
    ServoWrite16(servo, id, FAULT_RESET,         SET_HIGH);         osDelay(50);
    ServoWrite16(servo, id, SERVO_CONTROL_MODE,  POSITION_CONTROL); osDelay(50);
    ServoWrite16(servo, id, SERVO_POS_MODE,      INCREMENTAL);      osDelay(50);

    /* Fast ramp + the speed setpoint. */
    ServoWrite16(servo, id, SERVO_POS_ACC_TIME,  REACTION_FAST_ACC_TIME); osDelay(50);
    ServoWrite16(servo, id, SERVO_POS_DEC_TIME,  REACTION_FAST_DEC_TIME); osDelay(50);
    ServoWrite16(servo, id, SERVO_POS_SPEED,     v);             osDelay(50);

    UART_Print("Reaction(Continuous) : speed %u, swing %d deg, "
               "%d cycles\r\n", (unsigned) v, swing_deg, cycles);

    /* ---- Offset move to the starting position. ---- */
    ReactionSpeedStroke(servo, id, offset_deg, "OFFSET", 0);

    /* ---- The pendulum, exactly as ReactionMove() walks it. ----
     * Prime half a swing to the LEFT so the rocking is centred on where the
     * mixer started, then alternate full swings, then a closing half swing. */
    UART_Print("Reaction : priming %+d deg to the swing extreme\r\n",
               -swing_deg / 2);
    ReactionSpeedStroke(servo, id, -swing_deg / 2, "PRIME", 0);
    if (cycle_delay_ms > 0) osDelay(cycle_delay_ms);

    for (int i = 0; i < cycles; i++) {

        UART_Print("Reaction : Cycle %d  RIGHT (%+d deg)\r\n", i, swing_deg);
        ReactionSpeedStroke(servo, id,  swing_deg, "RIGHT", i);
        if (cycle_delay_ms > 0) osDelay(cycle_delay_ms);

        UART_Print("Reaction : Cycle %d  LEFT  (%+d deg)\r\n", i, -swing_deg);
        ReactionSpeedStroke(servo, id, -swing_deg, "LEFT", i);
        if (cycle_delay_ms > 0) osDelay(cycle_delay_ms);
    }

    UART_Print("Reaction : recentring %+d deg\r\n", swing_deg / 2);
    ReactionSpeedStroke(servo, id, swing_deg / 2, "RECENTRE", cycles);

    /* ---- Final stop: the shaft must not still be turning on return. ---- */
    ServoWrite16(servo, id, SERVO_MOTION_CMD,   SERVO_STOP);        osDelay(100);
}



/* ===========DISPENSING SERVO============*/

void AminoAcidServo(ServoHandle *servo, uint8_t id)
{
	ServoWrite16(servo,
                 id,
                 SERVO_HOME_TRIGGER,
                 SERVO_HOMING_START);

    osDelay(100);

    ServoWaitMotionComplete(servo,
                            id,
                            READ_CURRENT_POS,
                            5000);      // 5 s timeout

    UART_Print("Reached\r\n");

    ServoWrite16(servo,
                 id,
                 SERVO_MOTION_CMD,
                 SERVO_STOP);

    osDelay(100);
}

/*=============DISPENSING AMINO ACID MOVEMENT=============*/

void DispenserMove(ServoHandle *servo,
                    uint8_t id,
                    int deg,
                    uint16_t speed)
{
    int32_t position;
    uint16_t direction;

    /* STOP FIRST -- this was missing vs the working C++ Dispensing_Move(),
     * and is the likely cause of "same angle no matter what I enter". The
     * C++ opens every move with SERVO_MOTION_CMD = SERVO_STOP + 300ms
     * before writing a new target. On these incremental-mode drives, if
     * the servo is still latched in its previous motion state, writing a
     * new target position without stopping/re-arming first can leave it
     * ignoring the new value and repeating the prior move. */
    ServoWrite16(servo, id, SERVO_MOTION_CMD, SERVO_STOP);
    osDelay(300);

    /* Configure Servo (must configure the servo we're actually moving) */

    ServoPositionConfig(servo, id);

    /* Convert Degree to Pulse */

    position = DegreeToPulse((float)deg);

    /* Set Target Position.
     * Scale factor is 4.5, matching the C++ Dispensing_Move() for THIS
     * (dispensing) servo -- was 7.5, which is the factor the C++ uses for
     * the mixer/index moves, not the dispenser. 7.5 made every commanded
     * angle wrong by a constant 7.5/4.5 ratio. */
    ServoWrite32(servo,
                 id,
                 SERVO_POSITION,
                 position * 4.5);

    osDelay(50);

    /* Set Speed */

    ServoWrite16(servo,
                 id,
                 SERVO_POS_SPEED,
                 speed);

    osDelay(50);

    /* Determine Direction */

    if(position >= 0)
    {
        direction = SERVO_FORWARD;
    }
    else
    {
        direction = SERVO_REVERSE;
    }

    /* Start Motion */

    ServoWrite16(servo,
                 id,
                 SERVO_MOTION_CMD,
                 direction);

    osDelay(500);

    /* Wait for motion to complete, then stop — motion does not auto-stop */

    bool done = ServoWaitMotionComplete(servo, id, READ_CURRENT_POS, REACTION_MOTION_TIMEOUT);
    osDelay(500);

    ServoWrite16(servo, id, SERVO_MOTION_CMD, SERVO_STOP);
    osDelay(500);

    if (!done)
    {
        UART_Print("Dispensing Move Timeout\r\n");
    }
}

void ServoScanIDs(ServoHandle *servo)
{
    int32_t pos;

    UART_Print("\r\n=====================================\r\n");
    UART_Print("      MODBUS DEVICE SCAN\r\n");
    UART_Print("=====================================\r\n");

    for(uint8_t id = 1; id <= 10; id++)
    {
        UART_Print("Checking ID %d... ", id);

        if(ServoSafeReadPosition(servo,
                                 id,
                                 READ_CURRENT_POS,
                                 &pos))
        {
            UART_Print("FOUND  Position = %ld\r\n", pos);
        }
        else
        {
            UART_Print("No Response\r\n");
        }

        osDelay(50);
    }

    UART_Print("\r\nScan Complete.\r\n");
}

/* =========================================================
 DISPENSER PLATE — NAMED SLOTS
 =========================================================
 * Everything below turns a slot NAME into an absolute plate angle and drives
 * the MicroID servo there with DispenserMove().
 *
 * Why the bookkeeping: ServoPositionConfig() puts the drive in INCREMENTAL
 * mode, so DispenserMove(deg) moves BY deg from wherever the plate happens to
 * be — it does not move TO an angle. Calling DispenserMove(54) twice would put
 * the plate at 108 deg, not 54. So we hold the plate's current angle here and
 * command only the difference. Every named slot is therefore an absolute
 * destination that is correct no matter which slot you came from.
 ========================================================= */

/* Plate angle in degrees from the fixed home, normalised to [0, 360). */
static float gDispenserAngle = 0.0f;

/* Drive feedback counts at the fixed home. Kept for diagnostics only — the
 * motion itself runs off gDispenserAngle, which cannot be thrown off by a
 * dropped feedback read. */
static int32_t gDispenserHomePos = 0;

/* False until home has been latched; the first slot call latches it. */
static bool gDispenserHomeSet = false;

/* Syringe position, in 36 deg steps anticlockwise from home.
 * Syringe n is at n * AMINO_SLOT_STEP_DEG. Consecutive by default — CHANGE
 * THESE NUMBERS if the syringes are not loaded in this order round the plate. */
static const uint8_t AminoAcidSlotIndex[AA_SLOT_COUNT] = {
    [AA_THREONOL]    = 0,      /*   0 deg */
    [AA_CYSTEINE]    = 1,      /*  36 deg */
    [AA_THREONINE]   = 2,      /*  72 deg */
    [AA_LYSINE]      = 3,      /* 108 deg */
    [AA_DTRAPTOC]    = 4,      /* 144 deg */
    [AA_PHENAMINE]   = 5,      /* 180 deg */
    [AA_D_PHENAMINE] = 6,      /* 216 deg */
    [AA_SLOT_8]      = 7,      /* 252 deg — 8th syringe, name not supplied */
};

static const char *const AminoAcidSlotNames[AA_SLOT_COUNT] = {
    [AA_THREONOL]    = "Threonol",
    [AA_CYSTEINE]    = "Cysteine",
    [AA_THREONINE]   = "Threonine",
    [AA_LYSINE]      = "Lysine",
    [AA_DTRAPTOC]    = "Dtraptoc",
    [AA_PHENAMINE]   = "Phenamine",
    [AA_D_PHENAMINE] = "D-Phenamine",
    [AA_SLOT_8]      = "Slot-8",
};

/* Wash slot holding each solvent bottle. Straight through -- solvent N is in
 * slot N -- so the order here is the order round the plate. The angle is
 * N * WASH_SLOT_STEP_DEG - WASH_SLOT_OFFSET_DEG: DCM sits 16 deg CLOCKWISE of
 * home, and each solvent after it is 30 deg further ANTICLOCKWISE. */
static const uint8_t WashSolventSlotIndex[WASH_SLOT_COUNT] = {
    [WASH_DCM]                =  0,  /*  -16 deg  (16 clockwise of home) */
    [WASH_SOCL2]              =  1,  /*  +14 deg */
    [WASH_DMF]                =  2,  /*  +44 deg */
    [WASH_PIPERIDINE_20_DMF]  =  3,  /*  +74 deg */
    [WASH_MEOH]               =  4,  /* +104 deg */
    [WASH_DIPEA]              =  5,  /* +134 deg */
    [WASH_HATU]               =  6,  /* +164 deg */
    [WASH_TFA]                =  7,  /* +194 deg */
    [WASH_EDT]                =  8,  /* +224 deg */
    [WASH_TIS]                =  9,  /* +254 deg */
    [WASH_H2O]                = 10,  /* +284 deg */
};

static const char *const WashSolventSlotNames[WASH_SLOT_COUNT] = {
    [WASH_DCM]                = "DCM",
    [WASH_SOCL2]              = "SOCl2",
    [WASH_DMF]                = "DMF",
    [WASH_PIPERIDINE_20_DMF]  = "20% Piperidine/DMF",
    [WASH_MEOH]               = "MeOH",
    [WASH_DIPEA]              = "DIPEA",
    [WASH_HATU]               = "HATU",
    [WASH_TFA]                = "TFA",
    [WASH_EDT]                = "EDT",
    [WASH_TIS]                = "TIS",
    [WASH_H2O]                = "H2O",
};

/* ---------------------------------------------------------
 ANGLE HELPERS
 --------------------------------------------------------- */

/* Fold any angle into [0, 360). */
static float DispenserNormalise(float deg) {
    deg = fmodf(deg, 360.0f);

    if (deg < 0.0f) {
        deg += 360.0f;
    }

    return deg;
}

/* ---------------------------------------------------------
 CAPTURE FIXED HOME
 --------------------------------------------------------- */

void ServoHomeFixedCapture(ServoHandle *servo) {
    int32_t pos = 0;

    if (servo == NULL) {
        return;
    }

    /* Feedback is informational: if the read fails we still latch home, we
     * just have no count to print. The angle bookkeeping does not need it. */
    if (!ServoSafeReadPosition(servo, DISPENSER_SLOT_ID, READ_CURRENT_POS,
                               &pos)) {
        UART_Print("Home Fixed : position read failed (using 0)\r\n");

        pos = 0;
    }

    gDispenserHomePos = pos;
    gDispenserAngle   = 0.0f;
    gDispenserHomeSet = true;

    UART_Print("Home Fixed : latched at count %ld = 0 deg\r\n",
               (long) gDispenserHomePos);
}

/* ---------------------------------------------------------
 GO TO AN ABSOLUTE PLATE ANGLE
 ---------------------------------------------------------
 * Converts the absolute target into the incremental move DispenserMove()
 * actually wants, taking the shorter way round (never more than 180 deg), and
 * updates the tracked angle. */
static bool DispenserGotoAngle(ServoHandle *servo, float target_deg,
                               const char *label) {
    float delta;
    int   move_deg;

    if (servo == NULL) {
        return false;
    }

    /* First move of the run defines home. */
    if (!gDispenserHomeSet) {
        ServoHomeFixedCapture(servo);
    }

    target_deg = DispenserNormalise(target_deg);

    delta = target_deg - gDispenserAngle;

    /* Shorter way round: keep the move within +/-180 deg. */
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    if (fabsf(delta) < DISPENSER_SLOT_EPS_DEG) {
        UART_Print("Dispenser : %s -- already at %d deg\r\n", label,
                   (int) lroundf(target_deg));

        return true;
    }

    /* DISPENSER_SLOT_DIR maps "increasing angle" onto the sign this plate
     * actually needs (the working indexing code advances with -18). */
    move_deg = (int) lroundf(delta) * DISPENSER_SLOT_DIR;

    UART_Print("Dispenser : %s -- %d deg to %d deg (move %d)\r\n", label,
               (int) lroundf(gDispenserAngle), (int) lroundf(target_deg),
               move_deg);

    DispenserMove(servo, DISPENSER_SLOT_ID, move_deg, DISPENSER_SLOT_SPEED);

    /* DispenserMove() logs its own timeout; the plate has still been commanded
     * there, so tracking follows the command. */
    gDispenserAngle = target_deg;

    return true;
}

/* ---------------------------------------------------------
 AMINO ACID SLOTS  (18 deg apart)
 --------------------------------------------------------- */

bool AminoAcidSlots(ServoHandle *servo, AminoAcidSlot_t slot) {
    float target;

    if ((int) slot < 0 || (int) slot >= AA_SLOT_COUNT) {
        UART_Print("Amino Slot : invalid slot %d\r\n", (int) slot);

        return false;
    }

    /* Catches a bad edit to AminoAcidSlotIndex[] before it turns into a move
     * to the wrong syringe. */
    if (AminoAcidSlotIndex[slot] >= AMINO_PLATE_SLOTS) {
        UART_Print("Amino Slot : %s mapped to slot %u, plate has %d\r\n",
                   AminoAcidSlotNames[slot],
                   (unsigned) AminoAcidSlotIndex[slot], AMINO_PLATE_SLOTS);

        return false;
    }

    target = (float) AminoAcidSlotIndex[slot] * AMINO_SLOT_STEP_DEG;

    return DispenserGotoAngle(servo, target, AminoAcidSlotNames[slot]);
}

/* ---------------------------------------------------------
 WASH / SOLVENT SLOTS  (12-slot plate, 30 deg apart)
 --------------------------------------------------------- */

bool wash_solvents(ServoHandle *servo, WashSolventSlot_t slot) {
    float target;

    if ((int) slot < 0 || (int) slot >= WASH_SLOT_COUNT) {
        UART_Print("Wash Slot : invalid slot %d\r\n", (int) slot);

        return false;
    }

    /* Catches a bad edit to WashSolventSlotIndex[] before it turns into a move
     * to the wrong bottle. */
    if (WashSolventSlotIndex[slot] >= WASH_PLATE_SLOTS) {
        UART_Print("Wash Slot : %s mapped to slot %u, plate has %d\r\n",
                   WashSolventSlotNames[slot],
                   (unsigned) WashSolventSlotIndex[slot], WASH_PLATE_SLOTS);

        return false;
    }

    /* Evenly spaced 30 deg ring, shifted so that DCM (n = 0) sits
     * WASH_SLOT_OFFSET_DEG (16 deg) CLOCKWISE of home -- a NEGATIVE angle in
     * this frame -- and every solvent after it is one 30 deg pitch further
     * ANTICLOCKWISE:
     *
     *      DCM -16, SOCl2 +14, DMF +44, ... H2O +284
     *
     * These are ABSOLUTE positions measured from home, not steps from the
     * previous solvent, which is what keeps them right regardless of which
     * solvent ran last. The 30 deg figure is the DCM->SOCl2 spacing; because
     * wash_solvent_selection() homes before every solvent, the move actually
     * commanded for SOCl2 is home->+14, not 30.
     *
     * DispenserGotoAngle() normalises the -16 to 344 and then takes the
     * shorter way round, so DCM really does turn 16 deg clockwise off home
     * rather than 344 deg the other way. */
    target = ((float) WashSolventSlotIndex[slot] * WASH_SLOT_STEP_DEG)
             - WASH_SLOT_OFFSET_DEG;

    return DispenserGotoAngle(servo, target, WashSolventSlotNames[slot]);
}

/* ---------------------------------------------------------
 RETURN TO FIXED HOME
 ---------------------------------------------------------
 * "Home" is the position the plate was standing in when it was latched -- at
 * start-up, before it had turned at all. Calling this at any point brings the
 * plate back to EXACTLY that position.
 *
 * This delegates to DispenserMoveHome() so there is only ONE notion of home in
 * the system. It used to return to angle 0 through the software tracker, which
 * is not the same thing: the tracker accumulates commanded angles, so any
 * rounding or a move that did not complete left "angle 0" a little away from
 * the position actually latched. DispenserMoveHome() drives to the latched
 * ENCODER COUNT and verifies arrival, so repeated homing cannot drift. */

bool servo_home_fixed(ServoHandle *servo) {
    return DispenserMoveHome(servo);
}

/* ---------------------------------------------------------
 QUERY / NAMES
 --------------------------------------------------------- */

float ServoDispenserAngle(void) {
    return gDispenserAngle;
}

const char *AminoAcidSlotName(AminoAcidSlot_t slot) {
    if ((int) slot < 0 || (int) slot >= AA_SLOT_COUNT) {
        return "?";
    }

    return AminoAcidSlotNames[slot];
}

const char *WashSolventSlotName(WashSolventSlot_t slot) {
    if ((int) slot < 0 || (int) slot >= WASH_SLOT_COUNT) {
        return "?";
    }

    return WashSolventSlotNames[slot];
}

/* =========================================================
 REACTION (MIXER) SERVO — PARK AT HOME
 =========================================================
 * Drives MIXER_ID to an ABSOLUTE encoder count.
 *
 * The drive is in INCREMENTAL mode, so "go to count N" is not a thing you can
 * command directly: SERVO_POSITION is a distance, not a destination. So read
 * where the mixer actually is (READ_CURRENT_POS), work out the delta, and
 * command that. delta is already in the drive's own count units -- no
 * DegreeToPulse() and no DISPENSER_POS_SCALE here, unlike the degree-based
 * moves above.
 *
 * This is the same read/delta/trigger/wait pattern RobotMotion.c already uses
 * to park the mixer after the parallel reaction; putting it here makes it
 * callable from any file, and adds an explicit mode/speed setup plus an
 * arrival check so the result does not depend on what ran beforehand.
 *
 * BUS SHARING: the mixer sits on the same USART6 bus as the rail and carousel.
 * If you call this from a context where another task may also be talking to
 * that bus, wrap the call in the caller's own bus lock -- exactly as
 * RobotMotion.c wraps its park in servoBusLock()/servoBusUnlock(). This
 * function does not take that lock itself, because the lock is private to
 * RobotMotion.c and not every caller needs it.
 ========================================================= */

bool ReactionMoveToPulses(ServoHandle *servo, int32_t target_pulses) {
    int32_t  current = 0;
    int32_t  final = 0;
    int32_t  delta;
    int32_t  error;
    uint16_t direction;
    bool     done;

    if (servo == NULL) {
        return false;
    }

    if (!ServoSafeReadPosition(servo, MIXER_ID, READ_CURRENT_POS, &current)) {
        UART_Print("Reaction Home : position read failed -- park skipped\r\n");

        return false;
    }

    delta = target_pulses - current;

    UART_Print("Reaction Home : %ld -> %ld pulses (delta %ld)\r\n",
               (long) current, (long) target_pulses, (long) delta);

    if (delta == 0) {
        UART_Print("Reaction Home : already parked\r\n");

        return true;
    }

    /* Stop first so a new target is never stacked on a move still in progress
     * -- the same guard ReactionStep() and DispenserMove() open with. */
    ServoWrite16(servo, MIXER_ID, SERVO_MOTION_CMD, SERVO_STOP);
    osDelay(100);

    /* Make the two assumptions this move relies on explicit, rather than
     * inheriting them from whatever ran last: the delta is only meaningful in
     * INCREMENTAL mode, and the speed decides whether the move finishes inside
     * REACTION_HOME_TIMEOUT_MS. Writing INCREMENTAL when it is already
     * INCREMENTAL is a no-op. */
    ServoWrite16(servo, MIXER_ID, SERVO_POS_MODE, INCREMENTAL);
    osDelay(50);

    ServoWrite16(servo, MIXER_ID, SERVO_POS_SPEED, REACTION_HOME_SPEED);
    osDelay(50);

    ServoWrite32(servo, MIXER_ID, SERVO_POSITION, delta);
    osDelay(50);

    direction = (delta >= 0) ? SERVO_FORWARD : SERVO_REVERSE;

    ServoWrite16(servo, MIXER_ID, SERVO_MOTION_CMD, direction);
    osDelay(20);

    done = ServoWaitMotionComplete(servo, MIXER_ID, READ_CURRENT_POS,
                                   REACTION_HOME_TIMEOUT_MS);

    /* Stop before returning so the shaft is genuinely at rest. */
    ServoWrite16(servo, MIXER_ID, SERVO_MOTION_CMD, SERVO_STOP);
    osDelay(20);

    if (!done) {
        UART_Print("Reaction Home : motion TIMEOUT\r\n");

        return false;
    }

    /* Confirm it really landed there. A dropped frame on this shared bus would
     * otherwise leave the mixer somewhere else with nothing to show for it. */
    if (!ServoSafeReadPosition(servo, MIXER_ID, READ_CURRENT_POS, &final)) {
        UART_Print("Reaction Home : arrival read failed\r\n");

        return false;
    }

    error = final - target_pulses;

    if (error < 0) {
        error = -error;
    }

    if (error > REACTION_HOME_TOLERANCE) {
        UART_Print("Reaction Home : MISSED -- at %ld, wanted %ld (off by %ld)\r\n",
                   (long) final, (long) target_pulses, (long) error);

        return false;
    }

    UART_Print("Reaction Home : parked at %ld pulses\r\n", (long) final);

    return true;
}

bool ReactionMoveHome(ServoHandle *servo) {
    return ReactionMoveToPulses(servo, REACTION_HOME_PULSES);
}

/* =========================================================
 DISPENSER (MicroID) SERVO — PARK AT HOME
 =========================================================
 * Same read/delta/trigger/wait shape as the mixer park above, on MicroID.
 *
 * The delta is written to SERVO_POSITION in RAW encoder counts -- no
 * DegreeToPulse() and no DISPENSER_POS_SCALE. That is deliberate and is what
 * makes this different from DispenserMove(): the target and the feedback are
 * both already in the drive's own count domain (READ_CURRENT_POS returns a
 * running total in the scaled command domain), so scaling the difference again
 * would overshoot by a factor of 4.5.
 *
 * BUS SHARING: like the mixer park, this does not take RobotMotion.c's servo
 * bus lock -- that lock is private to that file. Callers that share the bus
 * must wrap this themselves.
 ========================================================= */

bool DispenserMoveToPulses(ServoHandle *servo, int32_t target_pulses) {
    int32_t  current = 0;
    int32_t  final = 0;
    int32_t  delta;
    int32_t  error;
    uint16_t direction;
    bool     done;

    if (servo == NULL) {
        return false;
    }

    if (!ServoSafeReadPosition(servo, DISPENSER_SLOT_ID, READ_CURRENT_POS,
                               &current)) {
        UART_Print("Dispenser Home : position read failed -- park skipped\r\n");

        return false;
    }

    delta = target_pulses - current;

    UART_Print("Dispenser Home : %ld -> %ld pulses (delta %ld)\r\n",
               (long) current, (long) target_pulses, (long) delta);

    if (delta == 0) {
        UART_Print("Dispenser Home : already parked\r\n");

        return true;
    }

    /* Stop first so a new target is never stacked on a move in progress -- the
     * same guard DispenserMove() opens with. */
    ServoWrite16(servo, DISPENSER_SLOT_ID, SERVO_MOTION_CMD, SERVO_STOP);
    osDelay(300);

    /* The delta is only meaningful in INCREMENTAL mode, and the speed decides
     * whether the move finishes inside DISPENSER_HOME_TIMEOUT_MS. Writing
     * INCREMENTAL when it is already INCREMENTAL is a no-op. */
    ServoWrite16(servo, DISPENSER_SLOT_ID, SERVO_POS_MODE, INCREMENTAL);
    osDelay(50);

    ServoWrite16(servo, DISPENSER_SLOT_ID, SERVO_POS_SPEED,
                 DISPENSER_HOME_SPEED);
    osDelay(50);

    ServoWrite32(servo, DISPENSER_SLOT_ID, SERVO_POSITION, delta);
    osDelay(50);

    direction = (delta >= 0) ? SERVO_FORWARD : SERVO_REVERSE;

    ServoWrite16(servo, DISPENSER_SLOT_ID, SERVO_MOTION_CMD, direction);
    osDelay(20);

    done = ServoWaitMotionComplete(servo, DISPENSER_SLOT_ID, READ_CURRENT_POS,
                                   DISPENSER_HOME_TIMEOUT_MS);

    /* Stop before returning so the plate is genuinely at rest. */
    ServoWrite16(servo, DISPENSER_SLOT_ID, SERVO_MOTION_CMD, SERVO_STOP);
    osDelay(20);

    if (!done) {
        UART_Print("Dispenser Home : motion TIMEOUT\r\n");

        return false;
    }

    /* Confirm it really landed there. A dropped frame on this shared bus would
     * otherwise leave the plate somewhere else with nothing to show for it. */
    if (!ServoSafeReadPosition(servo, DISPENSER_SLOT_ID, READ_CURRENT_POS,
                               &final)) {
        UART_Print("Dispenser Home : arrival read failed\r\n");

        return false;
    }

    error = final - target_pulses;

    if (error < 0) {
        error = -error;
    }

    if (error > DISPENSER_HOME_TOLERANCE) {
        UART_Print("Dispenser Home : MISSED -- at %ld, wanted %ld (off by %ld)\r\n",
                   (long) final, (long) target_pulses, (long) error);

        return false;
    }

    UART_Print("Dispenser Home : parked at %ld pulses\r\n", (long) final);

    return true;
}

/* Home is the position the plate was standing in when it was first latched --
 * i.e. where it was before the servo started rotating -- NOT a compile-time
 * constant. The latched value is an absolute encoder count, so returning to it
 * is exact and cannot drift no matter how many slot moves happen in between.
 *
 * First call after power-up (or after ServoHomeFixedCapture()): the plate has
 * not been turned by any of these helpers yet, so wherever it stands IS home.
 * Latch it and return without moving -- driving somewhere would defeat the
 * point of declaring the start position as home.
 *
 * Every later call: drive back to that exact count and re-datum the angle
 * tracker, since this position is 0 deg by definition and every named-slot
 * angle is measured from it. */
bool DispenserMoveHome(ServoHandle *servo) {
    if (servo == NULL) {
        return false;
    }

    if (!gDispenserHomeSet) {
        UART_Print("Dispenser Home : not yet latched -- declaring the current "
                   "position as home\r\n");

        ServoHomeFixedCapture(servo);

        return true;
    }

    if (!DispenserMoveToPulses(servo, gDispenserHomePos)) {
        return false;
    }

    gDispenserAngle = 0.0f;

    UART_Print("Dispenser Home : back at the latched home (%ld pulses), "
               "angle tracker re-datumed to 0 deg\r\n", (long) gDispenserHomePos);

    return true;
}
