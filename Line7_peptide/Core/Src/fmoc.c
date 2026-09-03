/*
 * fmoc.c -- Vapourtec Fmoc UV detector on USART6. Port of Fmoc.py.
 */

#include "fmoc.h"
#include "usart.h"
#include "uart.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"

#define FMOC_UART      (&huart6)
#define FMOC_LINE_LEN   128
#define FMOC_TIMEOUT_MS 1000u    /* serial.Serial(timeout=1) */
#define FMOC_PERIOD_MS   250u    /* time.sleep(0.25)         */

/* ser.reset_input_buffer() */
static void fmocFlush(void)
{
	uint8_t drop;

	while (HAL_UART_Receive(FMOC_UART, &drop, 1, 0) == HAL_OK) {
		/* discard */
	}
}

/* ser.readline() -- reads up to the newline, or gives up after the timeout.
 * Returns the length, 0 if nothing arrived. Already .strip()ped: leading
 * CR/LF left over from the previous reply are skipped and the terminator is
 * not stored. */
static int fmocReadLine(char *buf, int len)
{
	uint32_t start = HAL_GetTick();
	int      index = 0;
	uint8_t  ch;

	buf[0] = '\0';

	while ((HAL_GetTick() - start) < FMOC_TIMEOUT_MS) {
		if (HAL_UART_Receive(FMOC_UART, &ch, 1, 20) != HAL_OK) {
			continue;
		}

		if (ch == '\r' || ch == '\n') {
			if (index == 0) {
				continue;            /* tail of the previous line */
			}

			break;
		}

		if (index < (len - 1)) {
			buf[index++] = (char) ch;
		}
	}

	buf[index] = '\0';

	return index;
}

/* response.split() then v.strip(',') -- splitting on whitespace AND commas is
 * the same thing for this reply, and also copes if the detector drops the
 * spaces. Returns how many fields were found, up to `max`. */
static int fmocSplit(char *work, char *values[], int max)
{
	char *tok = strtok(work, " \t,");
	int   n   = 0;

	while (tok != NULL && n < max) {
		values[n++] = tok;
		tok = strtok(NULL, " \t,");
	}

	return n;
}

/* f"{value/divisor:.Nf}" -- done with integers because the project links
 * --specs=nano.specs, whose printf has no floating point. */
static void fmocFormat(char *buf, int len, long raw, long divisor, int decimals)
{
	const char *sign = "";

	if (raw < 0) {
		sign = "-";
		raw  = -raw;
	}

	if (decimals == 6) {
		snprintf(buf, len, "%s%ld.%06ld", sign, raw / divisor, raw % divisor);
	} else {
		snprintf(buf, len, "%s%ld.%03ld", sign, raw / divisor, raw % divisor);
	}
}

void FmocRun(void)
{
	char  response[FMOC_LINE_LEN];
	char  work[FMOC_LINE_LEN];
	char *values[8];

	UART_Print("\r\nConnected to USART6 (19200 8N1)\r\n");
	UART_Print("365AU\t\t460AU\t\tTEMP\r\n");

	for (;;) {
		/* Clear old data */
		fmocFlush();

		/* Send GA command followed by CR + LF */
		HAL_UART_Transmit(FMOC_UART, (uint8_t *) "GA\r\n", 4, 100);

		/* Read response */
		if (fmocReadLine(response, sizeof(response)) > 0) {
			int count;

			strncpy(work, response, sizeof(work) - 1);
			work[sizeof(work) - 1] = '\0';

			count = fmocSplit(work, values, 8);

			if (count >= 5) {
				char *end365;
				char *end460;
				char *endTemp;

				long raw365 = strtol(values[2], &end365,  10);
				long raw460 = strtol(values[3], &end460,  10);
				long rawT   = strtol(values[4], &endTemp, 10);

				if (*end365 != '\0' || *end460 != '\0' || *endTemp != '\0') {
					/* except ValueError */
					UART_Print("Could not parse: %s\r\n", response);
				} else {
					char a365[24];
					char a460[24];
					char temp[24];

					fmocFormat(a365, sizeof(a365), raw365, 1000000, 6);
					fmocFormat(a460, sizeof(a460), raw460, 1000000, 6);
					fmocFormat(temp, sizeof(temp), rawT,      1000, 3);

					UART_Print("%s\t%s\t%s\r\n", a365, a460, temp);
				}
			} else {
				UART_Print("Unexpected response: %s\r\n", response);
			}
		}

		/* Send command approximately once per second */
		osDelay(FMOC_PERIOD_MS);
	}
}

/* How long to listen for a reply after a command in FmocSendReceive(). */
#define FMOC_REPLY_WINDOW_MS  10000u

void FmocSendReceive(void)
{
	char cmd[64];

	UART_Print("\r\n=== USART6 TERMINAL (19200 8N1) ===\r\n"
	           "Type a command and press Enter -- CR+LF is added for you.\r\n"
	           "\"GA\" is the reading command. Type \"exit\" to leave.\r\n");

	for (;;) {
		uint32_t start;
		uint32_t count = 0;
		uint8_t  ch;

		UART_Print("\r\nUSART6> ");
		UART_ReadLine(cmd, sizeof(cmd));

		if (cmd[0] == '\0') {
			continue;               /* bare Enter -- just re-prompt */
		}

		if (strcmp(cmd, "exit") == 0 || strcmp(cmd, "quit") == 0) {
			UART_Print("[FMOC] Leaving the terminal.\r\n");

			return;
		}

		fmocFlush();

		HAL_UART_Transmit(FMOC_UART, (uint8_t *) cmd,
		                  (uint16_t) strlen(cmd), 100);
		HAL_UART_Transmit(FMOC_UART, (uint8_t *) "\r\n", 2, 100);

		UART_Print("[TX] %s\r\n[RX] ", cmd);

		start = HAL_GetTick();

		while ((HAL_GetTick() - start) < FMOC_REPLY_WINDOW_MS) {
			if (HAL_UART_Receive(FMOC_UART, &ch, 1, 20) != HAL_OK) {
				continue;
			}

			count++;

			/* Printable as-is, CR/LF pass through so lines break naturally,
			 * anything else as hex so it cannot go by invisibly. */
			if (ch == '\r' || ch == '\n' || (ch >= 0x20u && ch < 0x7Fu)) {
				UART_Print("%c", (char) ch);
			} else {
				UART_Print("<%02X>", (unsigned) ch);
			}
		}

		if (count == 0u) {
			UART_Print("(no reply in %lu ms)",
			           (unsigned long) FMOC_REPLY_WINDOW_MS);
		}

		UART_Print("\r\n[RX] %lu byte(s).\r\n", (unsigned long) count);
	}
}

void FmocTxTest(uint32_t seconds)
{
	/* 64 printable bytes ~= 33 ms of line time at 19200 baud, sent with no gap
	 * so the TX line is continuously busy. */
	static const char pattern[] =
		"UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU";

	uint32_t          start  = HAL_GetTick();
	uint32_t          blocks = 0;
	uint32_t          fails  = 0;
	HAL_StatusTypeDef last   = HAL_OK;

	UART_Print("\r\n[FMOC] Transmitting continuously on USART6 (PG14) for "
	           "%lu s -- the module's TX LED must light up solid.\r\n",
	           (unsigned long) seconds);

	while ((HAL_GetTick() - start) < (seconds * 1000u)) {
		last = HAL_UART_Transmit(FMOC_UART, (uint8_t *) pattern,
		                         (uint16_t) (sizeof(pattern) - 1u), 200);

		if (last != HAL_OK) {
			fails++;
		}

		blocks++;
	}

	UART_Print("[FMOC] Sent %lu block(s) of %u bytes, %lu failed. "
	           "Last HAL status %d.\r\n",
	           (unsigned long) blocks, (unsigned) (sizeof(pattern) - 1u),
	           (unsigned long) fails, (int) last);

	if (fails > 0u) {
		UART_Print("[FMOC] USART6 REFUSED to transmit -- the fault is on the "
		           "MCU side, not the wiring.\r\n");
	} else {
		UART_Print("[FMOC] USART6 transmitted fine. If the TX LED stayed dark, "
		           "PG14 is not connected to the transceiver's TTL input "
		           "(or that header pin is the module's output, not its "
		           "input).\r\n");
	}
}

/* USART6's pins, as MX_USART6_UART_Init()'s MSP sets them up:
 * PG14 = TX, PG9 = RX. */
#define FMOC_RX_PORT   GPIOG
#define FMOC_RX_PIN    GPIO_PIN_9

/* Read PG9 as a plain input with `pull` applied, after letting it settle. */
static GPIO_PinState fmocRxLevelWithPull(uint32_t pull)
{
	GPIO_InitTypeDef init = { 0 };

	init.Pin   = FMOC_RX_PIN;
	init.Mode  = GPIO_MODE_INPUT;
	init.Pull  = pull;
	init.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(FMOC_RX_PORT, &init);

	osDelay(5);   /* let the ~40k internal resistor settle the line */

	return HAL_GPIO_ReadPin(FMOC_RX_PORT, FMOC_RX_PIN);
}

void FmocRxDriveTest(void)
{
	GPIO_InitTypeDef init = { 0 };
	GPIO_PinState    pulledDown;
	GPIO_PinState    pulledUp;

	UART_Print("\r\n[FMOC] Taking PG9 off USART6 briefly to see whether "
	           "anything is driving it...\r\n");

	pulledDown = fmocRxLevelWithPull(GPIO_PULLDOWN);
	pulledUp   = fmocRxLevelWithPull(GPIO_PULLUP);

	/* Hand PG9 back to USART6. */
	init.Pin       = FMOC_RX_PIN;
	init.Mode      = GPIO_MODE_AF_PP;
	init.Pull      = GPIO_NOPULL;
	init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	init.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(FMOC_RX_PORT, &init);

	UART_Print("[FMOC] PG9 with pull-down: %s, with pull-up: %s\r\n",
	           (pulledDown == GPIO_PIN_SET) ? "HIGH" : "LOW",
	           (pulledUp   == GPIO_PIN_SET) ? "HIGH" : "LOW");

	if (pulledDown == GPIO_PIN_SET) {
		UART_Print("[FMOC] -> DRIVEN HIGH. A live transceiver output is on "
		           "PG9, which is what an idle RS232 line should look like. "
		           "The wiring INTO the MCU is good.\r\n");
	} else if (pulledUp == GPIO_PIN_SET) {
		UART_Print("[FMOC] -> FLOATING. PG9 just follows whichever internal "
		           "resistor is on, so NOTHING is connected to it: no wire, "
		           "the wrong pin on the SP3232EB, or that module has no "
		           "power. The garbage 'replies' are this pin picking up "
		           "noise.\r\n");
	} else {
		UART_Print("[FMOC] -> DRIVEN LOW. Something is actively holding PG9 "
		           "low -- the opposite of an idle RS232 line. PG9 is "
		           "probably on the wrong pin of the module.\r\n");
	}
}

/* Re-open USART6 at `baud`, everything else unchanged. */
static bool fmocSetBaud(uint32_t baud)
{
	HAL_UART_DeInit(FMOC_UART);

	FMOC_UART->Init.BaudRate = baud;

	if (HAL_UART_Init(FMOC_UART) != HAL_OK) {
		UART_Print("[FMOC] Could not re-open USART6 at %lu baud.\r\n",
		           (unsigned long) baud);

		return false;
	}

	osDelay(20);

	return true;
}

void FmocBaudScan(void)
{
	static const uint32_t rates[] = {
		2400u, 4800u, 9600u, 19200u, 38400u, 57600u, 115200u
	};

	UART_Print("\r\n=== USART6 BAUD SWEEP ===\r\n"
	           "Sending \"GA\" + CR LF at each rate and dumping the reply.\r\n"
	           "The rate that returns READABLE TEXT is the detector's.\r\n");

	for (unsigned r = 0; r < (sizeof(rates) / sizeof(rates[0])); r++) {
		uint8_t  ch;
		uint32_t start;
		uint32_t total     = 0;
		uint32_t printable = 0;

		if (!fmocSetBaud(rates[r])) {
			continue;
		}

		/* Drop anything left over from the previous rate. */
		while (HAL_UART_Receive(FMOC_UART, &ch, 1, 0) == HAL_OK) {
			/* discard */
		}

		HAL_UART_Transmit(FMOC_UART, (uint8_t *) "GA\r\n", 4, 100);

		UART_Print("\r\n%6lu baud : ", (unsigned long) rates[r]);

		start = HAL_GetTick();

		while ((HAL_GetTick() - start) < 400u) {
			if (HAL_UART_Receive(FMOC_UART, &ch, 1, 20) != HAL_OK) {
				continue;
			}

			total++;

			if (ch >= 0x20u && ch < 0x7Fu) {
				printable++;
				UART_Print("%c", (char) ch);
			} else if (ch == '\r' || ch == '\n') {
				UART_Print(".");
			} else {
				UART_Print("<%02X>", (unsigned) ch);
			}
		}

		if (total == 0u) {
			UART_Print("(nothing)");
		}

		UART_Print("\r\n         %lu byte(s), %lu printable\r\n",
		           (unsigned long) total, (unsigned long) printable);
	}

	/* Back to the rate the detector is documented at. */
	(void) fmocSetBaud(19200u);

	UART_Print("\r\n=== sweep done, USART6 back at 19200 8N1 ===\r\n"
	           "All rates junk, with the byte count varying each time? Then it "
	           "is NOT baud -- check the common ground to DB9 pin 5.\r\n");
}

void FmocMonitor(void)
{
	uint8_t ch;

	UART_Print("\r\n=== USART6 MONITOR (19200 8N1) ===\r\n"
	           "Printing everything that arrives. Nothing is sent.\r\n"
	           "Reset the board to stop.\r\n\r\n");

	for (;;) {
		if (HAL_UART_Receive(FMOC_UART, &ch, 1, 100) != HAL_OK) {
			continue;
		}

		if (ch == '\r' || ch == '\n' || (ch >= 0x20u && ch < 0x7Fu)) {
			UART_Print("%c", (char) ch);
		} else {
			UART_Print("<%02X>", (unsigned) ch);
		}
	}
}
