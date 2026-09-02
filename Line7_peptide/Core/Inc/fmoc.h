/*
 * fmoc.h -- Vapourtec Fmoc UV detector on USART6 (RS232 via a MAX3232).
 *
 * Direct port of the vendor's Fmoc.py. USART6 is already 19200 8N1
 * (PG14 = TX, PG9 = RX) from MX_USART6_UART_Init(), which matches the Python's
 * serial.Serial(..., 19200, EIGHTBITS, PARITY_NONE, STOPBITS_ONE).
 *
 * Loop, exactly as the Python does it:
 *      flush RX  ->  write "GA\r\n"  ->  read one line  ->  print  ->  250 ms
 */

#ifndef INC_FMOC_H_
#define INC_FMOC_H_

#include "stdint.h"

/* Runs the Python's `while True` loop on the console. Does not return. */
void FmocRun(void);

/* Sends nothing -- just prints whatever arrives on USART6 to the console, as it
 * arrives. CR/LF pass through so lines break naturally; anything else that is
 * not printable is shown as <XX> so it cannot go by invisibly. Does not
 * return. */
void FmocMonitor(void);

/* Interactive terminal on USART6.
 *
 * Prompts for a command, sends it with the CR+LF the detector expects, and
 * prints whatever comes back for FMOC_REPLY_WINDOW_MS -- printable bytes as
 * characters, everything else as <XX> hex, so a reply that ends differently
 * than expected still shows up. Type "exit" (or "quit") to leave. */
void FmocSendReceive(void);

/* TX visibility test.
 *
 * "GA\r\n" is 4 bytes -- at 19200 baud that is ~2 ms of line activity every
 * 250 ms, a 0.8% duty cycle, which no TX LED shows to the naked eye. This
 * transmits back-to-back for `seconds` so the line is busy essentially 100% of
 * the time and the LED is unmistakably lit. If it STILL does not light, PG14
 * is not reaching the transceiver's input. Reports the HAL result too, so a
 * UART that refuses to transmit is distinguished from one that transmits into
 * a disconnected wire. */
void FmocTxTest(uint32_t seconds);

/* Baud sweep.
 *
 * Garbled bytes have exactly two causes: the bit timing is wrong, or the
 * levels are marginal. This separates them. It sends "GA\r\n" at each common
 * baud rate and dumps what comes back, with a count of how many bytes were
 * printable ASCII.
 *
 *   One rate returns clean, readable text -> the detector is at THAT rate, or
 *   USART6's clock is not what CubeMX assumed. Either way it is a number to fix.
 *
 *   Every rate returns junk, and the byte COUNT varies run to run -> not a
 *   timing problem at all. That is an electrical fault, and by far the most
 *   common one is a missing common ground between the transceiver and the
 *   detector: RS232 is single-ended and referenced to DB9 pin 5, so without it
 *   the receiver slices noise.
 *
 * USART6 is put back to 19200 8N1 before returning. */
void FmocBaudScan(void);

/* Is anything actually DRIVING the RX pin, or is it floating?
 *
 * Briefly takes PG9 away from USART6, reads it with an internal pull-down and
 * then with a pull-up, and hands it back. A pin held HIGH against the pull-down
 * is being driven by a live transceiver. A pin that just follows whichever
 * resistor is enabled is connected to nothing -- which is what noise-shaped
 * "replies" mean. Restores the USART6 pin configuration before returning. */
void FmocRxDriveTest(void);

#endif /* INC_FMOC_H_ */
