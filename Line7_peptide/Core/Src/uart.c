/*
 * uart.c
 *
 *  Created on: 02-Jul-2026
 *      Author: Electronics Dept
 *
 *  NOTE: the actual HAL_UART_Transmit() calls in UART_Print()/
 *  UART_ReadLine()/UART_ReadString() were commented out (along with
 *  consoleTx() itself and the consoleMutexHandle it used), which meant
 *  every UART_Print() call in the whole project -- Load.c, Machinemenu.c,
 *  servo.c, etc. -- silently formatted its string into a local buffer and
 *  then went nowhere. Restored below.
 *
 *  consoleMutexHandle is NOT brought back: it existed only to guard
 *  against RobotRxTask (a background task that used to print to this same
 *  UART) racing the console's own prints. RobotRxTask no longer exists
 *  (see freertos.c -- fully removed), so there is currently only one
 *  writer of USART3 (the console/menu task itself), and no race to guard
 *  against. If a second task ever prints to USART3 again in the future,
 *  a mutex will need to come back here (and be created in freertos.c).
 */

#include "uart.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdlib.h"
#include "stdbool.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"

static void consoleTx(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart3, data, len, HAL_MAX_DELAY);
}

void UART_Print(const char *fmt, ...)
{
    /* 1024, raised from 512: vsnprintf() silently truncates anything longer
     * than this buffer, and the STEPPER MENU (with the motor-2 option added)
     * grew to ~516 characters as a single concatenated string literal -- one
     * char over 512, so the "Enter choice" prompt was being cut to "Enter ch"
     * with no error. 1024 gives headroom for that menu plus future text.
     *
     * NOTE: this buffer is on the STACK, so every UART_Print() call now uses
     * ~1 KB of its caller's task stack transiently. Fine given the enlarged
     * task stacks, but keep it in mind if a task with a small stack prints. */
    char buffer[1024];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    consoleTx((uint8_t *)buffer, strlen(buffer));
}

/* Which character ended the LAST line read from the console.
 *
 * A terminal that sends CRLF leaves the LF sitting in the UART after the CR
 * that ended the previous line. The next read then takes that LF as its first
 * character, returns an EMPTY line, and every answer the user types lands on
 * the FOLLOWING prompt -- a silent one-prompt shift. That is how the Reaction
 * Move menu ended up running with swing = 0: the swing the user typed was
 * eaten by the previous prompt.
 *
 * So remember the terminator and drop its partner if it turns up as the first
 * character of the next line. A bare Enter still returns an empty line, so
 * "press Enter to keep the default" keeps working. Shared by UART_ReadLine()
 * and UART_ReadString() because menus mix the two on the same UART. */
static uint8_t last_terminator = 0;

static bool IsStrayNewlinePair(uint8_t ch)
{
    return (ch == '\n' && last_terminator == '\r') ||
           (ch == '\r' && last_terminator == '\n');
}

void UART_ReadLine(char *buffer, uint16_t length)
{
    uint8_t ch;
    uint16_t index = 0;

    memset(buffer, 0, length);

    while (index < (length - 1))
    {
        if (HAL_UART_Receive(&huart3, &ch, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            /* Second half of the previous line's CRLF -- not a real Enter. */
            if (index == 0 && IsStrayNewlinePair(ch))
            {
                last_terminator = 0;
                continue;
            }

            /* Echo the received character */
            consoleTx(&ch, 1);

            /* End of line */
            if (ch == '\r' || ch == '\n')
            {
                uint8_t newline[] = "\r\n";
                consoleTx(newline, sizeof(newline) - 1);
                last_terminator = ch;
                break;
            }

            last_terminator = 0;

            /* Handle Backspace */
            if ((ch == '\b' || ch == 0x7F) && index > 0)
            {
                index--;

                uint8_t bs[] = "\b \b";
                consoleTx(bs, sizeof(bs) - 1);
            }
            else if (ch != '\b' && ch != 0x7F)
            {
                buffer[index++] = (char)ch;
            }
        }
    }

    buffer[index] = '\0';
}


void UART_ReadString(char *buffer, uint16_t maxLength)
{
    uint16_t i = 0;
    uint8_t ch;

    while (i < (maxLength - 1))
    {
        HAL_UART_Receive(&huart3, &ch, 1, HAL_MAX_DELAY);

        /* Second half of the previous line's CRLF -- see last_terminator. */
        if (i == 0 && IsStrayNewlinePair(ch))
        {
            last_terminator = 0;
            continue;
        }

        if (ch == '\r' || ch == '\n')
        {
            last_terminator = ch;
            break;
        }

        last_terminator = 0;

        buffer[i++] = ch;

        // Echo character back to terminal
        consoleTx(&ch, 1);
    }

    buffer[i] = '\0';

    // Print a new line
    char newline[] = "\r\n";
    consoleTx((uint8_t *)newline, (uint16_t)strlen(newline));
}

