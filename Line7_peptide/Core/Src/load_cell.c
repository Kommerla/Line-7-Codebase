/*
 * load_cell.c
 *
 *  Created on: 15-Jul-2026
 *      Author: ESE
 */
#include "uart.h"
#include "gpio.h"
#include "main.h"
#include "stepper.h"
#include "cmsis_os.h"
#include "Load.h"
static void LoadCellComputeCRC(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
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

    /* Modbus sends CRC Low Byte first, High Byte second */
    buf[len]     = crc & 0xFF;
    buf[len + 1] = (crc >> 8) & 0xFF;
}


bool LoadCellReadTest(LoadCellHandle *loadcell)
{
    uint8_t tx[8];
    uint8_t rx[7];
    HAL_StatusTypeDef status;

    /* Read Holding Register 0x0000 */
    tx[0] = 0x01;
    tx[1] = 0x03;
    tx[2] = 0x00;
    tx[3] = 0x00;
    tx[4] = 0x00;
    tx[5] = 0x01;

    LoadCellComputeCRC(tx, 6);

    UART_Print("\r\n========== LOADCELL TEST ==========\r\n");

    UART_Print("TX : ");
    for(int i = 0; i < 8; i++)
        UART_Print("%02X ", tx[i]);
    UART_Print("\r\n");

    __HAL_UART_CLEAR_OREFLAG(loadcell->huart);
    __HAL_UART_FLUSH_DRREGISTER(loadcell->huart);

    HAL_UART_Transmit(loadcell->huart, tx, 8, 1000);

    UART_Print("Waiting for reply...\r\n");

    memset(rx, 0, sizeof(rx));

    for(int i = 0; i < 7; i++)
    {
        status = HAL_UART_Receive(loadcell->huart, &rx[i], 1, 1000);

        if(status == HAL_OK)
        {
            UART_Print("RX[%d] = %02X\r\n", i, rx[i]);
        }
        else
        {
            UART_Print("Timeout at byte %d\r\n", i);

            UART_Print("UART ISR = 0x%08lX\r\n",
                       loadcell->huart->Instance->ISR);

            return false;
        }
    }

    UART_Print("Complete Frame : ");

    for(int i = 0; i < 7; i++)
        UART_Print("%02X ", rx[i]);

    UART_Print("\r\n");

    if(!LoadCellValidateCRC(rx, 7))
    {
        UART_Print("CRC FAILED\r\n");
        return false;
    }

    uint16_t value = ((uint16_t)rx[3] << 8) | rx[4];

    UART_Print("Register Value = %u (0x%04X)\r\n", value, value);

    UART_Print("========== TEST PASSED ==========\r\n");

    return true;
}
