#include "step42_protocol.h"

/*
 * Replace this function with the UART send function of your platform.
 *
 * STM32 HAL:
 *   HAL_UART_Transmit(&huart1, data, len, 100);
 *
 * TI DL:
 *   for each byte: DL_UART_Main_transmitDataBlocking(UART_0_INST, data[i]);
 *
 * Linux:
 *   write(fd, data, len);
 */
static void user_uart_send(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
}

void step42_demo(void)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint8_t seq = 0;
    uint16_t len;

    len = Step42_BuildSetStream(tx, sizeof(tx), ++seq, 0);
    user_uart_send(tx, len);

    len = Step42_BuildEnable(tx, sizeof(tx), ++seq, 1);
    user_uart_send(tx, len);

    len = Step42_BuildSetAngle(tx, sizeof(tx), ++seq, 90.0f);
    user_uart_send(tx, len);

    len = Step42_BuildSetSpeed(tx, sizeof(tx), ++seq, 1000, 3000);
    user_uart_send(tx, len);

    len = Step42_BuildSetSpeed(tx, sizeof(tx), ++seq, 0, 3000);
    user_uart_send(tx, len);
}

