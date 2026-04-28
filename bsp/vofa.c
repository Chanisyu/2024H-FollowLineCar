#include "vofa.h"

static UART_HandleTypeDef *vofa_uart = NULL;

void VOFA_Init(UART_HandleTypeDef *huart)
{
    vofa_uart = huart;
}

void VOFA_SendSpeedLoop(float target_speed,
                        float real_speed)
{
    if (vofa_uart == NULL)
    {
        return;
    }

    char buf[100];

    int len = snprintf(buf,
                       sizeof(buf),
                       "%f,%f\n",
                       target_speed,
                       real_speed);

    if (len > 0 && len < sizeof(buf))
    {
        HAL_UART_Transmit(vofa_uart, (uint8_t *)buf, len, 20);
    }
}
