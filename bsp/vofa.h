#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include <string.h>
#include <stdlib.h>

extern uint8_t vofa_rx_ch;
extern char vofa_rx_line[64];
extern uint8_t vofa_rx_idx;

void VOFA_Init(UART_HandleTypeDef *huart);
void VOFA_SendSpeedLoop(float target_speed,
                        float real_speed);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void VOFA_ParseLine(char *line);


#endif
