#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

void VOFA_Init(UART_HandleTypeDef *huart);
void VOFA_SendSpeedLoop(float target_speed,
                        float real_speed);

#endif
