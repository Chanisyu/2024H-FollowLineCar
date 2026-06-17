/**
 * @file    vofa.h
 * @brief   VOFA 串口调试接口、接收缓冲区和命令解析入口。
 */

#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include <string.h>
#include <stdlib.h>

extern uint8_t vofa_rx_ch;
extern char vofa_rx_line[64];
extern uint8_t vofa_rx_idx;
extern uint8_t vofa_stream_mode;
extern uint8_t vofa_speed_hold;

void VOFA_Init(UART_HandleTypeDef *huart);
void VOFA_SetSpeedTuneDefaults(void);
void VOFA_SendSpeedLoop(float target_speed,float real_speed);
void VOFA_SendGrayArrays(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void VOFA_ParseLine(char *line);

#endif
