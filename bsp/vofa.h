/**
 * @file    vofa.h
 * @brief   VOFA 串口调试接口、接收缓冲区和命令解析入口。
 */

#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include <string.h>
#include <stdlib.h>

typedef struct pid_s pid_t;

/* 待恢复注释 HAL_UART_Receive_IT() 待恢复注释 */
extern uint8_t vofa_rx_ch;

/* 待恢复注释 "STATUS"待恢复注释"T=20"待恢复注释"SET P:10 I:5 D:0.5"待恢复注释 */
extern char vofa_rx_line[64];

/* 待恢复注释 vofa_rx_line 待恢复注释 */
extern uint8_t vofa_rx_idx;

/*
 * 待恢复注释
 *   0 - 待恢复注释
 *   1 - 待恢复注释
 */
extern uint8_t vofa_stream_mode;

/*
 * 待恢复注释
 *   0 - State==0 待恢复注释
 *   1 - State==0 待恢复注释
 */
extern uint8_t vofa_speed_hold;

/* 待恢复注释 VOFA 待恢复注释 1 待恢复注释 */
void VOFA_Init(UART_HandleTypeDef *huart);

/* 待恢复注释 */
void VOFA_SetSpeedTuneDefaults(void);

/*
 * 待恢复注释 8d24f0e... 待恢复注释 PID 待恢复注释
 * 待恢复注释
 *   tick,target,now,out,error,p,i,d\n
 */
void VOFA_SendSpeedLoop(volatile pid_t *pid);

/*
 * 待恢复注释 8 待恢复注释 3 待恢复注释
 *   A,adc0,adc1,...,adc7
 *   N,nor0,nor1,...,nor7
 *   D,dark0,dark1,...,dark7
 */
void VOFA_SendGrayArrays(void);

/* USART1 待恢复注释 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* 待恢复注释PID 待恢复注释 */
void VOFA_ParseLine(char *line);

#endif
