/**
 * @file    vofa.h
 * @brief   VOFA 串口调试接口、接收缓冲区和命令解析入口。
 */

#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include <string.h>
#include <stdlib.h>

// 串口单字节接收缓冲区，由 HAL_UART_Receive_IT() 写入。
extern uint8_t vofa_rx_ch;

// 一行 VOFA 命令的接收缓冲区，例如 "T=50"、"KP=10"。
extern char vofa_rx_line[64];

// 当前已接收到 vofa_rx_line 的第几个字符。
extern uint8_t vofa_rx_idx;

/*
 * 初始化 VOFA 串口通信。
 * huart：用于和 VOFA 通信的 UART 句柄，例如 &huart1。
 */
void VOFA_Init(UART_HandleTypeDef *huart);

/*
 * 向 VOFA 发送速度环波形数据。
 * target_speed：目标速度，用作第一路波形。
 * real_speed：实际速度，用作第二路波形。
 */
void VOFA_SendSpeedLoop(float target_speed,float real_speed);

/*
 * 发送 8 路灰度数组。
 * 每次发送 3 行：
 *   A,adc0,adc1,...,adc7
 *   N,nor0,nor1,...,nor7
 *   D,dark0,dark1,...,dark7
 */
void VOFA_SendGrayArrays(void);

/*
 * HAL 串口接收完成回调函数。
 * huart：触发本次接收中断的 UART 句柄。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/*
 * 解析 VOFA 发来的单行命令。
 * line：以 '\0' 结尾的命令字符串，例如 "T=50"、"KP=8.5"。
 */
void VOFA_ParseLine(char *line);

#endif
