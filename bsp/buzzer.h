/**
 * @file    buzzer.h
 * @brief   低电平有效蜂鸣器接口。
 */

#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "main.h"

#define BUZZER_PORT GPIOA
#define BUZZER_PIN  GPIO_PIN_10  // GPIOA10，低电平有效。

void Buzzer_Start(uint32_t ms);
void Buzzer_Task(void);

#endif
