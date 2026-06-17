/**
 * @file    Motor.h
 * @brief   左右轮电机方向和 PWM 控制接口。
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

void MotorAR_set(uint16_t Duty,uint8_t Dir);  // Dir=0 后退，Dir=1 前进。
void MotorBL_set(uint16_t Duty,uint8_t Dir);  // Dir=0 后退，Dir=1 前进。

void MotorAR_start(void);
void MotorAR_stop(void);
void MotorBL_start(void);
void MotorBL_stop(void);

extern volatile int16_t Count;

extern volatile uint32_t total_left;   // 左轮编码器累计值。
extern volatile uint32_t total_right;  // 右轮编码器累计值。

#endif
