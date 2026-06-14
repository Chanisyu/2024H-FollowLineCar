#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

void MotorAR_set(uint16_t Duty , uint8_t Dir);	// 0为后退，1为前进
void MotorBL_set(uint16_t Duty , uint8_t Dir);	// 0为后退，1为前进

/**** 启动和停止PWM波输出 ****/
void MotorAR_start(void);
void MotorAR_stop(void);
void MotorBL_start(void);
void MotorBL_stop(void);

extern volatile int16_t Count;

/**** 左右轮编码器累积值 ****/
extern volatile uint32_t total_left;
extern volatile uint32_t total_right;

#endif
