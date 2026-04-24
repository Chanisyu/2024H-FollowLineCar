#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

void MotorAR_set(uint16_t Duty , uint8_t Dir);	// 0为后退，1为前进
void MotorBL_set(uint16_t Duty , uint8_t Dir);	// 0为后退，1为前进
void MotorAR_start(void);
void MotorAR_stop(void);
void MotorBL_start(void);
void MotorBL_stop(void);

extern volatile int16_t Count;
extern volatile uint32_t total_left;
extern volatile uint32_t total_right;

#endif
