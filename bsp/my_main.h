#ifndef __MY_MAIN__
#define __MY_MAIN__

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdio.h"
#include "math.h"

#include "Motor.h"
#include "OLED.h"
#include "pid.h"
#include "mpu6050.h"
#include "HMC5883L.h"
#include "filter.h"
#include "buzzer.h"
#include "key.h"
#include "gray_track.h"
#include "HMC5883L_Calibration.h"
#include "vofa.h"

extern char Text[];
extern int8_t State;
extern uint8_t ANGLOOP;
extern int16_t ANGStra;
extern float KddForANG;
extern float KppForANG;

void setup();
void loop();
void key_proc();
void OLED_proc();
void Data_proc();
void VOFA_proc();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif
