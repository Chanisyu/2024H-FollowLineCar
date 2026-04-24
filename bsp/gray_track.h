#ifndef __gray_track_h_
#define __gray_track_h_
#include "main.h"

#define O1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)
#define O2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)
#define O3 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)
#define O4 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)
#define O5 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)

// 请把上述引脚配置为输入模式
void track(void);

extern float Kp;
extern float Kd;
extern float Kpp;
extern float Kdd;

#endif
