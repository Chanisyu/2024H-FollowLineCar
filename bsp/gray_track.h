#ifndef __gray_track_h_
#define __gray_track_h_
#include "main.h"

// 推挽输出
#define GRAY_CLK_PORT GPIOA
#define GRAY_CLK_PIN  GPIO_PIN_3
// 输入模式
#define GRAY_DAT_PORT GPIOA
#define GRAY_DAT_PIN  GPIO_PIN_4

#define O1 ((gray>>0)&1)
#define O2 ((gray>>1)&1)
#define O3 ((gray>>2)&1)
#define O4 ((gray>>3)&1)
#define O5 ((gray>>4)&1)
#define O6 ((gray>>5)&1)
#define O7 ((gray>>6)&1)
#define O8 ((gray>>7)&1)

void track(void);
void track_reset(void);

extern float Kp;
extern float Kd;
extern float Kpp;
extern float Kdd;

#endif
