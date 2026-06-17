/**
 * @file    gray_track.h
 * @brief   数字灰度传感器循迹接口和 CLK/DAT 引脚定义。
 */

#ifndef __gray_track_h_
#define __gray_track_h_
#include "main.h"

#define GRAY_CLK_PORT GPIOA  // 推挽输出：串行读取时钟线。
#define GRAY_CLK_PIN  GPIO_PIN_3
#define GRAY_DAT_PORT GPIOA  // 输入模式：串行读取数据线。
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