/**
 * @file    gray_track.h
 * @brief   感为无 MCU 八路灰度传感器读取接口和循迹控制接口。
 */

#ifndef __gray_track_h_
#define __gray_track_h_

#include "main.h"

/*
 * 无 MCU 灰度传感器地址线定义。
 * AD0/AD1/AD2 共同决定当前 OUT 输出的是 8 路探头中的哪一路。
 */
#define GRAY_AD0_PORT GPIOA
#define GRAY_AD0_PIN  GPIO_PIN_3
#define GRAY_AD1_PORT GPIOB
#define GRAY_AD1_PIN  GPIO_PIN_0
#define GRAY_AD2_PORT GPIOB
#define GRAY_AD2_PIN  GPIO_PIN_1

/*
 * 官方例程 Direction=1 时，会把地址 0 的读取值放到数组最右侧。
 * 这里默认保持官方方向，使 gray_analog[0]~gray_analog[7] 对应 O1~O8。
 * 如果你实测发现左右方向反了，把这个宏改成 0 即可，不需要改读取逻辑。
 */
#define GRAY_REVERSE_ORDER 1

/*
 * 兼容旧版数字灰度写法。
 * 使用这些宏时，函数内部必须存在名为 gray 的局部变量。
 * bit=1 表示白底，bit=0 表示黑线。
 */
#define O1 ((gray>>0)&1)
#define O2 ((gray>>1)&1)
#define O3 ((gray>>2)&1)
#define O4 ((gray>>3)&1)
#define O5 ((gray>>4)&1)
#define O6 ((gray>>5)&1)
#define O7 ((gray>>6)&1)
#define O8 ((gray>>7)&1)

/* 更新 8 路模拟灰度、归一化灰度、黑线强度和二值灰度。 */
void gray_sensor_update(void);
/* 读取兼容旧逻辑的 8 位数字灰度，1=白底，0=黑线。 */
uint8_t gray_board_read(void);
/* 根据灰度误差设置左右轮目标速度。 */
void track(void);
/* 重置循迹状态，清空上一次误差、丢线计数和输出滤波状态。 */
void track_reset(void);

/* 8 路原始 ADC 值，白底通常较大，黑线通常较小。 */
extern uint16_t gray_analog[8];
/* 归一化后的白色强度，白底约 4096，黑线约 0。 */
extern uint16_t gray_normal[8];
/* 黑线强度，黑线约 4096，白底约 0，track_error() 主要使用这个数组。 */
extern uint16_t gray_dark[8];
/* 兼容旧辅助板的 8 位二值灰度结果，bit=1 白，bit=0 黑。 */
extern uint8_t gray_raw;

/* 每一路白底和黑线的 ADC 标定值，后续应使用实测值替换默认值。 */
extern uint16_t gray_white_cal[8];
extern uint16_t gray_black_cal[8];

extern float Kp;
extern float Kd;
extern float Kpp;
extern float Kdd;

#endif
