/**
 * @file    key.h
 * @brief   4 路按键消抖和事件标志接口。
 */

#ifndef __KEY_H__
#define __KEY_H__

#include "main.h"

struct key
{
	uint8_t judge_sta;  // 按键判定状态机状态。
	uint8_t key_sta;    // 当前采样电平。
	uint32_t key_time;  // 按下持续时间，单位 ms。
	uint8_t key_short;  // 短按事件标志。
	uint8_t key_long;   // 长按事件标志。
};

void key_task();

extern volatile struct key KEYS[];

#endif
