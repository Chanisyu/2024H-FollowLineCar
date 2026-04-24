#ifndef __KEY_H__
#define __KEY_H__

#include "main.h"

struct key
{
	uint8_t judge_sta;
	uint8_t key_sta;
	uint32_t key_time;
	uint8_t key_short;
	uint8_t key_long;
};

void key_task();

extern volatile struct key KEYS[];

#endif
