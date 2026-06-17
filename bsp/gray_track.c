#include "main.h"
#include "gray_track.h"
#include "pid.h"
#include "mpu6050.h"
#include "math.h"
#include "Motor.h"

float last_err = 0;
uint16_t lose_cnt = 0;

float now_out = 0;
float last_out = 0;
float fina_out = 0;

float Kp = 8;
float Kd = 2.45;
float Kpp = 0.0f;
float Kdd = 0.0f;

#define TRACK_BASE_SPEED 36.0f
#define TRACK_SPEED_MAX  72.0f

static void gray_delay_short(void)
{
	for (volatile int i = 0; i < 100; i++) {
		__NOP();
	}
}

uint8_t gray_board_read(void)
{
	uint8_t value = 0;

	HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_RESET);

	for (uint8_t i = 0; i < 8; i++) 
	{
		HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_SET);
		gray_delay_short();

		if (HAL_GPIO_ReadPin(GRAY_DAT_PORT, GRAY_DAT_PIN) == GPIO_PIN_SET)
		{
			value |= (1u << i);
		}

		HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_RESET);
		gray_delay_short();
	}

	return value;
}

// 通过灰度传感器计算error
// TODO:这个地方要不要把循迹环也封装成一个结构体那样
float track_error(void)
{		
	uint8_t gray = gray_board_read();
	
	float sum = 0;
	int cnt = 0;

	if(O1 == 0) { sum -= 3.5; cnt++; }
	if(O2 == 0) { sum -= 2.5; cnt++; }
	if(O3 == 0) { sum -= 1.5; cnt++; }
	if(O4 == 0) { sum -= 0.5; cnt++; }
	if(O5 == 0) { sum += 0.5; cnt++; }
	if(O6 == 0) { sum += 1.5; cnt++; }
	if(O7 == 0) { sum += 2.5; cnt++; }
	if(O8 == 0) { sum += 3.5; cnt++; }

	// 丢线处理
	if(cnt == 0)
	{
		// track_error被track调用，track在pid_control里被调用，而pid_control在【20ms】的定时器中断里被调用，
		// 所以每一次lose_cnt代表的是20ms。
		lose_cnt++;
		if(lose_cnt >= 50)
		{
			// 先停车
			pid_set_tar_speed(0, 0);
//			// 把速度环的目标角度设置为初始状态的反方向，希望以此让小车出弯后直行
//			angle.target = ANGStra - 180;
//			// 重置左右轮累计距离
//			total_left = 0;
//			total_right = 0;
//			State = 1;
			return 0;
		}
		return last_err > 0 ? 4 : -4;
	}
	else
	{
		lose_cnt = 0;
	}
	
	if(cnt != 0)
	{
		return sum / (float)cnt;
	}

	return last_err;
}

void track_reset(void)
{
	last_err = 0.0f;
	lose_cnt = 0;
	now_out = 0.0f;
	last_out = 0.0f;
	fina_out = 0.0f;
}

void track(void)
{
	float err = track_error();
	float derr = err - last_err;
	last_err = err;

	// 灰度偏差P修正 + 灰度偏差D修正 + 非线性大偏差修正 + 陀螺仪角速度修正
	now_out = Kp * err + Kd * derr + Kpp * (err * fabsf(err)) + Kdd * (float)((float)gz - gyro_zero_z) / 16.4f;
	fina_out = now_out*0.3f + last_out*0.7f;
	last_out = fina_out;

	// TODO:这个地方的base可以设定为全局变量方便修改
	if (fina_out > TRACK_BASE_SPEED) fina_out = TRACK_BASE_SPEED;
	if (fina_out < -TRACK_BASE_SPEED) fina_out = -TRACK_BASE_SPEED;

	int right = (int)(TRACK_BASE_SPEED - fina_out);
	int left  = (int)(TRACK_BASE_SPEED + fina_out);

	if (right < 0) right = 0;
	if (left < 0) left = 0;
	if (right > (int)TRACK_SPEED_MAX) right = (int)TRACK_SPEED_MAX;
	if (left > (int)TRACK_SPEED_MAX) left = (int)TRACK_SPEED_MAX;

	pid_set_tar_speed(right, left);
}

