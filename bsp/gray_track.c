#include "main.h"
#include "gray_track.h"
#include "pid.h"
#include "mpu6050.h"
#include "math.h"
#include "Motor.h"

float last_err = 0;
uint8_t lose_cnt = 0;
float Kp = 14;
float Kd = 2;
float Kpp = 1.5;
float Kdd = 0.5;

// 通过灰度传感器计算error
float track_error(void)
{		
	float sum = 0;
	int cnt = 0;

	if (O1 == 0) { sum -= 1.5; cnt++; }
	if (O2 == 0) { sum -= 0.3; cnt++; }
	if (O3 == 0) { cnt++; }
	if (O4 == 0) { sum += 0.3; cnt++; }
	if (O5 == 0) { sum += 1.5; cnt++; }

	if (cnt == 0)
	{
		lose_cnt++;
//		if(lose_cnt >= 260)
//		{
//			total_left = 0;
//			total_right = 0;
//			State = 1;
//			lose_cnt = 0;
//		}
		if(lose_cnt >= 50)
		{
			pid_set_tar_speed(0, 0);
			angle.target = ANGStra - 180;
			total_left = 0;
			total_right = 0;
			State = 1;
			return 0;
		}
		return last_err > 0 ? 3 : -3;
	}
	else
	{
		lose_cnt = 0;
	}
	
	if(cnt != 0)
	{
		return sum / (float)cnt;
	}
}

void track(void)
{
	double err = track_error();
	double derr = err - last_err;
	last_err = err;

	float out = Kp * err + Kd * derr + Kpp * (err*fabs(err)) + Kdd * (float)((float)gz - gyro_zero_z)/16.4f;

	int base = 40;
	int right = base - (int)out;
	int left  = base + (int)out;

	if (right < 0) right = 0;
	if (left < 0) left = 0;
	if (right > 60) right = 60;
	if (left > 60) left = 60;

	pid_set_tar_speed(right, left);
}

