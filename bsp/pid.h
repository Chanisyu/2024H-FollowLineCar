#ifndef __PID_h_
#define __PID_h_
#include "main.h"

enum
{
  POSITION_PID = 0,  // 位置式
  DELTA_PID = 1,     // 增量式
};

typedef struct
{
	float target;	
	float now;
	float error[3];		
	float p,i,d;
	float pout, dout, iout;
	float out;   
	
	uint8_t pid_mode;

}pid_t;

void pid_cal_motor(pid_t *pid);
void pid_cal_angle(pid_t *pid);
void pid_Init(pid_t *pid ,uint8_t Mode ,float p ,float i ,float d);
void pid_control();
void pid_set_tar_speed(int16_t spdAR ,int16_t spdBL);
void pidout_limit(pid_t *pid);
void pid_set_base_speed(int16_t Speed);


extern pid_t MotorAR;
extern pid_t MotorBL;
extern pid_t angle;
extern volatile int16_t base_speed;


#endif
