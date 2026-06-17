/**
 * @file    pid.h
 * @brief   小车速度环和角度环 PID 数据结构及接口。
 */

#ifndef __PID_h_
#define __PID_h_
#include "main.h"

enum
{
  POSITION_PID=0,  // 位置式
  DELTA_PID=1,     // 增量式
};

typedef struct pid_s
{
	float target;      // 目标速度或目标角度，由上层控制逻辑在每个阶段更新。
	float now;         // 当前实测速度或当前角度，作为 PID 反馈量。
	float error[3];    // 最近三次偏差；error[0] 为本轮偏差，用于 P、I、D 项计算。
	float p,i,d;       // PID 三个可调系数；增大 P 响应更快，I 消除稳态误差，D 抑制震荡。
	float pout,dout,iout;
	float out;         // PID 最终输出，速度环对应电机 PWM 控制量。
	uint8_t pid_mode;  // PID 计算模式：POSITION_PID 为位置式，DELTA_PID 为增量式。

} pid_t;

void pid_cal_motor(volatile pid_t *pid);
void pid_cal_angle(pid_t *pid);
void pid_Init(volatile pid_t *pid,uint8_t Mode,float p,float i,float d);
void pid_control();
void pid_set_tar_speed(float spdAR,float spdBL);
void pidout_limit(volatile pid_t *pid);
void pid_set_base_speed(int16_t Speed);
void pid_reset_motor(volatile pid_t *pid);
void pid_reset_speed_loop(void);


extern volatile pid_t MotorAR;
extern volatile pid_t MotorBL;
extern pid_t angle;
extern volatile int16_t base_speed;


#endif
