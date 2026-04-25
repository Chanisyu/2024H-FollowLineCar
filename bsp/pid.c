#include "pid.h"
#include "Motor.h"
#include "tim.h"
#include "usart.h"
#include "gray_track.h"
#include "mpu6050.h"
#include "HMC5883L.h"

// 创建结构体实例
// 速度环结构体
pid_t MotorAR;
pid_t MotorBL;
// 角度环结构体
pid_t angle;

volatile int16_t base_speed;

void datavision_send()  // 上位机波形发送函数
{
    // 数据包头
	HAL_UART_Transmit(&huart1,(uint8_t[]){0x03},1,100);
	HAL_UART_Transmit(&huart1,(uint8_t[]){0xfc},1,100);
	
    // 发送数据
	HAL_UART_Transmit(&huart1,(uint8_t[]){(uint8_t)MotorAR.target},1,100);
	HAL_UART_Transmit(&huart1,(uint8_t[]){(uint8_t)MotorAR.now},1,100);
	
    // 数据包尾
	HAL_UART_Transmit(&huart1,(uint8_t[]){0xfc},1,100);
	HAL_UART_Transmit(&huart1,(uint8_t[]){0x03},1,100);
}

void pid_Init(pid_t *pid ,uint8_t Mode ,float p ,float i ,float d)
{
	pid->pid_mode = Mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
}

void pid_set_tar_speed(int16_t spdAR ,int16_t spdBL)
{
	MotorAR.target = spdAR;
	MotorBL.target = spdBL;
}

void pid_set_base_speed(int16_t Speed)
{
	base_speed = Speed;
}

void pid_control()
{
	// 角度环
	if(ANGLOOP == 1)
	{
		// 1.设定目标角度
		// 2.获取当前角度
		angle.now = yaw_hmc;
		// 3.输入pid控制器计算
		pid_cal_angle(&angle);
		// 输出限幅
		if(angle.out > 200)
		{
			angle.out = 200;
		}
		if(angle.out < -200)
		{
			angle.out = -200;
		}
		// 4.应用输出值
		pid_set_tar_speed(base_speed - angle.out, base_speed + angle.out);
	}
	
	// 速度环
	// 1.设定目标速度
	if(State == 2)
	{
		track();
	}
	// 2.获取当前速度
	MotorAR.now = (int16_t)(__HAL_TIM_GET_COUNTER(&htim2));
	__HAL_TIM_SET_COUNTER(&htim2,0);
	total_right += (uint32_t)MotorAR.now;
	
	MotorBL.now = -(int16_t)(__HAL_TIM_GET_COUNTER(&htim3));
	__HAL_TIM_SET_COUNTER(&htim3,0);
	total_left += (uint32_t)MotorBL.now;
	
	// 3.输入pid控制器计算
	pid_cal_motor(&MotorAR);
	pid_cal_motor(&MotorBL);
	pidout_limit(&MotorAR);
	pidout_limit(&MotorBL);

	// 4.应用输出值
	if(MotorAR.out >= 0)	{MotorAR_set(MotorAR.out,1);}
	else	{MotorAR_set(-MotorAR.out,0);}
	if(MotorBL.out >= 0)	{MotorBL_set(MotorBL.out,1);}
	else	{MotorBL_set(-MotorBL.out,0);}
	
	// 5.给上位机发送数据
	datavision_send();
}

void pid_cal_motor(pid_t *pid)
{
	// 计算当前偏差
	pid->error[0] = pid->target - pid->now;
	// 计算输出
	if(pid->pid_mode == DELTA_PID)  // 增量式
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)  // 位置式
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// 记录前两次偏差
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	
}

void pidout_limit(pid_t *pid)
{
		// 输出限幅
	if(pid->out>=20000)	
		pid->out=20000;
	if(pid->out<=-20000)	
		pid->out=-20000;
}

// 角度专用，处理了角度跳变问题
void pid_cal_angle(pid_t *pid)
{
	// 计算当前偏差
	// 处理角度跳变
	float err = pid->target - pid->now;
	while(err > 180) err -= 360;
	while(err < -180) err += 360;
	pid->error[0] = err;

	// 计算输出
	if(pid->pid_mode == DELTA_PID)  // 增量式
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	// 增加了Kpp
	else if(pid->pid_mode == POSITION_PID)  // 位置式
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout + KddForANG * ( (float)((float)gz - gyro_zero_z)/16.4f ) + KppForANG * pid->error[0]*fabs( pid->error[0] );
	}

	// 记录前两次偏差
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
}

