/**
 * @file    pid.c
 * @brief   小车速度环、角度环 PID 计算和电机输出限幅。
 *
 * 文件结构：
 *   1. pid_Init()          - 初始化 PID 结构体参数和历史误差
 *   2. pid_control()       - 按控制标志执行角度环、速度环和电机输出
 *   3. pid_cal_motor()     - 计算左右轮速度环 PID 输出
 *   4. pidout_limit()      - 限制 PWM 控制量，保护电机驱动
 *   5. pid_cal_angle()     - 计算航向角闭环输出并处理 0/360 度跳变
 */

#include "pid.h"
#include "Motor.h"
#include "tim.h"
#include "usart.h"
#include "gray_track.h"
#include "mpu6050.h"
#include "HMC5883L.h"

// ================================================================
// 1. PID 控制对象
//
//    MotorAR / MotorBL 为左右轮速度环，跨控制周期保存目标速度、
//    实际速度、历史误差和累计输出。
//    angle 为航向角环，使用磁力计偏航角和陀螺仪 Z 轴角速度辅助修正。
// ================================================================

volatile pid_t MotorAR;
volatile pid_t MotorBL;
pid_t angle;

volatile int16_t base_speed;

void pid_Init(volatile pid_t *pid,uint8_t Mode,float p,float i,float d)
{
  pid->pid_mode=Mode;
  pid->p=p;
  pid->i=i;
  pid->d=d;
  pid->target=0.0f;
  pid->now=0.0f;
  pid->error[0]=0.0f;
  pid->error[1]=0.0f;
  pid->error[2]=0.0f;
  pid->pout=0.0f;
  pid->iout=0.0f;
  pid->dout=0.0f;
  pid->out=0.0f;
}

void pid_set_tar_speed(float spdAR,float spdBL)
{
  MotorAR.target=spdAR;
  MotorBL.target=spdBL;
}

void pid_set_base_speed(int16_t Speed)
{
  base_speed=Speed;
}

void pid_reset_motor(volatile pid_t *pid)
{
  pid->error[0]=0.0f;
  pid->error[1]=0.0f;
  pid->error[2]=0.0f;
  pid->pout=0.0f;
  pid->iout=0.0f;
  pid->dout=0.0f;
  pid->out=0.0f;
}

void pid_reset_speed_loop(void)
{
  pid_reset_motor(&MotorAR);
  pid_reset_motor(&MotorBL);
}

void pid_control()
{
  if(PIDConFlag==0)  return;
  PIDConFlag=0;

  // ------- 1. 角度环修正 -------
  if(ANGLOOP==1)
  {
    angle.now=yaw_hmc;
    pid_cal_angle(&angle);

    // 角度环输出作为左右轮差速量，限幅避免航向误差过大时直接打满电机。
    if(angle.out>200)
    {
      angle.out=200;
    }
    if(angle.out<-200)
    {
      angle.out=-200;
    }

    // angle.out>0 时右轮目标更高、左轮更低，小车向左侧修正。
    pid_set_tar_speed(base_speed-angle.out,base_speed+angle.out);
  }

  // ------- 2. 循迹目标速度 -------
  if(State==2)
  {
    track();  // 循迹状态下由灰度模块实时刷新左右轮目标速度。
  }

  // ------- 3. 速度环 PID -------
  // MotorAR.now / MotorBL.now 在 TIM4 中断中由编码器计数更新。
  pid_cal_motor(&MotorAR);
  pid_cal_motor(&MotorBL);
  pidout_limit(&MotorAR);
  pidout_limit(&MotorBL);

  // ------- 4. 电机方向和 PWM 输出 -------
  if(MotorAR.out>=0)
  {
    MotorAR_set(MotorAR.out,1);
  }
  else
  {
    MotorAR_set(-MotorAR.out,0);
  }
  if(MotorBL.out>=0)
  {
    MotorBL_set(MotorBL.out,1);
  }
  else
  {
    MotorBL_set(-MotorBL.out,0);
  }
}

// ================================================================
// 2. 速度环 PID 计算
//
//    增量式 PID 直接累加到 out，适合速度闭环连续调节。
//    位置式 PID 直接由当前误差、积分误差和误差变化量组成输出。
// ================================================================
void pid_cal_motor(volatile pid_t *pid)
{
  pid->error[0]=pid->target-pid->now;
  if(pid->pid_mode==DELTA_PID)
  {
    pid->pout=pid->p*(pid->error[0]-pid->error[1]);
    pid->iout=pid->i*pid->error[0];
    pid->dout=pid->d*(pid->error[0]-2*pid->error[1]+pid->error[2]);
    pid->out+=pid->pout+pid->iout+pid->dout;
  }
  else if(pid->pid_mode==POSITION_PID)
  {
    pid->pout=pid->p*pid->error[0];
    pid->iout+=pid->i*pid->error[0];
    pid->dout=pid->d*(pid->error[0]-pid->error[1]);
    pid->out=pid->pout+pid->iout+pid->dout;
  }

  // error[1]/error[2] 跨周期保存，供下一轮 D 项和增量式差分使用。
  pid->error[2]=pid->error[1];
  pid->error[1]=pid->error[0];
}

// 输出限幅保护：PWM 控制量超过驱动可用范围时钳住，避免积分把输出推到不可控区间。
void pidout_limit(volatile pid_t *pid)
{
  if(pid->out>=20000)  pid->out=20000;
  if(pid->out<=-20000)  pid->out=-20000;
}

// ================================================================
// 3. 角度环 PID 计算
//
//    航向角以度为单位，磁力计输出在 -180~180 或 0~360 附近会跳变。
//    先把误差折算到 -180~180，保证小车选择最短方向修正。
//    KddForANG 使用陀螺仪角速度增加阻尼，KppForANG 放大大角度误差。
// ================================================================
void pid_cal_angle(pid_t *pid)
{
  float err=pid->target-pid->now;
  while(err>180)  err-=360;
  while(err<-180)  err+=360;
  pid->error[0]=err;

  if(pid->pid_mode==DELTA_PID)
  {
    pid->pout=pid->p*(pid->error[0]-pid->error[1]);
    pid->iout=pid->i*pid->error[0];
    pid->dout=pid->d*(pid->error[0]-2*pid->error[1]+pid->error[2]);
    pid->out+=pid->pout+pid->iout+pid->dout;
  }
  else if(pid->pid_mode==POSITION_PID)
  {
    pid->pout=pid->p*pid->error[0];
    pid->iout+=pid->i*pid->error[0];
    pid->dout=pid->d*(pid->error[0]-pid->error[1]);
    pid->out=pid->pout+pid->iout+pid->dout+KddForANG*((float)((float)gz-gyro_zero_z)/16.4f)+KppForANG*pid->error[0]*fabs(pid->error[0]);
  }

  pid->error[2]=pid->error[1];
  pid->error[1]=pid->error[0];
}
