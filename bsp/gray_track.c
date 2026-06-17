/**
 * @file    gray_track.c
 * @brief   8 路数字灰度传感器读取、循迹误差计算和左右轮差速控制。
 *
 * 文件结构：
 *   1. gray_delay_short()  - 灰度模块串行时序短延时
 *   2. gray_board_read()   - 读取 8 路灰度数字量
 *   3. track_error()       - 根据压线探头计算循迹误差，并处理丢线保护
 *   4. track_reset()       - 清空循迹控制跨帧状态
 *   5. track()             - 根据误差计算差速目标速度
 */

#include "main.h"
#include "gray_track.h"
#include "pid.h"
#include "mpu6050.h"
#include "math.h"
#include "Motor.h"

float last_err=0;
uint16_t lose_cnt=0;

float now_out=0;
float last_out=0;
float fina_out=0;

float Kp=5.0f;   // 比例系数：越大回正越快，过大容易左右震荡。
float Kd=1.0f;   // 微分系数：根据误差变化提供阻尼，过大容易响应迟钝。
float Kpp=0.0f;  // 非线性比例项：放大大偏差时的转向量。
float Kdd=0.0f;  // 陀螺仪角速度阻尼项，用于抑制车身快速摆动。

#define TRACK_BASE_SPEED 18.0f
#define TRACK_SPEED_MAX  36.0f

static void gray_delay_short(void)
{
  for(volatile int i=0;i<100;i++)
  {
    __NOP();  // 给灰度模块 CLK/DAT 串行时序留稳定时间。
  }
}

uint8_t gray_board_read(void)
{
  uint8_t value=0;

  HAL_GPIO_WritePin(GRAY_CLK_PORT,GRAY_CLK_PIN,GPIO_PIN_RESET);

  for(uint8_t i=0;i<8;i++)
  {
    HAL_GPIO_WritePin(GRAY_CLK_PORT,GRAY_CLK_PIN,GPIO_PIN_SET);
    gray_delay_short();

    HAL_GPIO_WritePin(GRAY_CLK_PORT,GRAY_CLK_PIN,GPIO_PIN_RESET);

    if(HAL_GPIO_ReadPin(GRAY_DAT_PORT,GRAY_DAT_PIN)==GPIO_PIN_SET)
    {
      value|=(1u<<i);  // bit0~bit7 对应 O1~O8，便于后续用宏判断每一路状态。
    }
  }

  return value;
}

float track_error(void)
{
  uint8_t gray=gray_board_read();
  float sum=0;
  int cnt=0;

  // 左侧探头压线给负误差，右侧探头压线给正误差，数值越外侧权重越大。
  if(O1==0)  {sum-=3.5;cnt++;}
  if(O2==0)  {sum-=2.5;cnt++;}
  if(O3==0)  {sum-=1.5;cnt++;}
  if(O4==0)  {sum-=0.5;cnt++;}
  if(O5==0)  {sum+=0.5;cnt++;}
  if(O6==0)  {sum+=1.5;cnt++;}
  if(O7==0)  {sum+=2.5;cnt++;}
  if(O8==0)  {sum+=3.5;cnt++;}

  if(cnt==0)
  {
    /*
     * track_error() 由 track() 调用，track() 在 pid_control() 中执行，
     * pid_control() 由 10ms 定时器节拍驱动，所以 lose_cnt 每加 1 约等于丢线 10ms。
     */
    lose_cnt++;
    if(lose_cnt>=100)
    {
      // 连续约 1s 没有检测到黑线时停车，避免小车继续盲走。
      pid_set_tar_speed(0,0);
//      // 把速度环的目标角度设置为初始状态的反方向，希望以此让小车出弯后直行
//      angle.target = ANGStra - 180;
//      // 重置左右轮累计距离
//      total_left = 0;
//      total_right = 0;
//      State = 1;
      return 0;
    }
    return last_err>0?4:-4;  // 短暂丢线时沿上一轮偏差方向继续找线。
  }
  else
  {
    lose_cnt=0;
  }

  if(cnt!=0)
  {
    return sum/(float)cnt;
  }

  return last_err;
}

void track_reset(void)
{
  last_err=0.0f;
  lose_cnt=0;
  now_out=0.0f;
  last_out=0.0f;
  fina_out=0.0f;
}

void track(void)
{
  float err=track_error();
  float derr=err-last_err;
  last_err=err;

  // 灰度 P/D 修正 + 大偏差非线性修正 + 陀螺仪角速度阻尼修正。
  now_out=Kp*err+Kd*derr+Kpp*(err*fabsf(err))+Kdd*(float)((float)gz-gyro_zero_z)/16.4f;
  fina_out=now_out*0.3f+last_out*0.7f;  // 低通滤波，降低循迹输出抖动。
  last_out=fina_out;

  if(fina_out>TRACK_BASE_SPEED)  fina_out=TRACK_BASE_SPEED;
  if(fina_out<-TRACK_BASE_SPEED)  fina_out=-TRACK_BASE_SPEED;

  int right=(int)(TRACK_BASE_SPEED-fina_out);
  int left=(int)(TRACK_BASE_SPEED+fina_out);

  if(right<0)  right=0;
  if(left<0)  left=0;
  if(right>(int)TRACK_SPEED_MAX)  right=(int)TRACK_SPEED_MAX;
  if(left>(int)TRACK_SPEED_MAX)  left=(int)TRACK_SPEED_MAX;

  pid_set_tar_speed(right,left);
}