/**
 * @file    Motor.c
 * @brief   左右轮电机方向控制和 PWM 启停接口。
 *
 * 文件结构：
 *   1. MotorAR_set()    - 设置右轮方向和 PWM 占空比
 *   2. MotorBL_set()    - 设置左轮方向和 PWM 占空比
 *   3. MotorAR_start()  - 启动右轮 PWM 输出
 *   4. MotorBL_start()  - 启动左轮 PWM 输出
 *   5. MotorAR_stop()   - 停止右轮 PWM 输出
 *   6. MotorBL_stop()   - 停止左轮 PWM 输出
 */

#include "Motor.h"
#include "tim.h"

volatile int16_t Count=0;
volatile uint32_t total_left=0;   // 左轮编码器累计值，跨周期保存总行驶量。
volatile uint32_t total_right=0;  // 右轮编码器累计值，跨周期保存总行驶量。

// ================================================================
// 1. 右轮电机控制
//
//    PB12/PB13 控制右轮 H 桥方向，TIM1_CH1 输出 PWM。
//    Dir=1 表示前进，Dir=0 表示后退；Duty 越大电机驱动力越强。
// ================================================================
void MotorAR_set(uint16_t Duty,uint8_t Dir)
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,!Dir);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,Dir);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty);
}

// ================================================================
// 2. 左轮电机控制
//
//    PB14/PB15 控制左轮 H 桥方向，TIM1_CH2 输出 PWM。
//    Dir=1 表示前进，Dir=0 表示后退；左右轮方向逻辑保持一致。
// ================================================================
void MotorBL_set(uint16_t Duty,uint8_t Dir)
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,!Dir);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,Dir);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Duty);
}

void MotorAR_start(void)
{
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}

void MotorBL_start(void)
{
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
}

void MotorAR_stop(void)
{
  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
}

void MotorBL_stop(void)
{
  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
}
