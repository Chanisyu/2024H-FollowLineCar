#include "Motor.h"
#include "tim.h"

volatile int16_t Count = 0;
volatile uint32_t total_left = 0;
volatile uint32_t total_right = 0;

void MotorAR_set(uint16_t Duty , uint8_t Dir)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,!Dir);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,Dir);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty);
}

void MotorBL_set(uint16_t Duty , uint8_t Dir)
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
