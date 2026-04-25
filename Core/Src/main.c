// Hello World！
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"

#include "Motor.h"
#include "OLED.h"
#include "pid.h"
#include "mpu6050.h"
#include "HMC5883L.h"
#include "filter.h"
#include "buzzer.h"
#include "key.h"
#include "gray_track.h"
#include "HMC5883L_Calibration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char Text[30];
volatile int16_t right_speed;
volatile int16_t left_speed;
volatile uint8_t MPUDisp_Flag = 0;
volatile uint8_t Data_Flag = 0;
int8_t State = -1;
uint8_t View = 0;
uint8_t Selt_para = 0;

uint8_t ANGLOOP = 0;
// 开始时的直行角度
int16_t ANGStra;
HMC5883L_CalibrationResult HMC5883L_Cali_Res;
float KddForANG = 0;
float KppForANG = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// 定时器中断，每20ms触发一次，用于控制pid和按键扫描。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{	
		pid_control();
		key_task();
		MPUDisp_Flag = 1;
	}
}

// 外部中断，由MPU6050的INT引脚接到PB5，再开启外部中断，20ms触发一次，50Hz。
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_5)
	{
		Data_Flag = 1;
	}
}

// 按键判断逻辑，负责各个界面下按键的作用逻辑。
void key_proc()
{
	if(KEYS[0].key_short == 1)
	{
		State++;
		if(State >= 1)
		{
			State = -1;
		}
		OLED_Clear();
		KEYS[0].key_short = 0;
	}
	else if(KEYS[0].key_long == 1)
	{
		State = 1;
		OLED_Clear();
		KEYS[0].key_long = 0;
	}
	if(State == 0)
	{
		if(KEYS[1].key_short == 1)
		{
			Selt_para++;
			if(Selt_para >= 4)
			{
				Selt_para = 0;
			}
			KEYS[1].key_short = 0;
		}
		else if(KEYS[2].key_short == 1)
		{
			switch(Selt_para)
			{
				case 0:
				{
					Kp+=0.5;
					break;
				}
				case 1:
				{
					Kd+=0.5;
					break;
				}
				case 2:
				{
					Kpp+=0.5;
					break;
				}
				case 3:
				{
					Kdd+=0.5;
					break;
				}
			}
			KEYS[2].key_short = 0;
		}
		else if(KEYS[3].key_short == 1)
		{
			switch(Selt_para)
			{
				case 0:
				{
					Kp-=0.5;
					break;
				}
				case 1:
				{
					Kd-=0.5;
					break;
				}
				case 2:
				{
					Kpp-=0.5;
					break;
				}
				case 3:
				{
					Kdd-=0.5;
					break;
				}
			}
			KEYS[3].key_short = 0;
		}
	}
	if(State == -1)
	{
		// 这里复用了State0的参数选择状态变量
		if(KEYS[1].key_short == 1)
		{
			Selt_para++;
			if(Selt_para >= 3)
			{
				Selt_para = 0;
			}
			KEYS[1].key_short = 0;
		}
		else if(KEYS[2].key_short == 1)
		{
			switch(Selt_para)
			{
				case 0:
				{
					angle.p+=0.1;
					break;
				}
				case 1:
				{
					KppForANG+=0.01;
					break;
				}
				case 2:
				{
					angle.d+=0.5;
					break;
				}
			}
			KEYS[2].key_short = 0;
		}
		else if(KEYS[3].key_short == 1)
		{
			switch(Selt_para)
			{
				case 0:
				{
					angle.p-=0.1;
					break;
				}
				case 1:
				{
					KppForANG-=0.01;
					break;
				}
				case 2:
				{
					angle.d-=0.5;
					break;
				}
			}
			KEYS[3].key_short = 0;
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // 初始化OLED屏幕
  OLED_Init();
  // 启动右轮的TIM2编码器测速
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  // 启动左轮的TIM3编码器测速
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  // 蜂鸣器默认关闭
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);

  MotorAR_set(0,1);
  MotorAR_start();
  MotorBL_set(0,1);
  MotorBL_start(); 
	
	// PID参数的初始化
  pid_Init(&MotorAR,DELTA_PID,10,10,0);
  pid_Init(&MotorBL,DELTA_PID,10,10,0);
	pid_Init(&angle,POSITION_PID,0.9,0,18);
	
  pid_set_tar_speed(0,0);
  
	MPU6050_Init();
	HAL_Delay(50);
  HMC5883L_Init();
	
	// MPU6050gz的零漂校准
	calibrate_gyro();
	HMC5883L_Calibration_RunBlocking(&HMC5883L_Cali_Res, 600, 20, App_ReadSample, App_DelayMs, 0);
	
	// 启动TIM4，20ms触发一次。TIM4的中断回调函数我写在上面一点了，往上翻就能找到了。
  // 用于每20ms触发一次PID
  HAL_TIM_Base_Start_IT(&htim4);

	
/*----------第一题--------------------------------------------------------------------*/
		
		
/*------------------------------------------------------------------------------------*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(MPUDisp_Flag == 1)
		{
			if(State == -1)
			{
				snprintf(Text,30,"yaw_hmc=%.3f   ", yaw_hmc);
				OLED_ShowString(1, 1, Text);
				snprintf(Text,30,"P=%.1f   ", angle.p);
				OLED_ShowString(2, 1, Text);
				snprintf(Text,30,"Kpp=%.2f   ", KppForANG);
				OLED_ShowString(3, 1, Text);
				snprintf(Text,30,"D=%.1f   ", angle.d);
				OLED_ShowString(4, 1, Text);
			}
			else if(State == 0)
			{
				if(Selt_para == 0)
				{
					snprintf(Text,30,"Kp=%.2f ", Kp);
					OLED_ShowStringReverse(1, 1, Text);
					snprintf(Text,30,"Kd=%.2f ", Kd);
					OLED_ShowString(2, 1, Text);
					snprintf(Text,30,"Kpp=%.2f ", Kpp);
					OLED_ShowString(3, 1, Text);
					snprintf(Text,30,"Kdd=%.2f ", Kdd);
					OLED_ShowString(4, 1, Text);
				}
				else if(Selt_para == 1)
				{
					snprintf(Text,30,"Kp=%.2f ", Kp);
					OLED_ShowString(1, 1, Text);
					snprintf(Text,30,"Kd=%.2f ", Kd);
					OLED_ShowStringReverse(2, 1, Text);
					snprintf(Text,30,"Kpp=%.2f ", Kpp);
					OLED_ShowString(3, 1, Text);
					snprintf(Text,30,"Kdd=%.2f ", Kdd);
					OLED_ShowString(4, 1, Text);
				}
				else if(Selt_para == 2)
				{
					snprintf(Text,30,"Kp=%.2f ", Kp);
					OLED_ShowString(1, 1, Text);
					snprintf(Text,30,"Kd=%.2f ", Kd);
					OLED_ShowString(2, 1, Text);
					snprintf(Text,30,"Kpp=%.2f ", Kpp);
					OLED_ShowStringReverse(3, 1, Text);
					snprintf(Text,30,"Kdd=%.2f ", Kdd);
					OLED_ShowString(4, 1, Text);
				}
				else if(Selt_para == 3)
				{
					snprintf(Text,30,"Kp=%.2f ", Kp);
					OLED_ShowString(1, 1, Text);
					snprintf(Text,30,"Kd=%.2f ", Kd);
					OLED_ShowString(2, 1, Text);
					snprintf(Text,30,"Kpp=%.2f ", Kpp);
					OLED_ShowString(3, 1, Text);
					snprintf(Text,30,"Kdd=%.2f ", Kdd);
					OLED_ShowStringReverse(4, 1, Text);
				}
			}
			else if(State == 1 || State == 2)
			{
				snprintf(Text,30,"State = %d   ", State);
				OLED_ShowString(1, 1, Text);
				snprintf(Text,30,"L=%.0f   R=%.0f   ",MotorBL.target,MotorAR.target);
				OLED_ShowString(2, 1, Text);
				snprintf(Text,30,"yaw_hmc=%.3f   ", yaw_hmc);
				OLED_ShowString(3, 1, Text);
			}
			
			MPUDisp_Flag = 0;
		}
		
		if(Data_Flag == 1)
		{
			// 获取原始数据
			MPU6050_GetData();		
			HMC5883L_GetData(&hmc_x, &hmc_y, &hmc_z);
			
			// 通过陀螺仪计算角度，这个*0.005和EXTI的频率有关的，现在EXTI的频率是20ms一次，所以
			// 这里是*0.02，如果改变了EXTI的频率这里也要变的，EXTI频率的改变方法在MPU6050_Init()里有写
			roll_gyro += (float)gx / 16.4 * 0.02;
			pitch_gyro += (float)gy / 16.4 * 0.02;
			yaw_gyro += ((float)gz - (float)gyro_zero_z) / 16.4 * 0.02;
			
			// 计算加速度计角度
			roll_acc = atan((float)ay/az) * 57.296;
			pitch_acc = - atan((float)ax/az) * 57.296;
			yaw_acc = atan((float)ay/ax) * 57.296;
			
			// 计算磁力偏航角
			hmc_x_cal = ((float)hmc_x - HMC5883L_Cali_Res.offset_x) * HMC5883L_Cali_Res.scale_x;
			hmc_y_cal = ((float)hmc_y - HMC5883L_Cali_Res.offset_y) * HMC5883L_Cali_Res.scale_y;
			
			yaw_hmc = atan2f(hmc_y_cal, hmc_x_cal)*57.296f;		
			
			// 卡尔曼滤波融合角度		
			roll_Kalman = Kalman_Filter(&KF_Roll, roll_acc, (float)gx / 16.4 );
			pitch_Kalman = Kalman_Filter(&KF_Pitch, pitch_acc, (float)gy / 16.4 );
			yaw_Kalman = Kalman_Filter(&KF_Yaw, yaw_hmc, ((float)gz - (float)gyro_zero_z) / 16.4 );
			
			Data_Flag = 0;
		}
		
/*----------第一题--------------------------------------------------------------------*/
		
		// 测试角度环
		if(State == -1)
		{
			angle.target = -150;
			ANGLOOP = 1;
		}
		
		if(State == 0)
		{
			pid_set_tar_speed(0,0);
			ANGStra = yaw_hmc;
			angle.target = ANGStra;
			
		}
		
		if(State == 1)
		{
			ANGLOOP = 1;
			// 让小车沿着一开始摆放的方向行驶，不使用角度环
			if( (total_left + total_right)/2 > 7000)
			{
				State = 2;
				// 关闭角度环，启动循迹环
				ANGLOOP = 0;
			}
			else if( (total_left + total_right)/2 > 5500 )
			{
				pid_set_base_speed(5);
			}
			else
			{
				pid_set_base_speed(15);
			}
			
		}
		
		if(State == 2)
		{
			
		}
		
		key_proc();

		
/*------------------------------------------------------------------------------------*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
