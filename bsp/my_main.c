#include "my_main.h"


char Text[30];
volatile int16_t right_speed;
volatile int16_t left_speed;

volatile uint8_t Disp_Flag = 0;
volatile uint8_t Data_Flag = 0;
volatile uint8_t VOFA_Flag = 0;
volatile uint8_t PIDConFlag = 0;

// 小车运行状态
int8_t State = -1;
// OLED/按键调参时选中的参数
uint8_t Selt_para = 0;
// 页面显示状态
uint8_t View = 0;

// 是否启用角度环
uint8_t ANGLOOP = 0;
// 起步时记录的直行航向角
int16_t ANGStra;

// 磁力计校准结果
HMC5883L_CalibrationResult HMC5883L_Cali_Res;
float KddForANG = 0;
float KppForANG = 0;

void setup()
{
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
  pid_Init(&MotorAR,DELTA_PID,17,5,0.5);
  pid_Init(&MotorBL,DELTA_PID,17,5,0.5);
	pid_Init(&angle,POSITION_PID,0.9,0,18);
	
  pid_set_tar_speed(0,0);
   
	MPU6050_Init();
	HAL_Delay(50);
  HMC5883L_Init();
	
	// MPU6050gz的零漂校准
	calibrate_gyro();
	// HMC5883L的软铁硬铁校准
	// HMC5883L_Calibration_RunBlocking(&HMC5883L_Cali_Res, 600, 20, App_ReadSample, App_DelayMs, 0);
	
	
	// 启动TIM4，10ms触发一次。
  HAL_TIM_Base_Start_IT(&htim4);

	VOFA_Init(&huart1);
	
  /*----------第一题--------------------------*/
  /*-----------------------------------------*/
}

void loop()
{		
  /*----------第一题---------------------------------------*/
	
	if(State == 0)
	{
		pid_set_tar_speed(0,0);
		ANGStra = yaw_hmc;
		angle.target = ANGStra;
	}
	
	if(State == 1)
	{
		// ANGLOOP = 1;
		// 让小车沿着一开始摆放的方向行驶，不使用角度环
		if( (total_left + total_right)/2 > 7500)
		{
			State = 2;
			// 关闭角度环，启动循迹环
			track_reset();
			pid_set_tar_speed(18,18);
			ANGLOOP = 0;
		}
		else if( (total_left + total_right)/2 > 5500 )
		{
			// pid_set_base_speed(30);
			pid_set_tar_speed(15,15);
		}
		else
		{
			// pid_set_base_speed(50);
			pid_set_tar_speed(25,25);
		}
	}
	
	if(State == 2)
	{
		
	}
	
	key_proc();
		
  /*----后台任务-------------------------------*/
	OLED_proc();
	Data_proc();
	pid_control();
//	VOFA_proc();
}


/**** 按键判断逻辑，负责各个界面下按键的作用逻辑。 ****/
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
					Kp+=0.1;
					break;
				}
				case 1:
				{
					Kd+=0.05;
					break;
				}
				case 2:
				{
					Kpp+=0.05;
					break;
				}
				case 3:
				{
					Kdd+=0.05;
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
					Kp-=0.1;
					break;
				}
				case 1:
				{
					Kd-=0.05;
					break;
				}
				case 2:
				{
					Kpp-=0.05;
					break;
				}
				case 3:
				{
					Kdd-=0.05;
					break;
				}
			}
			KEYS[3].key_short = 0;
		}
	}
}


/**** OLED显示逻辑，负责OLED显示内容 ****/
void OLED_proc()
{
	if(Disp_Flag == 0) return;
	
	if(State == -1)
	{
		snprintf(Text,30,"yaw_hmc=%.3f   ", yaw_hmc);
		OLED_ShowString(1, 1, Text);
		snprintf(Text,30,"gz=%d   ", gz);
		OLED_ShowString(2, 1, Text);
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
		snprintf(Text,30,"L=%.0f   R=%.0f   ",MotorBL.now,MotorAR.now);
		OLED_ShowString(2, 1, Text);
		snprintf(Text,30,"yaw_hmc=%.3f   ", yaw_hmc);
		OLED_ShowString(3, 1, Text);
	}
	
	Disp_Flag = 0;
}
	

/**** 数据计算逻辑，负责数据计算 ****/
void Data_proc()
{
	if(Data_Flag == 0) return;

	// 获取原始数据
	MPU6050_GetData();		
	HMC5883L_GetData(&hmc_x, &hmc_y, &hmc_z);
	
	// 通过陀螺仪计算角度，这个*0.01和EXTI的频率有关的，现在EXTI的频率是10ms一次，所以
	// 这里是*0.01，如果改变了EXTI的频率这里也要变的，EXTI频率的改变方法在MPU6050_Init()里有写
	roll_gyro += (float)gx / 16.4 * 0.01;
	pitch_gyro += (float)gy / 16.4 * 0.01;
	yaw_gyro += ((float)gz - (float)gyro_zero_z) / 16.4 * 0.01;
	
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

/**** 发送调试数据给VOFA上位机 ****/
void VOFA_proc()
{
	if(VOFA_Flag == 0) return;
}


/**** 定时器中断，每10ms触发一次，用于控制pid和按键扫描。 ****/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{	
		PIDConFlag=1;
		key_task();
		Disp_Flag = 1;
	}
}

/**** 外部中断，由MPU6050的INT引脚接到PB5，再开启外部中断，10ms触发一次，100Hz. ****/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_5)
	{
		Data_Flag = 1;
	}
}
