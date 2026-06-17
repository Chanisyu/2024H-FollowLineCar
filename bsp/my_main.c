/**
 * @file    my_main.c
 * @brief   小车主流程、状态机、数据显示、传感器计算和中断回调。
 *
 * 文件结构：
 *   1. setup()                         - 初始化 OLED、电机、PID、IMU、磁力计和定时器
 *   2. loop()                          - 主循环状态调度和后台任务
 *   3. key_proc()                      - 按键调参和运行状态切换
 *   4. OLED_proc()                     - OLED 页面显示
 *   5. Data_proc()                     - IMU/磁力计数据计算和滤波融合
 *   6. VOFA_proc()                     - VOFA 调试数据发送入口
 *   7. HAL_TIM_PeriodElapsedCallback() - 10ms 定时器任务
 *   8. HAL_GPIO_EXTI_Callback()        - MPU6050 数据就绪中断入口
 */

#include "my_main.h"

char Text[30];
volatile int16_t right_speed;
volatile int16_t left_speed;

volatile uint8_t Disp_Flag=0;   // OLED 刷新标志，由 10ms 定时器置位。
volatile uint8_t Data_Flag=0;   // 传感器计算标志，由 MPU6050 INT 外部中断置位。
volatile uint8_t VOFA_Flag=0;   // VOFA 调试发送标志，预留给上位机波形输出。
volatile uint8_t PIDConFlag=0;  // PID 控制标志，由 10ms 定时器置位。

int8_t State=-1;      // 小车运行状态：-1 待机显示，0 调参，1 直行段，2 循迹段。
uint8_t Selt_para=0;  // OLED/按键调参时选中的参数：0=Kp，1=Kd，2=Kpp，3=Kdd。
uint8_t View=0;       // 页面显示状态，预留给后续多页面扩展。

uint8_t ANGLOOP=0;    // 是否启用角度环：1 使用磁力计航向闭环，0 不使用。
int16_t ANGStra;      // 起步时记录的直行航向角，用作角度环目标。

HMC5883L_CalibrationResult HMC5883L_Cali_Res;  // 磁力计软铁/硬铁校准结果。
float KddForANG=0;    // 角度环陀螺仪阻尼系数，越大越抑制转向摆动。
float KppForANG=0;    // 角度环非线性比例系数，越大越放大大角度误差。

void setup()
{
  // ------- 1. 初始化显示和执行机构 -------
  OLED_Init();
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  // TIM2 读取右轮编码器。
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);  // TIM3 读取左轮编码器。
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);  // 蜂鸣器低电平有效，默认关闭。

  MotorAR_set(0,1);
  MotorAR_start();
  MotorBL_set(0,1);
  MotorBL_start();

  // ------- 2. 初始化控制参数 -------
  pid_Init(&MotorAR,DELTA_PID,17,5,0.5);
  pid_Init(&MotorBL,DELTA_PID,17,5,0.5);
  pid_Init(&angle,POSITION_PID,0.9,0,18);
  pid_set_tar_speed(0,0);

  // ------- 3. 初始化姿态传感器 -------
  MPU6050_Init();
  HAL_Delay(50);
  HMC5883L_Init();

  calibrate_gyro();  // 静止采样 gz 零漂，后续 yaw_gyro 和角度环阻尼都会用到。
  // HMC5883L_Calibration_RunBlocking(&HMC5883L_Cali_Res,600,20,App_ReadSample,App_DelayMs,0);

  // ------- 4. 启动周期任务和串口调试 -------
  HAL_TIM_Base_Start_IT(&htim4);  // TIM4 每 10ms 触发一次，用于 PID、按键和显示节拍。
  VOFA_Init(&huart1);
}

void loop()
{
  // ------- 1. 运行状态机 -------
  if(State==0)
  {
    pid_set_tar_speed(0,0);
    ANGStra=yaw_hmc;
    angle.target=ANGStra;
  }

  if(State==1)
  {
    // 按编码器里程分段给速度，超过阈值后进入灰度循迹段。
    if((total_left+total_right)/2>7500)
    {
      State=2;
      track_reset();
      pid_set_tar_speed(18,18);
      ANGLOOP=0;
    }
    else if((total_left+total_right)/2>5500)
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

  if(State==2)
  {
  }

  key_proc();

  // ------- 2. 后台任务 -------
  OLED_proc();
  Data_proc();
  pid_control();
//  VOFA_proc();
}

// ================================================================
// 1. 按键处理
//
//    KEY0 短按切换待机/调参，长按进入运行状态。
//    调参页面中 KEY1 选择参数，KEY2 增大参数，KEY3 减小参数。
// ================================================================
void key_proc()
{
  if(KEYS[0].key_short==1)
  {
    State++;
    if(State>=1)
    {
      State=-1;
    }
    OLED_Clear();
    KEYS[0].key_short=0;
  }
  else if(KEYS[0].key_long==1)
  {
    State=1;
    OLED_Clear();
    KEYS[0].key_long=0;
  }

  if(State==0)
  {
    if(KEYS[1].key_short==1)
    {
      Selt_para++;
      if(Selt_para>=4)
      {
        Selt_para=0;
      }
      KEYS[1].key_short=0;
    }
    else if(KEYS[2].key_short==1)
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
      KEYS[2].key_short=0;
    }
    else if(KEYS[3].key_short==1)
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
      KEYS[3].key_short=0;
    }
  }
}

// ================================================================
// 2. OLED 显示
//
//    Disp_Flag 由 TIM4 每 10ms 置位，这里只在置位时刷新，避免主循环重复刷屏。
//    调参页面用反白显示当前选中参数，便于现场调 PID。
// ================================================================
void OLED_proc()
{
  if(Disp_Flag==0)  return;

  if(State==-1)
  {
    snprintf(Text,30,"yaw_hmc=%.3f   ",yaw_hmc);
    OLED_ShowString(1,1,Text);
    snprintf(Text,30,"gz=%d   ",gz);
    OLED_ShowString(2,1,Text);
  }
  else if(State==0)
  {
    if(Selt_para==0)
    {
      snprintf(Text,30,"Kp=%.2f ",Kp);
      OLED_ShowStringReverse(1,1,Text);
      snprintf(Text,30,"Kd=%.2f ",Kd);
      OLED_ShowString(2,1,Text);
      snprintf(Text,30,"Kpp=%.2f ",Kpp);
      OLED_ShowString(3,1,Text);
      snprintf(Text,30,"Kdd=%.2f ",Kdd);
      OLED_ShowString(4,1,Text);
    }
    else if(Selt_para==1)
    {
      snprintf(Text,30,"Kp=%.2f ",Kp);
      OLED_ShowString(1,1,Text);
      snprintf(Text,30,"Kd=%.2f ",Kd);
      OLED_ShowStringReverse(2,1,Text);
      snprintf(Text,30,"Kpp=%.2f ",Kpp);
      OLED_ShowString(3,1,Text);
      snprintf(Text,30,"Kdd=%.2f ",Kdd);
      OLED_ShowString(4,1,Text);
    }
    else if(Selt_para==2)
    {
      snprintf(Text,30,"Kp=%.2f ",Kp);
      OLED_ShowString(1,1,Text);
      snprintf(Text,30,"Kd=%.2f ",Kd);
      OLED_ShowString(2,1,Text);
      snprintf(Text,30,"Kpp=%.2f ",Kpp);
      OLED_ShowStringReverse(3,1,Text);
      snprintf(Text,30,"Kdd=%.2f ",Kdd);
      OLED_ShowString(4,1,Text);
    }
    else if(Selt_para==3)
    {
      snprintf(Text,30,"Kp=%.2f ",Kp);
      OLED_ShowString(1,1,Text);
      snprintf(Text,30,"Kd=%.2f ",Kd);
      OLED_ShowString(2,1,Text);
      snprintf(Text,30,"Kpp=%.2f ",Kpp);
      OLED_ShowString(3,1,Text);
      snprintf(Text,30,"Kdd=%.2f ",Kdd);
      OLED_ShowStringReverse(4,1,Text);
    }
  }
  else if(State==1||State==2)
  {
    snprintf(Text,30,"State = %d   ",State);
    OLED_ShowString(1,1,Text);
    snprintf(Text,30,"L=%.0f   R=%.0f   ",MotorBL.now,MotorAR.now);
    OLED_ShowString(2,1,Text);
    snprintf(Text,30,"yaw_hmc=%.3f   ",yaw_hmc);
    OLED_ShowString(3,1,Text);
  }

  Disp_Flag=0;
}

// ================================================================
// 3. 姿态和航向数据计算
//
//    Data_Flag 由 MPU6050 INT 外部中断置位，当前节拍约 10ms。
//    陀螺仪积分负责短期响应，磁力计航向负责长期方向参考，卡尔曼滤波用于融合。
// ================================================================
void Data_proc()
{
  if(Data_Flag==0)  return;

  // ------- 1. 获取原始传感器数据 -------
  MPU6050_GetData();
  HMC5883L_GetData(&hmc_x,&hmc_y,&hmc_z);

  // ------- 2. 陀螺仪积分角 -------
  // 16.4 LSB/(deg/s) 对应 +/-2000dps 量程；0.01s 对应 10ms 中断周期。
  roll_gyro+=(float)gx/16.4*0.01;
  pitch_gyro+=(float)gy/16.4*0.01;
  yaw_gyro+=((float)gz-(float)gyro_zero_z)/16.4*0.01;

  // ------- 3. 加速度计姿态角 -------
  roll_acc=atan((float)ay/az)*57.296;
  pitch_acc=-atan((float)ax/az)*57.296;
  yaw_acc=atan((float)ay/ax)*57.296;

  // ------- 4. 磁力计航向角 -------
  hmc_x_cal=((float)hmc_x-HMC5883L_Cali_Res.offset_x)*HMC5883L_Cali_Res.scale_x;
  hmc_y_cal=((float)hmc_y-HMC5883L_Cali_Res.offset_y)*HMC5883L_Cali_Res.scale_y;
  yaw_hmc=atan2f(hmc_y_cal,hmc_x_cal)*57.296f;

  // ------- 5. 卡尔曼滤波融合 -------
  roll_Kalman=Kalman_Filter(&KF_Roll,roll_acc,(float)gx/16.4);
  pitch_Kalman=Kalman_Filter(&KF_Pitch,pitch_acc,(float)gy/16.4);
  yaw_Kalman=Kalman_Filter(&KF_Yaw,yaw_hmc,((float)gz-(float)gyro_zero_z)/16.4);

  Data_Flag=0;
}

void VOFA_proc()
{
  static uint32_t last_send_tick=0;
  uint32_t now_tick=HAL_GetTick();

  if(now_tick-last_send_tick<1000)
  {
    return;
  }

  last_send_tick=now_tick;
  VOFA_SendGrayArrays();
}

// ================================================================
// 4. TIM4 周期中断
//
//    TIM4 每 10ms 触发一次：
//      1) 置位 PID 控制标志；
//      2) 读取并清零左右轮编码器计数；
//      3) 扫描按键并置位 OLED 刷新标志。
// ================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM4)
  {
    PIDConFlag=1;
		key_task();
    Disp_Flag=1;

    MotorAR.now=(int16_t)(__HAL_TIM_GET_COUNTER(&htim2));
    __HAL_TIM_SET_COUNTER(&htim2,0);
    total_right+=(uint32_t)MotorAR.now;

    MotorBL.now=-(int16_t)(__HAL_TIM_GET_COUNTER(&htim3));
    __HAL_TIM_SET_COUNTER(&htim3,0);
    total_left+=(uint32_t)MotorBL.now;
  }
}

// MPU6050 的 INT 引脚接 PB5，数据就绪时置位 Data_Flag，计算放到主循环中执行。
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==GPIO_PIN_5)
  {
    Data_Flag=1;
  }
}


