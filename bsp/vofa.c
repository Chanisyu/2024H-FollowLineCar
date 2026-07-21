/**
 * @file    vofa.c
 * @brief   VOFA 串口命令解析和调试波形发送。
 *
 * 文件结构：
 *   1. VOFA_Init()                 - 启动串口接收中断
 *   2. VOFA_SetSpeedTuneDefaults() - 恢复速度调参默认状态
 *   3. VOFA_SendSpeedLoop()        - 发送速度环波形数据
 *   4. VOFA_SendGrayArrays()       - 发送灰度传感器数组数据
 *   5. HAL_UART_RxCpltCallback()   - 接收 1 字节并拼接成命令行
 *   6. VOFA_ParseLine()            - 解析串口命令并修改参数
 */

#include "vofa.h"
#include "OLED.h"
#include "stdio.h"
#include "pid.h"
#include "gray_track.h"

#define VOFA_STREAM_SPEED 0
#define VOFA_STREAM_GRAY  1

uint8_t vofa_rx_ch;
char vofa_rx_line[64];
uint8_t vofa_rx_idx=0;
uint8_t vofa_stream_mode=VOFA_STREAM_SPEED;
uint8_t vofa_speed_hold=0;

static UART_HandleTypeDef *vofa_uart=NULL;

void VOFA_SetSpeedTuneDefaults(void)
{
  vofa_stream_mode=VOFA_STREAM_SPEED;
  vofa_speed_hold=0;
  pid_set_tar_speed(0,0);
  pid_reset_speed_loop();
}

void VOFA_Init(UART_HandleTypeDef *huart)
{
  vofa_uart=huart;
  VOFA_SetSpeedTuneDefaults();
  HAL_UART_Receive_IT(vofa_uart,&vofa_rx_ch,1);
}

void VOFA_SendSpeedLoop(float target_speed,float real_speed)
{
  if(vofa_uart==NULL)
  {
    return;
  }

  char buf[100];
  int len=snprintf(buf,sizeof(buf),"%f,%f\n",target_speed,real_speed);

  if(len>0&&len<sizeof(buf))
  {
    HAL_UART_Transmit(vofa_uart,(uint8_t *)buf,len,20);
  }
}

void VOFA_SendGrayArrays(void)
{
  if(vofa_uart==NULL)
  {
    return;
  }

  gray_sensor_update();

  char buf[256];
  int len=snprintf(buf,sizeof(buf),
                   "A,%u,%u,%u,%u,%u,%u,%u,%u\r\n"
                   "N,%u,%u,%u,%u,%u,%u,%u,%u\r\n"
                   "D,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                   gray_analog[0],gray_analog[1],gray_analog[2],gray_analog[3],
                   gray_analog[4],gray_analog[5],gray_analog[6],gray_analog[7],
                   gray_normal[0],gray_normal[1],gray_normal[2],gray_normal[3],
                   gray_normal[4],gray_normal[5],gray_normal[6],gray_normal[7],
                   gray_dark[0],gray_dark[1],gray_dark[2],gray_dark[3],
                   gray_dark[4],gray_dark[5],gray_dark[6],gray_dark[7]);

  if(len>0&&len<sizeof(buf))
  {
    HAL_UART_Transmit(vofa_uart,(uint8_t *)buf,len,100);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {
    if(vofa_rx_ch=='\n')
    {
      if(vofa_rx_idx>0)
      {
        vofa_rx_line[vofa_rx_idx]='\0';
        VOFA_ParseLine(vofa_rx_line);
        vofa_rx_idx=0;
      }
    }
    else if(vofa_rx_ch=='\r')
    {
    }
    else
    {
      if(vofa_rx_idx<sizeof(vofa_rx_line)-1)
      {
        vofa_rx_line[vofa_rx_idx++]=vofa_rx_ch;
      }
      else
      {
        vofa_rx_idx=0;
      }
    }

    HAL_UART_Receive_IT(vofa_uart,&vofa_rx_ch,1);
  }
}

void VOFA_ParseLine(char *line)
{
  float value;

  if(strcmp(line,"MODE=SPEED")==0)
  {
    vofa_stream_mode=VOFA_STREAM_SPEED;
  }
  else if(strcmp(line,"MODE=GRAY")==0)
  {
    vofa_stream_mode=VOFA_STREAM_GRAY;
  }
  else if(strcmp(line,"STOP")==0)
  {
    vofa_speed_hold=0;
    pid_set_tar_speed(0,0);
    pid_reset_speed_loop();
  }
  else if(strcmp(line,"RSTPID")==0)
  {
    pid_reset_speed_loop();
  }
  else if(sscanf(line,"T=%f",&value)==1)
  {
    vofa_stream_mode=VOFA_STREAM_SPEED;
    vofa_speed_hold=1;
    pid_set_tar_speed(value,value);
    pid_reset_speed_loop();
    snprintf(Text,30,"T=%.2f   ",value);
    OLED_ShowString(4,1,Text);
  }
  else if(sscanf(line,"KP=%f",&value)==1)
  {
    MotorBL.p=value;
    MotorAR.p=value;
  }
  else if(sscanf(line,"KI=%f",&value)==1)
  {
    MotorBL.i=value;
    MotorAR.i=value;
  }
  else if(sscanf(line,"KD=%f",&value)==1)
  {
    MotorBL.d=value;
    MotorAR.d=value;
  }
}
