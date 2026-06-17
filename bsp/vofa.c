/**
 * @file    vofa.c
 * @brief   VOFA 串口命令解析和调试波形发送。
 *
 * 文件结构：
 *   1. VOFA_Init()              - 启动串口接收中断
 *   2. VOFA_SendSpeedLoop()      - 发送速度环波形数据
 *   3. HAL_UART_RxCpltCallback() - 接收 1 字节并拼接成命令行
 *   4. VOFA_ParseLine()          - 解析串口命令并修改参数
 */

#include "vofa.h"
#include "OLED.h"
#include "stdio.h"
#include "pid.h"

uint8_t vofa_rx_ch;
char vofa_rx_line[64];
uint8_t vofa_rx_idx=0;

static UART_HandleTypeDef *vofa_uart=NULL;

void VOFA_Init(UART_HandleTypeDef *huart)
{
  vofa_uart=huart;
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

  if(sscanf(line,"T=%f",&value)==1)
  {
    snprintf(Text,10,"%f",value);
    OLED_ShowString(4,1,Text);
    pid_set_tar_speed(value,value);
  }
  else if(sscanf(line,"KP=%f",&value)==1)
  {
    MotorBL.p=value;
  }
  else if(sscanf(line,"KI=%f",&value)==1)
  {
    MotorBL.i=value;
  }
  else if(sscanf(line,"KD=%f",&value)==1)
  {
    MotorBL.d=value;
  }
}
