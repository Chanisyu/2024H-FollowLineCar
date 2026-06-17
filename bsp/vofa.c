/*
   * 速度环曲线格式为 target,real\n，第一列为目标速度，第二列为实测速度。
   */
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

  /*
   * 先刷新灰度传感器数据，再发送 8 路原始 ADC、归一化和压线判定数组。
   */
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
      /* 忽略 CR 字符，以 LF 作为命令结束标志。 */
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
