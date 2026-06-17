/*
   * 兼容 8d24f0e 自动调参数据格式。
   * 每帧发送 tick、目标/实测速度、PID 输出、当前偏差及 P/I/D 参数，供 VOFA/LLM 分析速度环。
   *   tick,target,now,out,error,p,i,d\n
   */
  char buf[128];
  int len=snprintf(buf,
                   sizeof(buf),
                   "%lu,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n",
                   HAL_GetTick(),
                   pid->target,
                   pid->now,
                   pid->out,
                   pid->error[0],
                   pid->p,
                   pid->i,
                   pid->d);

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
       * 收到 LF 表示一行命令结束；补写 '\0' 后交给 VOFA_ParseLine() 解析。
       */
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
      /* STATUS 命令立即回传左轮速度环当前状态，便于上位机查看。 */
    VOFA_SendSpeedLoop(&MotorBL);
  }
  else if(sscanf(line,"SET P:%f I:%f D:%f",&p,&i,&d)==3)
  {
    /* SET P/I/D 同时更新左右轮速度环参数，保证两侧控制器使用同一组系数。 */
    MotorBL.p=p;
    MotorBL.i=i;
    MotorBL.d=d;
    MotorAR.p=p;
    MotorAR.i=i;
    MotorAR.d=d;
  }
  else if(strcmp(line,"MODE=SPEED")==0)
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
    /*
     * 设置目标速度后清除历史 PID 状态，避免旧积分项和旧偏差影响新的阶跃响应。
     */
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
  else if(sscanf(line,"TL=%f",&value)==1)
  {
    vofa_stream_mode=VOFA_STREAM_SPEED;
    vofa_speed_hold=1;
    MotorBL.target=value;
    pid_reset_motor(&MotorBL);
  }
  else if(sscanf(line,"TR=%f",&value)==1)
  {
    vofa_stream_mode=VOFA_STREAM_SPEED;
    vofa_speed_hold=1;
    MotorAR.target=value;
    pid_reset_motor(&MotorAR);
  }
  else if(sscanf(line,"KPL=%f",&value)==1)
  {
    MotorBL.p=value;
  }
  else if(sscanf(line,"KIL=%f",&value)==1)
  {
    MotorBL.i=value;
  }
  else if(sscanf(line,"KDL=%f",&value)==1)
  {
    MotorBL.d=value;
  }
  else if(sscanf(line,"KPR=%f",&value)==1)
  {
    MotorAR.p=value;
  }
  else if(sscanf(line,"KIR=%f",&value)==1)
  {
    MotorAR.i=value;
  }
  else if(sscanf(line,"KDR=%f",&value)==1)
  {
    MotorAR.d=value;
  }
}
