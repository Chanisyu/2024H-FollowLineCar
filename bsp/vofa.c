#include "vofa.h"
#include "OLED.h"
#include "stdio.h"
#include "pid.h"

uint8_t vofa_rx_ch;
char vofa_rx_line[64];
uint8_t vofa_rx_idx = 0;

static UART_HandleTypeDef *vofa_uart = NULL;

void VOFA_Init(UART_HandleTypeDef *huart)
{
	vofa_uart = huart;
	HAL_UART_Receive_IT(vofa_uart,&vofa_rx_ch,1);
}

void VOFA_SendSpeedLoop(volatile pid_t *pid)
{
	if (vofa_uart == NULL)
	{
		return;
	}

	if (pid == NULL)
	{
		return;
	}

	char buf[128];

	int len = snprintf(buf,
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

	if (len > 0 && len < sizeof(buf))
	{
			HAL_UART_Transmit(vofa_uart, (uint8_t *)buf, len, 20);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		if (vofa_rx_ch == '\n')
		{
			if (vofa_rx_idx > 0)
			{
				vofa_rx_line[vofa_rx_idx] = '\0';
				VOFA_ParseLine(vofa_rx_line);
				vofa_rx_idx = 0;
			}
		}
		else if (vofa_rx_ch == '\r')
		{
			/* Ignore CR so both LF and CRLF commands work. */
		}
		else
		{
			if (vofa_rx_idx < sizeof(vofa_rx_line) - 1)
			{
				vofa_rx_line[vofa_rx_idx++] = vofa_rx_ch;
			}
			else
			{
				vofa_rx_idx = 0;
			}
		}

		HAL_UART_Receive_IT(vofa_uart, &vofa_rx_ch, 1);
	}
}

void VOFA_ParseLine(char *line)
{
    float p;
    float i;
    float d;
    float value;

    if (strcmp(line, "STATUS") == 0)
    {
        VOFA_SendSpeedLoop(&MotorBL);
    }
    else if (sscanf(line, "SET P:%f I:%f D:%f", &p, &i, &d) == 3)
    {
        MotorBL.p = p;
        MotorBL.i = i;
        MotorBL.d = d;
        MotorAR.p = p;
        MotorAR.i = i;
        MotorAR.d = d;
    }
    else if (sscanf(line, "T=%f", &value) == 1)
    {
			snprintf(Text,10,"%f",value);
			OLED_ShowString(4,1,Text);
			pid_set_tar_speed(value,value);
    }
    else if (sscanf(line, "KP=%f", &value) == 1)
    {
      MotorBL.p = value;
      MotorAR.p = value;
    }
    else if (sscanf(line, "KI=%f", &value) == 1)
    {
      MotorBL.i = value;
      MotorAR.i = value;
    }
    else if (sscanf(line, "KD=%f", &value) == 1)
    {
      MotorBL.d = value;
      MotorAR.d = value;
    }
//    else if (strcmp(line, "STOP") == 0)
//    {
//        target_speed = 0.0f;
//        speed_pid.integral = 0.0f;
//        speed_pid.output = 0.0f;
//    }
//    else if (strcmp(line, "RESETI") == 0)
//    {
//        speed_pid.integral = 0.0f;
//    }
}
