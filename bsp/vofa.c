#include "vofa.h"
#include "OLED.h"
#include "stdio.h"
#include "pid.h"

uint8_t vofa_rx_ch;
char vofa_rx_line[64];
uint8_t vofa_rx_idx = 0;

static UART_HandleTypeDef *vofa_uart = NULL;

/*
 * 初始化 VOFA 串口通信。
 * huart：用于和 VOFA 通信的 UART 句柄，函数会保存它并启动首次中断接收。
 */
void VOFA_Init(UART_HandleTypeDef *huart)
{
	vofa_uart = huart;
	HAL_UART_Receive_IT(vofa_uart,&vofa_rx_ch,1);
}

/*
 * 向 VOFA 发送速度环波形数据。
 * target_speed：目标速度，作为 VOFA 第一列数据。
 * real_speed：实际速度，作为 VOFA 第二列数据。
 */
void VOFA_SendSpeedLoop(float target_speed,
                        float real_speed)
{
	if (vofa_uart == NULL)
	{
		return;
	}

	char buf[100];

	int len = snprintf(buf,
										 sizeof(buf),
										 "%f,%f\n",
										 target_speed,
										 real_speed);

	if (len > 0 && len < sizeof(buf))
	{
			HAL_UART_Transmit(vofa_uart, (uint8_t *)buf, len, 20);
	}
}

/*
 * HAL 串口接收完成回调函数。
 * huart：触发回调的 UART 句柄；这里只处理 USART1 收到的数据。
 *
 * 作用：
 *     每次接收 1 个字符，拼成一行命令；遇到 '\n' 后调用 VOFA_ParseLine()。
 */
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

/*
 * 解析 VOFA 发来的一行命令。
 * line：命令字符串，支持 "T=..."、"KP=..."、"KI=..."、"KD=..."。
 *
 * 作用：
 *     T  修改左右轮目标速度；
 *     KP 修改左轮速度环 P 参数；
 *     KI 修改左轮速度环 I 参数；
 *     KD 修改左轮速度环 D 参数。
 */
void VOFA_ParseLine(char *line)
{
    float value;

    if (sscanf(line, "T=%f", &value) == 1)
    {
			snprintf(Text,10,"%f",value);
			OLED_ShowString(4,1,Text);
			pid_set_tar_speed(value,value);
    }
    else if (sscanf(line, "KP=%f", &value) == 1)
    {
      MotorBL.p = value;
    }
    else if (sscanf(line, "KI=%f", &value) == 1)
    {
      MotorBL.i = value;
    }
    else if (sscanf(line, "KD=%f", &value) == 1)
    {
      MotorBL.d = value;
    }
}
