#include "key.h"

volatile struct key KEYS[4] = {0};

void key_task()
{
		KEYS[0].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
		KEYS[1].key_sta = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
		KEYS[2].key_sta = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);
		KEYS[3].key_sta = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12);
		
		for(uint8_t i=0;i<4;i++)
		{
			switch(KEYS[i].judge_sta)
			{
				case 0:
				{
					if(KEYS[i].key_sta == 0)
					{
						KEYS[i].judge_sta = 1;
						KEYS[i].key_time = 0;
					}
					break;
				}
				case 1:
				{
					if(KEYS[i].key_sta == 0)
					{
						KEYS[i].key_time+=10;
					}
					else if(KEYS[i].key_sta == 1)
					{
						if(KEYS[i].key_time >= 20 && KEYS[i].key_time<=990)
						{
							KEYS[i].key_short = 1;
						}
						else if(KEYS[i].key_time > 990)
						{
							KEYS[i].key_long = 1;
						}
						
						KEYS[i].judge_sta = 0;
						KEYS[i].key_time = 0;
					}
					break;
				}
			}
		}
}
