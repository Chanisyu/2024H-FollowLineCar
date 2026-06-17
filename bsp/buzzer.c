/**
 * @file    buzzer.c
 * @brief   低电平有效蜂鸣器的非阻塞鸣叫控制。
 *
 * 文件结构：
 *   1. Buzzer_On()     - 拉低蜂鸣器引脚开始鸣叫
 *   2. Buzzer_Off()    - 拉高蜂鸣器引脚停止鸣叫
 *   3. Buzzer_Start()  - 记录开始时间并启动一次鸣叫
 *   4. Buzzer_Task()   - 在主循环中轮询，到时自动关闭蜂鸣器
 */

#include "buzzer.h"

static uint8_t buzzer_busy=0;          // 跨帧保存鸣叫状态，避免阻塞式延时卡住主循环。
static uint32_t buzzer_start_tick=0;   // 记录 HAL tick 起点，用于计算持续时间。
static uint32_t buzzer_time=0;         // 本次鸣叫持续时间，单位 ms。

static void Buzzer_On(void)
{
  HAL_GPIO_WritePin(BUZZER_PORT,BUZZER_PIN,GPIO_PIN_RESET);
}

static void Buzzer_Off(void)
{
  HAL_GPIO_WritePin(BUZZER_PORT,BUZZER_PIN,GPIO_PIN_SET);
}

void Buzzer_Start(uint32_t ms)
{
  Buzzer_On();
  buzzer_start_tick=HAL_GetTick();
  buzzer_time=ms;
  buzzer_busy=1;
}

void Buzzer_Task(void)
{
  if(buzzer_busy)
  {
    if(HAL_GetTick()-buzzer_start_tick>=buzzer_time)
    {
      Buzzer_Off();
      buzzer_busy=0;
    }
  }
}
