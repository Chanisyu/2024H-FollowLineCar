#include "buzzer.h"

static uint8_t  buzzer_busy = 0;
static uint32_t buzzer_start_tick = 0;
static uint32_t buzzer_time = 0;

static void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

static void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

void Buzzer_Start(uint32_t ms)
{
    Buzzer_On();
    buzzer_start_tick = HAL_GetTick();
    buzzer_time = ms;
    buzzer_busy = 1;
}

void Buzzer_Task(void)
{
    if(buzzer_busy)
    {
        if(HAL_GetTick() - buzzer_start_tick >= buzzer_time)
        {
            Buzzer_Off();
            buzzer_busy = 0;
        }
    }
}
