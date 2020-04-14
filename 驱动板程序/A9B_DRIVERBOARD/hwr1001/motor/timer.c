#include "timer.h"
#include "includes.h"

void TIM4_IT_Interrupt_Switch (uint8_t VALUE)
{
    __HAL_TIM_CLEAR_IT (&htim4, TIM_IT_UPDATE);
    if (VALUE == 1)
    {
        if (HAL_TIM_Base_Start_IT (&htim4) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else
    {
        if (HAL_TIM_Base_Stop_IT (&htim4) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void TIM1_IT_Interrupt_Switch (uint8_t VALUE)
{
    __HAL_TIM_CLEAR_IT (&htim1, TIM_IT_UPDATE);
    if (VALUE == 1)
    {
        if (HAL_TIM_Base_Start_IT (&htim1) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else
    {
        if (HAL_TIM_Base_Stop_IT (&htim1) != HAL_OK)
        {
            Error_Handler();
        }
    }
}
