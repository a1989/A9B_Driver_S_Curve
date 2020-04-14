#include "encoder.h"
#include "includes.h"

EncoderType GetEncoder;
s32 Location_Cnt = 0;

// 累加编码器的值
void Encoder_Total (void)
{
	if (encoder_cnt_direction_flag == 1)
	{
		Location_Cnt = (s32)(Location_Cnt + (GetEncoder.V3));
	}
	else if (encoder_cnt_direction_flag == 0)
	{
		Location_Cnt = (s32)(Location_Cnt + (-(GetEncoder.V3)));
	}
}

void Get_Encoder_3 (void)
{
    s32 CNT3_temp,CNT3_last;

    GetEncoder.cnt3 = __HAL_TIM_GET_COUNTER (&htim3);
    CNT3_last = GetEncoder.CNT3;
    CNT3_temp = GetEncoder.rcnt3 * prd + GetEncoder.cnt3;
    GetEncoder.V3 = CNT3_temp - CNT3_last;
    while ((s32)(GetEncoder.V3) > Vbreak)
    {
        GetEncoder.rcnt3--;
        CNT3_temp = GetEncoder.rcnt3 * prd + GetEncoder.cnt3;
        GetEncoder.V3 = CNT3_temp - CNT3_last;
    }
    while ((s32)(GetEncoder.V3) < -Vbreak)
    {
        GetEncoder.rcnt3++;
        CNT3_temp = GetEncoder.rcnt3 * prd + GetEncoder.cnt3;
        GetEncoder.V3 = CNT3_temp - CNT3_last;
    }
    GetEncoder.CNT3 = CNT3_temp;
}

void Encoder_Config (void)
{
    Get_Encoder_3();
}

void TIM3_Encoder_Switch (uint8_t VALUE)
{
    __HAL_TIM_SET_COUNTER (&htimx_Encoder, 0);
    if (VALUE == 1)
    {
        if (HAL_TIM_Encoder_Start (&htimx_Encoder, TIM_CHANNEL_ALL) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_TIM_Encoder_Start (&htimx_Encoder, TIM_CHANNEL_ALL) != HAL_OK)
        {
            Error_Handler();
        }
    }
    else
    {
        if (HAL_TIM_Encoder_Stop (&htimx_Encoder, TIM_CHANNEL_ALL) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_TIM_Encoder_Stop (&htimx_Encoder, TIM_CHANNEL_ALL) != HAL_OK)
        {
            Error_Handler();
        }
    }
}
