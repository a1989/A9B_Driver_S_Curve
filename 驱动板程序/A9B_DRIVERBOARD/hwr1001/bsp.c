#include "bsp.h"
#include "includes.h"

void BSP_Initializes (void)
{
	if (bsp_init_flag == 0)
	{
		//At24c512_Init();//初始化EEP
		Driver_Board_Parameter_Init();//参数读取，必须放在最前面
		Driver_Can_Init();//CAN滤波器初始化
		Drv8711_Init(); //DRV8711参数写入，默认参数
		PID_Init();//电机PID参数初始化
		HAL_TIM_OC_Stop_IT (&htim2, TIM_CHANNEL_1);//停止PWM比较输出
		//HAL_TIM_OC_Start_IT ( &htim2,TIM_CHANNEL_1 );
		TIM3_Encoder_Switch (1);//开编码器
		TIM4_IT_Interrupt_Switch (1);//开启定时器4中断
		TIM1_IT_Interrupt_Switch (1);//开启定时器1中断
		HAL_CAN_Receive_IT (&hcan, CAN_FIFO0); //开CAN接收中断

		HAL_ADCEx_Calibration_Start (&hadc1);
		HAL_ADC_Start_DMA (&hadc1, ADC_Get_Info, 3);
		Uart_Receive_Interrupt_Switch (&huart1, &uart1_receive_byte);//开中断
		bsp_init_flag = 1;
		printf ("\r\n hwrobot steper motor driver board bsp init is ok !");
	}
}
