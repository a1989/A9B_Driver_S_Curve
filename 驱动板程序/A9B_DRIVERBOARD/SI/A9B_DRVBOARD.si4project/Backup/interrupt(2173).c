#include "interrupt.h"
#include "includes.h"

uint16_t time3_count = 0; //用于循环计数
uint8_t led_task_cnt = 50;

uint8_t can_send_data_task_cnt = 5;
uint8_t location_write_task_cnt = 60;

//uint8_t motor_limit_cnt = 70;
uint8_t motor_limit_cnt = 50;	//zyg


extern EncoderType GetEncoder;

uint8_t Uart_Receive_Interrupt_Switch (UART_HandleTypeDef* huart, uint8_t* uart_receive_data)
{
	uint8_t statu;

	__HAL_UNLOCK (huart);
	statu = HAL_UART_Receive_IT (huart, uart_receive_data, 1);
	return statu;
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef* huart) //串口中断回调函数
{
	uint8_t i;

	if (huart->Instance == huart1.Instance) //串口1调试串口
	{
		uart1_rxbuff[uart1_rxdata_cnt] = uart1_receive_byte;
		if (uart1_rxbuff[uart1_rxdata_cnt] == '\n')
		{
			//printf ("\r\n uart1_rxbuff: %s", uart1_rxbuff);
			HexStrToByte (uart1_rxbuff, uart1_rdata, UART1_RX_BUFF_LEN + 1); //数据转换
			uart1_debug_data_len =uart1_rdata[3] ;//数据长度
			if ((uart1_rdata[0] == UARTI_DEBUG_DATA_FRAME_START) && (uart1_rdata[1] == UARTI_DEBUG_DATA_FRAME_SECOND)) //接受到的是控制数据
			{
				memset (UARTI_DEBUG_DATA, 0, 256);
				for (i=0; i < uart1_debug_data_len; i++) //将数据填充进入数组
				{
					UARTI_DEBUG_DATA[i] = uart1_rdata[i];
					//printf ("\r\n uart1_rdata: %s", i);
				}
				// printf ("\r\n uart1_rdata: %x", PULLER_TO_CAR_DATA[4]);
				uart1_Receive_Right_flag = 1;
			}
			else
			{
				uart1_Receive_Right_flag = 0;
			}
			uart1_rxdata_cnt = 0;
			memset (uart1_rdata, 0, 256);
		}
		else
		{
			uart1_rxdata_cnt++;
		}
		if (uart1_rxdata_cnt > 255)
		{
			uart1_rxdata_cnt = 0;
		}
		Uart_Receive_Interrupt_Switch (&huart1, &uart1_receive_byte); //开中断,单字符接收
	}
}

//tim call back
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{

	if (htim->Instance == htim4.Instance) //tim4 interrupt
	{
		time3_count++;
		if ((time3_count % led_task_cnt ) == 0)
		{
			led_display_cnt_flag = 1;    // 1s 用于打印计时
		}
		if ((time3_count % can_send_data_task_cnt) == 0)
		{
			can_send_data_cnt_flag = 1;    //100ms发送一次
		}
		if ((time3_count % location_write_task_cnt) == 0)
		{
			location_write_cnt_flag = 1;    //1200ms发送一次
		}
		if ((motor_limit_flag == 1) || (motor_limit_flag == 2))
		{
			if ((time3_count % motor_limit_cnt) == 0)
			{
				motor_limit_cnt_flag = 1;
			}
		}
		if (time3_count > 65530)
		{
			time3_count = 0;
		}
		DevelopmentFramwork(); // 电机控制 周期20ms
	}
	if (htim->Instance == htim3.Instance) //tim3 interrupt
	{
		if (htim->Instance->CR1 & 0x0010) //小心注意
		{
			GetEncoder.rcnt3 -= 1;
		}
		else
		{
			GetEncoder.rcnt3 += 1;
		}
	}
	if (htim->Instance == htim1.Instance) //tim1 interrupt
	{
		//System_Control(); // 系统控制函数
	}
}

void HAL_CAN_ErrorCallback (CAN_HandleTypeDef* hcan)
{
	printf ("\r\nCAN出错\r\n");
	HAL_CAN_Receive_IT (hcan, CAN_FIFO0); //开CAN接收中断
}

//CAN中断函数
void HAL_CAN_RxCpltCallback (CAN_HandleTypeDef *hcan)
{
	uint8_t i = 0;
	/* 比较ID是否为0x1314 */
	if (RxMessage.StdId == driver_can_stdid)
	{
		for (i = 0; i < 8; i++)
		{
			MAIN_TO_DRIVER_DATA[i] = RxMessage.Data[i];
		}
		can_Receive_Right_flag = 1; // 接收正确
	}
	else
	{
		can_Receive_Right_flag = 0;
	}
	if (HAL_BUSY == HAL_CAN_Receive_IT (hcan, CAN_FIFO0)) //开启中断接收
	{
		/* Enable FIFO 0 overrun and message pending Interrupt */
		__HAL_CAN_ENABLE_IT (hcan, CAN_IT_FOV0 | CAN_IT_FMP0);
	}
	/* 准备中断接收 */
	//HAL_CAN_Receive_IT (hcan, CAN_FIFO0);
}
/**
  * 函数功能: 定时器比较输出中断回调函数
  * 输入参数: htim：定时器句柄指针
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_TIM_OC_DelayElapsedCallback (TIM_HandleTypeDef *htim)
{
	uint16_t count;

	count=__HAL_TIM_GET_COUNTER (&StepMotor_TIM);
	__HAL_TIM_SET_COMPARE (&StepMotor_TIM, TIM_CHANNEL_1, (uint16_t)(count + Toggle_Pulse));
	tim_Pulse_count++;
	StepMotor_Pulse_cnt = tim_Pulse_count / (motor_step_value * 2);
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	if (GPIO_Pin == INA20X_CMPOUT_Pin)
	{
//      Aim_Location = CaptureNumber;
//      Vel_Exp_Val = 0;
//      HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_1);
//      Drv8711_TORQUE_Set(0); // 设置力矩
//      memset(MAIN_TO_DRIVER_DATA,0,8);
//      over_current_error = 1;
//    __HAL_GPIO_EXTI_CLEAR_IT(INA20X_CMPOUT_Pin);
	}
}
