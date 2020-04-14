#include "includes.h"
#include "control.h"
#include <math.h>


uint8_t bsp_init_flag = 0;
uint8_t soft_reset_flag = 0;//软急停
uint8_t driver_board_enable_flag = 0;//驱动板使能标志
uint8_t driver_error_flag = 0; //错误标志
uint8_t driver_over_current_flag = 0; //过流标志
uint8_t debug_mode_flag = 0; //调试模式标志
uint8_t firmware_Update_flag = 0;//固件更新标志
uint8_t car_main_board_enable_flag = 1;//主控板使能
uint8_t can_Receive_Right_flag = 1;//数据正确标志
uint8_t motor_stop_flag = 1;//电机停止标志位
extern EncoderType GetEncoder;

uint16_t driver_can_stdid = 0xa2; //定义驱动板ID
uint16_t mian_can_stid = 0xa0; // 定义主控板ID

CanTxMsgTypeDef TxMessage ;
CanRxMsgTypeDef RxMessage ;

uint8_t DRIVER_TO_MAIN_DATA[DRIVER_TO_MAIN_DATA_LEN] = {0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 驱动板到主控板的数据
uint8_t MAIN_TO_DRIVER_DATA[MAIN_TO_DRIVER_DATA_LEN] = {0x0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 主控板到驱动板的数据 ,
uint8_t  can_rbuff[MAIN_TO_DRIVER_DATA_LEN];//接收缓存

uint8_t UARTI_DEBUG_DATA[UARTI_DEBUG_DATA_LEN] = {UARTI_DEBUG_DATA_FRAME_START, UARTI_DEBUG_DATA_FRAME_SECOND};
uint8_t uart1_rxbuff[UART1_RX_BUFF_LEN];
uint8_t uart1_rdata[UART1_RX_BUFF_LEN];
uint8_t uart1_tdata[UART1_TX_BUFF_LEN];
uint16_t uart1_rxdata_cnt = 0;
uint8_t  uart1_debug_data_len = 10;//调试串口
uint8_t uart1_receive_byte = 0; //串口接收的字符
uint8_t uart1_Receive_Right_flag = 0;//数据接收正确标志
uint8_t main_request_data_flag = 0;//主控板请求数据
uint8_t led_display_cnt_flag = 0;//led闪烁任务
uint8_t can_send_data_cnt_flag = 0;//CAN发送数据标志
uint8_t location_write_cnt_flag = 0;//写电机位置标志

float screw_distance = 0; //丝杆模式下要移动的距离
int32_t Aim_Location = 0;//位置信息
uint8_t motor_speed = 0;//电机速度
uint8_t percircle_distance = 0;//丝杆模式下转动每圈所运动距离

uint8_t motor_move_direction_flag = 0;//运动正方向标志位
uint8_t encoder_cnt_direction_flag = 0;//编码器计数方向
uint8_t motor_limit_flag = 0;
uint8_t motor_maxspeed = 120;

uint32_t ADC_Get_Info[3];
float ADC_Current_Value;
float ADC_Voltage_Value;
float ADC_Temperature_Value;

uint8_t iwdg_system_flag = 1; //系统控制函数看门狗标志位
uint8_t iwdg_motor_ctrl_flag = 1;

s32 Location_Cnt_tmp = 0;
uint16_t location_addr = 0;

s32 eep_motor_location = 0;
uint8_t  eep_driver_id = 0; //驱动器ID
uint8_t  eep_motor_speed = 0; //电机速度
uint8_t  eep_motor_torque = 0;//电机电流 0-0xff
uint8_t  eep_motor_step_value = 0;//电机细分
uint16_t eep_location_addr = 0;

uint8_t motor_limit_cnt_flag = 0;
/**********zyg 2019.8.28*******************/
int32_t gCurrent_pos = 0;	//当前目标位置
int32_t gNew_pos = 0;		//新的目标位置


extern uint8_t motor_limit_cnt;	//zyg

/*****************************/

//看门狗更新函数
void Iwdg_Updata (void)
{
	if ((iwdg_system_flag == 1) && (iwdg_motor_ctrl_flag == 1) && (driver_over_current_flag == 0))
	{
		HAL_IWDG_Refresh (&hiwdg); // 200ms喂狗一次
		iwdg_system_flag = 0;
		iwdg_motor_ctrl_flag = 0;
	}
}
//系统控制函数
void System_Control (void)
{
	CAN_Receive_Data_Analysis();//数据解析
	Send_Driver_To_Mian_Data();//发送数据到主控板
	Eep_Data_Update();//任何时候都可以更新
	if (bsp_init_flag == 1) //等待底层初始化完成
	{
		if (driver_board_enable_flag == 1)
		{
			if ((can_Receive_Right_flag == 1) && (driver_over_current_flag == 0) && (soft_reset_flag ==0)) //驱动板使能,接收数据正确
			{
				Check_Location();//PID赋值给期望值，位置给定
			}
			else
			{
				Dis_Target = Location_Cnt; //停止
			}
		}
		else
		{

		}
	}
	iwdg_system_flag = 1;
}

void Driver_Board_Parameter_Init (void)
{
	//float set_motor_current = 0;
	//At24c512_WriteByte ( 0x00,0 ); //0~7
	//At24c512_WriteByte ( 0x01,0 ); //0~7

	eep_driver_id =  At24c512_ReadByte ( 0xF0 ); //80地址为板卡的ID
	eep_motor_torque = At24c512_ReadByte ( 0xF1 ); //81地址为板卡的电流
	eep_motor_step_value = At24c512_ReadByte ( 0xF2 ); //82地址为板卡细分设置

	eep_location_addr = ( At24c512_ReadByte (0x00) << 8) + At24c512_ReadByte (0x01);
	if (eep_location_addr >= 2) //防止第一次写的时候数据地址为0xff
	{
		At24c512_WriteByte (0x00, 0); //0~7
		At24c512_WriteByte (0x01, 0); //0~7
		eep_location_addr = ( At24c512_ReadByte ( 0x00 ) <<8 ) +At24c512_ReadByte ( 0x01 );
	}
	eep_motor_location = (At24c512_ReadByte (eep_location_addr + 2) << 24)+ (At24c512_ReadByte (eep_location_addr + 3) << 16)
	                     + (At24c512_ReadByte (eep_location_addr + 4) << 8)+ (At24c512_ReadByte (eep_location_addr + 5)); //驱动器当前位置
	//printf ("\r\n 驱动器当前位置:%d ", eep_motor_location);
	drv_torque_value = eep_motor_torque;//设置电流
	motor_step_value = eep_motor_step_value;//读取细分
	driver_can_stdid = eep_driver_id;//读取ID
	location_addr = eep_location_addr;
	/*
	if((driver_can_stdid == 0xa3) || (driver_can_stdid == 0xa2))
	{
		motor_limit_cnt = 150;
	}
	*/
	//位置赋值
	Location_Cnt = eep_motor_location;
	Dis_Target = eep_motor_location;
	gCurrent_pos = eep_motor_location;		//zyg 2019.8.28
	CaptureNumber = eep_motor_location;
	Location_Cnt_tmp = eep_motor_location;
	//set_motor_current = Drv8711_Ifs_Set (drv_torque_value);
	Drv8711_Ifs_Set (drv_torque_value);
        
	//printf ("\r\n 读取板卡设置参数");
	//printf ("\r\n 驱动器ID:%x", driver_can_stdid);
	//printf ("\r\n 驱动器当前位置:%d ", Location_Cnt);
	//printf ("\r\n 当前数据地址：%d", location_addr);
	//printf ("\r\n 驱动器电流:%2.3fA", set_motor_current);
	//printf ("\r\n 驱动器细分:%d", motor_step_value);
}
//用于修改驱动器配置参数
void Eep_Data_Update ( void )
{
	uint8_t ID;
	uint8_t torque;
	float current;
	uint8_t step;

	if (uart1_Receive_Right_flag == 1) //接收到正确数据
	{
		eep_driver_id = UARTI_DEBUG_DATA[3];
		eep_motor_torque = UARTI_DEBUG_DATA[4];
		eep_motor_step_value = UARTI_DEBUG_DATA[5];
		At24c512_WriteByte (0xF0, eep_driver_id);
		At24c512_WriteByte (0xF1, eep_motor_torque);
		At24c512_WriteByte (0xF2, eep_motor_step_value);

		ID = At24c512_ReadByte (0xF0);
		At24c512_ReadByte (0xF0);
		torque = At24c512_ReadByte (0xF1);
		step = At24c512_ReadByte (0xF2);
		At24c512_ReadByte (0xF2);
		current = Drv8711_Ifs_Set (torque);
		Drv8711_Ifs_Set (torque);
		printf ("\r\n ID:0x%x current:%2.3fA step:%d ", ID, current, step);
		printf ("\r\n parameters updada succees!......");
		uart1_Receive_Right_flag = 0;//置0
	}
}
//记录电机的位置
void Motor_Location_Write (void)
{
	s32 location_tmp = 0;

	if (location_write_cnt_flag == 1)
	{
		if (eep_motor_location != Location_Cnt)
		{
			Location_Cnt_tmp = (uint32_t)Location_Cnt;
			At24c512_WriteByte ((location_addr + 5), Location_Cnt_tmp & 0x000000ff); //0~7
			At24c512_WriteByte ((location_addr + 4), (Location_Cnt_tmp >> 8) & 0x0000ff); //8~15
			At24c512_WriteByte ((location_addr + 3), (Location_Cnt_tmp >> 16) & 0x00ff); //16~23
			At24c512_WriteByte ((location_addr + 2), Location_Cnt_tmp >> 24); //24~31
			// Delay_ms(10);
			location_tmp = (At24c512_ReadByte (location_addr + 2) << 24) + (At24c512_ReadByte (location_addr + 3) << 16)
			               + (At24c512_ReadByte (location_addr + 4) << 8) + (At24c512_ReadByte (location_addr + 5)); //驱动器当前位置
			if (location_tmp != Location_Cnt_tmp)
			{
				location_addr++;
				if (location_addr > 0xE0)
				{
					location_addr = 0xE0;
				}
				At24c512_WriteByte (0x00, location_addr >> 8); //0~7
				At24c512_WriteByte (0x01, location_addr && 0x00ff); //0~7
			}
			//printf ("\r\n 当前地址：%d", location_addr);
			//printf ("\r\n 当前位置：%d", Location_Cnt_tmp);
		}
		eep_motor_location = Location_Cnt_tmp;
		location_write_cnt_flag = 0;
	}
}

void CAN_Receive_Data_Analysis (void) //CAN接收的数据解析,放在CAN接受中断里比较合适
{
	if (can_Receive_Right_flag == 1) //如果数据正确接收才能解析
	{
		driver_board_enable_flag = ReadByte_Bit (MAIN_TO_DRIVER_DATA[0], 0); //电机使能标志
		main_request_data_flag = ReadByte_Bit (MAIN_TO_DRIVER_DATA[0], 7);   //请求数据
		soft_reset_flag = ReadByte_Bit (MAIN_TO_DRIVER_DATA[0], 1);          //软件急停
		motor_move_direction_flag = ReadByte_Bit (MAIN_TO_DRIVER_DATA[0], 2); //方向
		encoder_cnt_direction_flag = ReadByte_Bit (MAIN_TO_DRIVER_DATA[0], 3); //编码器计数方向

		Aim_Location = (s32)((MAIN_TO_DRIVER_DATA[3] << 24) + (MAIN_TO_DRIVER_DATA[4] << 16) + (MAIN_TO_DRIVER_DATA[5] << 8) + MAIN_TO_DRIVER_DATA[6]); //关节电机
		if (Aim_Location > 12000000)
		{
			Aim_Location = 12000000;
		}
		if (Aim_Location < -12000000)
		{
			Aim_Location = -12000000;
		}
		motor_speed = (MAIN_TO_DRIVER_DATA[7]); //速度赋值
		if ((motor_limit_flag == 1) || (motor_limit_flag == 2))
		{
			motor_speed = 25;
			if(motor_limit_flag == 1)
			{
				if(Aim_Location <= 10)
				{
					Aim_Location = 0;	//在零位时不执行负方向
				}
				/*
				else if(Aim_Location <= 800)
				{
					Aim_Location = 800;
				}
				*/
			}
        }
		else if (motor_speed > motor_maxspeed)
		{
			motor_speed = motor_maxspeed;
		}
	}
}
//获取上传的数据
void CAN_SendData_Acquisition (void)
{
	s32 nLocationCnt = 0;
	nLocationCnt = Location_Cnt;

	DRIVER_TO_MAIN_DATA[0] = SetByte_Bit (DRIVER_TO_MAIN_DATA[0], 0, Start_Position_Read());
	DRIVER_TO_MAIN_DATA[0] = SetByte_Bit (DRIVER_TO_MAIN_DATA[0], 1, End_Position_Read());
	DRIVER_TO_MAIN_DATA[2] = drv8711_status_value;//驱动器状态寄存器值
	DRIVER_TO_MAIN_DATA[3] = nLocationCnt >> 24;
	DRIVER_TO_MAIN_DATA[4] = (nLocationCnt >> 16) & 0x00ff;
	DRIVER_TO_MAIN_DATA[5] = (nLocationCnt >> 8) & 0x0000ff;
	DRIVER_TO_MAIN_DATA[6] = nLocationCnt  &0x000000ff;
	DRIVER_TO_MAIN_DATA[7] = MSF; //反馈速度
}

//限位开关扫描
void Limit_Switch_Scanning (void)
{
	if (Start_Position_Read() == 1) //start position
	{
		motor_limit_flag = 1;
		if ((Dis_Target <= Location_Cnt) && (motor_limit_cnt_flag == 1))
		{
			MAIN_TO_DRIVER_DATA[3] = 0;
			MAIN_TO_DRIVER_DATA[4] = 0;
			MAIN_TO_DRIVER_DATA[5] = 0;
			MAIN_TO_DRIVER_DATA[6] = 0;
			Dis_Target = 0;
			Location_Cnt = 0;
			gCurrent_pos = 0;	//zyg
		}
	}
	else if (End_Position_Read() == 1)//end position
	{
		motor_limit_flag = 2;
		if ((Dis_Target >= Location_Cnt) && (motor_limit_cnt_flag == 1))
		{
			MAIN_TO_DRIVER_DATA[3] = Location_Cnt >> 24;
			MAIN_TO_DRIVER_DATA[4] = (Location_Cnt >> 16) & 0x00ff;
			MAIN_TO_DRIVER_DATA[5] = (Location_Cnt >> 8) & 0x0000ff;
			MAIN_TO_DRIVER_DATA[6] = Location_Cnt & 0x000000ff;
			Dis_Target = Location_Cnt;
			gCurrent_pos = Location_Cnt;	//zyg
		}
	}
	else
	{
		motor_limit_flag = 0;
        motor_limit_cnt_flag = 0;
	}
}

void Check_Location (void)
{
	int32_t i = 0;

	Vel_Target = (float)motor_speed;//速度给定
	//Vel_Target *= 2;
	//Aim_Location = (s32)((MAIN_TO_DRIVER_DATA[3] << 24) +(MAIN_TO_DRIVER_DATA[4] << 16) + (MAIN_TO_DRIVER_DATA[5] << 8) + MAIN_TO_DRIVER_DATA[6]); 
	gNew_pos = Aim_Location;
	if(gCurrent_pos != gNew_pos)
	{
		if(gNew_pos > gCurrent_pos)	//正方向
                
		{
			if(gCurrent_pos < CaptureNumber)
			{
				i = CaptureNumber - gCurrent_pos;
				if(i <= 20)
				{
					gCurrent_pos = gNew_pos;
				}
			}
			else
			{
				gCurrent_pos = gNew_pos;
			}
		}
                
		else
		{
			i = gCurrent_pos - CaptureNumber;
			if(i <= 20)
			{
				gCurrent_pos = gNew_pos;
			}
		}
		
	/*
		if(abs(gCurrent_pos) >= abs(CaptureNumber)) //正方向
		{
			if(abs(gNew_pos) >=  abs(gCurrent_pos))
			{
				gCurrent_pos = gNew_pos;
			}
			else
			{
	           	a = abs(gCurrent_pos);
	            b = abs(CaptureNumber);
				i = abs(a - b);
				if(i <= 20)
				{
					gCurrent_pos = gNew_pos;
				}
			}
		}
		else	//负方向
		{
			if(abs(gNew_pos) <=  abs(gCurrent_pos))
			{
				gCurrent_pos = gNew_pos;
			}
			else
			{	
				a = abs(gCurrent_pos);
	                        b = abs(CaptureNumber);
				i = abs(a - b);
				if(i >= 20)
				{
					gCurrent_pos = gNew_pos;
				}
			}
		}
		*/
	}

	if((CaptureNumber > (gCurrent_pos-10)) && (CaptureNumber < (gCurrent_pos + 10)))
	{
	//	Dis_Target = Aim_Location; //赋值
	//	Dis_Target = gCurrent_pos;
		Dis_Target = CaptureNumber;
	}
	else
	{
	//	Dis_Target = CaptureNumber;
        Dis_Target = gCurrent_pos;
	}
}

//CAN发送数据
void Send_Driver_To_Mian_Data (void)
{
	if ((main_request_data_flag == 1)) //数据请求了在发送&& ( can_send_data_cnt_flag ==1 ) can_send_data_cnt_flag = 0;
	{
		CAN_SendData_Acquisition();//加载数据
		CAN_SetMsg();//填充数据
		__HAL_UNLOCK (&hcan);
		// HAL_CAN_Transmit(&hcan,0XFFFF);
		HAL_CAN_Transmit_IT (&hcan);
		MAIN_TO_DRIVER_DATA[0] = SetByte_Bit (MAIN_TO_DRIVER_DATA[0], 7, 0);
		main_request_data_flag = 0;
	}
}

// CAN筛选器初始化
void Driver_Can_Init (void)
{
	hcan.pRxMsg = &RxMessage;
	hcan.pTxMsg = &TxMessage;
	if (HAL_CAN_Init (&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	CAN_Filter_Config_Scale16_IdList();
	RxMes_Init();
}

/**
  * @brief  初始化 Rx Message数据结构体
  * @param  RxMessage: 指向要初始化的数据结构体
  * @retval None
  */
void RxMes_Init (void)
{
	uint8_t ubCounter = 0;

	/*把接收结构体清零*/
	RxMessage.StdId = 0x00;
	RxMessage.ExtId = 0x00;
	RxMessage.IDE = CAN_ID_STD;
	RxMessage.DLC = 0;
	RxMessage.FMI = 0;
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
	{
		RxMessage.Data[ubCounter] = 0x00;
	}
}

/*
 * 函数名：CAN_SetMsg
 * 描述  ：CAN通信报文内容设置,设置一个数据内容为0-7的数据包
 * 输入  ：发送报文结构体
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_SetMsg (void)
{
	uint8_t ubCounter = 0;
	TxMessage.StdId = driver_can_stdid;
	//TxMessage.ExtId = 0x1314;                     //使用的扩展ID
	TxMessage.IDE = CAN_ID_STD;                  //扩展模式
	TxMessage.RTR = CAN_RTR_DATA;                 //发送的是数据
	TxMessage.DLC = 8;                            //数据长度为8字节

	/*设置要发送的数据0-7*/
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
	{
		TxMessage.Data[ubCounter] = DRIVER_TO_MAIN_DATA[ubCounter];
	}
}

//CAN筛选器配置
void CAN_Filter_Config_Scale16_IdList (void)
{
	CAN_FilterConfTypeDef  sFilterConfig;
	uint32_t StdId1 = driver_can_stdid;
	uint32_t StdId2 = 0x124;
	uint32_t StdId3 = 0x125;
	uint32_t StdId4 = 0x126;

	sFilterConfig.FilterNumber = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = StdId1 << 5;
	sFilterConfig.FilterIdLow = StdId2 << 5;
	sFilterConfig.FilterMaskIdHigh = StdId3 << 5;
	sFilterConfig.FilterMaskIdLow = StdId4 << 5;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;
	if (HAL_CAN_ConfigFilter (&hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void Ina201_ADC_Get (void)
{
	HAL_ADCEx_Calibration_Start (&hadc1);
	HAL_ADC_Start_DMA (&hadc1, ADC_Get_Info, 3);
	ADC_Voltage_Value = (float)(ADC_Get_Info[0] & 0xfff) * 3.3 / 4096;
	ADC_Current_Value = (float)(ADC_Get_Info[1] & 0xfff) * 3.3 / 4096;
	ADC_Temperature_Value = (float)(ADC_Get_Info[2] & 0xfff) * 3.3 / 4096;
	ADC_Temperature_Value = (ADC_Temperature_Value) / 0.0025;
	//printf("\r\n ADC_Voltage_Value:%f",ADC_Voltage_Value);
	//printf("\r\n ADC_Current_Value:%f",ADC_Current_Value);
	//printf("\r\n ADC_Temperature_Value:%f",ADC_Temperature_Value);
}
