#include "motor.h"
#include "includes.h"
#include "tim.h"
#include <string.h>

uint16_t StepMotor_Pulse_cnt = 0;
uint32_t tim_Pulse_count= 0 ;

uint16_t Toggle_Pulse = 500;         // 比较输出周期，值越小输出频率越快
uint8_t  motor_reduction_ratio = 1; //电机减速比
uint16_t encoder_lines = 1000; // 编码器线数

float Dis_Exp_Val = 0;     // PID计算出来的期望值
float Vel_Exp_Val = 0;     // PID计算出来的期望值
float Dis_Target = 0;             // 目标位置所对应编码器脉冲值
float Vel_Target = 0;             // 每单位采样周期内的脉冲数(频率)
uint16_t SUM_Pulse = 0;           // 1秒内的总脉冲
int32_t MSF = 0;                  // 电机反馈速度
int32_t CaptureNumber = 0;          // 输入捕获数
int32_t Last_CaptureNumber = 0;// 上一次捕获值
uint8_t aTxBuffer[SENDBUFF_SIZE]; // 串口DMA发送缓冲区
uint8_t Motion_Dir = 0;           // 电机运动方向

uint8_t torque_enable_done = 0;
uint8_t torque_disable_done = 0;

extern EncoderType GetEncoder;
float Vel_Exp_Val_tmp = 5;


//步进电机辅助输出脉冲计数器
/* 私有类型定义 --------------------------------------------------------------*/
typedef struct
{
    float SetPoint;    // 目标值  单位:mm
    float LastError;     // 前一次误差
    float PrevError;     // 前两次误差
    long SumError;     // 累计误差
    double Proportion; // Kp系数
    double Integral;   // Ki系数
    double Derivative; // Kd系数
} PID;

/* 私有变量 ------------------------------------------------------------------*/
static PID sPID,vPID;
uint8_t Time_Flag = 0;              // 任务时间标记
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能：增量式PID速度环计算
  * 输入参数：NextPoint     由编码器得到的速度值
  *           TargetVal    目标值
  * 返 回 值：经过PID运算得到的增量值
  * 说    明：增量式 PID 速度环控制设计,计算得到的结果仍然是速度值
  */
float IncPIDCalc (int NextPoint, float TargetVal, PID* ptr )
{
    float iError = 0, iIncpid = 0;                       // 当前误差

    iError = TargetVal - NextPoint;                     // 增量计算
    if ((iError < 0.8) && (iError > -0.8))
    {
        iError = 0;    // |e| < 0.8,不做调整
    }
    iIncpid = (ptr->Proportion * iError)               // E[k]项
             - (ptr->Integral * ptr->LastError)    // E[k-1]项
             + (ptr->Derivative * ptr->PrevError); // E[k-2]项
    ptr->PrevError = ptr->LastError;                    // 存储误差，用于下次计算
    ptr->LastError = iError;
    return (iIncpid);                                 // 返回增量值
}

/**
  * 函数功能：PID参数初始化
  * 输入参数：无
  * 返 回 值：无
  * 说    明：无
*/
void PID_Init (void)
{
   // sPID.Proportion = 0.15;   //比例系数//0.15//0.35
    sPID.Proportion = 0.020;//0.015
    sPID.Integral   = 0.001;      //积分系数0.005
  //  sPID.Integral   = 0.035;      //积分系数
    sPID.Derivative = 0.001;      //微分系数0.005
    sPID.LastError  = 0;      //前一次的误差
    sPID.PrevError  = 0;      //前两次的误差
    sPID.SetPoint   = 50;     //目标值
    sPID.SumError   = 0;      //累计误差

  //  vPID.Proportion = 0.035;     //比例系数 // 0.035// 0.015  0.09
    vPID.Proportion = 0.045; //0.08
    vPID.Integral   = 0.002;     //积分系数 //0.005  0.05
    vPID.Derivative = 0.001;      //0.09微分系数// 0.08//0.5
    vPID.LastError  = 0;      //前一次的误差
    vPID.PrevError  = 0;      //前两次的误差
    vPID.SetPoint   = 7;      //目标值
    vPID.SumError   = 0;      //累计误差
}

void DevelopmentFramwork (void)
{
	if (driver_board_enable_flag == 1)
	{ 
		Encoder_Config();
		Encoder_Total();
		MotorSpeedLocatin_Set (Vel_Target, Dis_Target);
	}
	iwdg_motor_ctrl_flag = 1;
}

void MotorSpeedLocatin_Set (float speed, float lacation)
{
    float speed_step = 0.5;
//	float speed_step = 1;

    STEPMOTOR_TORQUE_Disable();
    if ((speed - Vel_Exp_Val_tmp) > speed_step)
	{
		Vel_Exp_Val_tmp = Vel_Exp_Val_tmp + speed_step;
    }
    else if ((speed - Vel_Exp_Val_tmp) < -speed_step)
	{
		Vel_Exp_Val_tmp = Vel_Exp_Val_tmp - speed_step;
    }
    else
    {
    	Vel_Exp_Val_tmp = speed;
    }
    CaptureNumber = Location_Cnt;
    MSF = GetEncoder.V3;
    MSF = abs (MSF);
    //对速度进行累计,得到1s内的脉冲数
    SUM_Pulse += MSF;
    //位置环PID计算,根据计算结果判断电机运动方向
    Dis_Exp_Val = IncPIDCalc (Location_Cnt, lacation, &sPID);
    Motion_Dir = Dis_Exp_Val < 0 ? CCW : CW;
    Dis_Exp_Val = abs (Dis_Exp_Val);
    if (Vel_Exp_Val_tmp <= 0)
	{
		STEPMOTOR_OUTPUT_DISABLE(); //速度为零直接停止
    }
    else
    {
        //位置环输出作为速度环的输入,需要限制位置环的输出不会超过速度环目标值
        if (Dis_Exp_Val >= Vel_Exp_Val_tmp)
        {
            Dis_Exp_Val = Vel_Exp_Val_tmp;
        }
        Vel_Exp_Val += IncPIDCalc (MSF, Dis_Exp_Val, &vPID);
        //当到达目标位置的时候,这时候已经电机非常慢了.为了减少超调,可以直接将速度环的输出清零
        if (Vel_Exp_Val <= 0.1)
        {
            Vel_Exp_Val = 0;
            Vel_Exp_Val_tmp = 5;
        }
        else if (Vel_Exp_Val > (float)motor_maxspeed)
		{
			Vel_Exp_Val = (float)motor_maxspeed;
        }
        /* 经过PID计算得到的结果是编码器的输出期望值的增量,
        需要转换为步进电机的控制量(频率值),这里乘上一个系数6400/2400
        */
        STEPMOTOR_Motion_Ctrl (Motion_Dir, Vel_Exp_Val * FEEDBACK_CONST); //乘上一个系数,6400/2400,将PID计算结果转换为步进电机的频率(速度)
//        if ( ( abs ( ( abs ( ( int ) Dis_Target ) - abs ( ( int ) Location_Cnt ) ) ) <20 ) )
//        {
////            if ( Vel_Exp_Val == 0 )
////            {
////                //STEPMOTOR_TORQUE_Enable();
////                 STEPMOTOR_OUTPUT_DISABLE();
////            }
////            else
////            {
////                Vel_Exp_Val = Vel_Exp_Val/2;
////            }
//        }
//        else
//        {
//            //STEPMOTOR_TORQUE_Disable();
//            STEPMOTOR_OUTPUT_ENABLE();
//
//        }
    }
}

void Get_Motor_Statues (void)
{
    /* 速度值计算: v= 1s内的总步数*编码器单步步进距离 */
    /* 当前位置 = 编码器捕获值*编码器单步步进距离 */
    // sprintf((char*)aTxBuffer,"\r\n捕获值:%d 当前位置:%.2fmm 速度:%.1f mm/s",CaptureNumber,(float)(CaptureNumber*MPP),(float)SUM_Pulse*MPP);
    // sprintf((char*)aTxBuffer+strlen((const char*)aTxBuffer),"\r\n1s内编码器计数值:%d",SUM_Pulse);
    // HAL_UART_Transmit_DMA(&husart_debug, aTxBuffer, strlen((const char*)aTxBuffer));
    //printf ("\r\n 捕获值:%d 当前位置:%.2fmm 速度:%.1f mm/s", CaptureNumber, (float)(CaptureNumber * MPP), (float)SUM_Pulse * MPP);
    //printf ("\r\n 输出脉冲个数: %d", StepMotor_Pulse_cnt);
    SUM_Pulse = 0;
}
/**
  * 函数功能: 步进电机运动控制
  * 输入参数: Dir:步进电机运动方向 0:反转 1正转
  *           Frequency:步进电机频率,0:停止
  * 返 回 值: void
  * 说    明: 无
  */
void STEPMOTOR_Motion_Ctrl (uint8_t Dir, float Frequency)
{
    uint16_t Step_Delay = 0;  //步进延时

    if (Frequency <= 0)
    {
         STEPMOTOR_OUTPUT_DISABLE();
        //STEPMOTOR_TORQUE_Enable();
    }
    else
    {
        if (Dir == CCW)
        {
            STEPMOTOR_DIR_REVERSAL();
        }
        else
        {
            STEPMOTOR_DIR_FORWARD();    //方向控制
        }
        /*
          步进电机速度由定时器输出脉冲频率(f)决定,
            f = c/F;c是计数器的计数值,F是定时器频率
          推导过程:(T是定时器输出脉冲周期)
            T=c*t => T=c/F => f = 1/T = F/c;
        */
        Step_Delay = (uint16_t)(FREQ_UINT / Frequency);
         STEPMOTOR_OUTPUT_ENABLE();
        // STEPMOTOR_TORQUE_Disable();
        Toggle_Pulse = Step_Delay >> 1;//算出来的结果是周期
    }
}

void STEPMOTOR_OUTPUT_DISABLE (void) //电机停止运转
{
    SPI_DRV8711_Write (CTRL_Register_ADDR, drv8711_ctrl_value & 0xfffe);
    motor_stop_flag = 1;
}

void STEPMOTOR_OUTPUT_ENABLE (void) //电机运转
{
    SPI_DRV8711_Write (CTRL_Register_ADDR, drv8711_ctrl_value | 0x0001);
    motor_stop_flag = 0;
}

void STEPMOTOR_TORQUE_Enable (void) //使能力矩
{
    if (torque_enable_done == 0)
    {
        HAL_TIM_OC_Stop_IT (&htim2, TIM_CHANNEL_1);
        Drv8711_TORQUE_Set (0); // 设置力矩
        STEPMOTOR_OUTPUT_ENABLE();
        torque_enable_done = 1;
        torque_disable_done = 0;
        //motor_stop_flag = 1;
    }
}
void STEPMOTOR_TORQUE_Disable (void) //失能力矩
{
    if (torque_disable_done == 0)
    {
        HAL_TIM_OC_Start_IT (&htim2, TIM_CHANNEL_1);
        Drv8711_TORQUE_Set (drv_torque_value); // 设置力矩
        STEPMOTOR_OUTPUT_ENABLE();
        torque_disable_done = 1;
        torque_enable_done = 0;
        //motor_stop_flag = 0;
    }
}

void STEPMOTOR_DIR_REVERSAL (void)
{
    if (motor_move_direction_flag == 0)
    {
        HAL_GPIO_WritePin (DRV8711_DIR_GPIO_Port, DRV8711_DIR_Pin, GPIO_PIN_SET);
    }
    else if (motor_move_direction_flag == 1)
    {
        HAL_GPIO_WritePin (DRV8711_DIR_GPIO_Port, DRV8711_DIR_Pin, GPIO_PIN_RESET);
    }
}

void STEPMOTOR_DIR_FORWARD (void)
{
    if (motor_move_direction_flag == 0)
    {
        HAL_GPIO_WritePin (DRV8711_DIR_GPIO_Port, DRV8711_DIR_Pin, GPIO_PIN_RESET);
    }
    else if (motor_move_direction_flag == 1)
    {
        HAL_GPIO_WritePin (DRV8711_DIR_GPIO_Port, DRV8711_DIR_Pin, GPIO_PIN_SET);
    }
}
