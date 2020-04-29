#include "motor.h"
#include "includes.h"
#include "tim.h"
#include <string.h>
#include <math.h>

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
//float Vel_Exp_Val_tmp = 1;

uint8_t g_iNominalStartSpeed = 0; //期望的初始速度,编码器计数/s
uint8_t g_iNominalEndSpeed = 0;	//期望的停止速度,编码器计数/s
float g_fSpeedExpect = 0.0;
float g_fMinimumValue = 0.5;
float g_fAccTime = 0.4;
float g_fDecTime = 0.4;
#define ACC_TIME_DIVISION	20

CurveBlock structCurveBlock;

//转速,距离
const uint16_t arrSpeedTable[ACC_TIME_DIVISION][2] = {
{49,0},
{199,3},
{449,8},
{799,15},
{1250,25},
{1800,36},
{2450,49},
{3199,63},
{4049,80},
{4999,99},
{5000,100},
{5950,119},
{6800,136},
{7550,151},
{8200,164},
{8750,175},
{9200,184},
{9550,191},
{9800,196},
{9950,199}
};

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
    sPID.Proportion = 0.1;//0.015
    sPID.Integral   = 0.005;      //积分系数0.005
  //  sPID.Integral   = 0.035;      //积分系数
    sPID.Derivative = 0.005;      //微分系数0.005
    sPID.LastError  = 0;      //前一次的误差
    sPID.PrevError  = 0;      //前两次的误差
    sPID.SetPoint   = 50;     //目标值
    sPID.SumError   = 0;      //累计误差

  //  vPID.Proportion = 0.035;     //比例系数 // 0.035// 0.015  0.09
    vPID.Proportion = 0.12; //0.08
    //vPID.Integral   = 0.002;     //积分系数 //0.005  0.05
    vPID.Integral   = 0.01;
   // vPID.Derivative = 0.001;      //0.09微分系数// 0.08//0.5
    vPID.Derivative = 0.05;
    vPID.LastError  = 0;      //前一次的误差
    vPID.PrevError  = 0;      //前两次的误差
    vPID.SetPoint   = 7;      //目标值
    vPID.SumError   = 0;      //累计误差
}

void DevelopmentFramwork (void)
{
	if (driver_board_enable_flag == 1)
	{ 
//		HAL_TIM_OC_Stop_IT (&htim2, TIM_CHANNEL_1);
//		Encoder_Config();
//		Encoder_Total();
		MotorSpeedLocatin_Set (Vel_Target, Dis_Target);
	}
	iwdg_motor_ctrl_flag = 1;
}

void CurveBlockPrepare(CurveParams *structParams)
{
		structParams->bRecalculated = false;
		structParams->bStop = false;
		structParams->bBusy = false;
		structParams->bPlateauAll = false;
		structParams->dJerk = 0;
		structParams->iAccStepComplete = 0;
		structParams->iAccStepIndex = 0;
		structParams->iDecStepIndex = 0;
		structParams->bAccAddStep = true;
		structParams->bDecAddStep = true;
		structParams->iLastStepCount = 0;
		//printf("\r\nreset");
}

void CurveBlockStop(CurveParams *structParams)
{
		structParams->bStop = true;
//		structParams->bBusy = false;
}

void StartMove(uint8_t iDirection)
{
		if (iDirection == CCW)
		{
				STEPMOTOR_DIR_REVERSAL();
		}
		else
		{
				STEPMOTOR_DIR_FORWARD();    //方向控制
		}

		STEPMOTOR_OUTPUT_ENABLE();		
}

void CalculateCurve(CurveParams *structParams, uint16_t iSpeed, int32_t iTargetLocation)
{
		//加减速缩放系数
		static float fZoomFactorAcc = 1; 
		static float fZoomFactorDec = 1;
		//曲线与标准曲线的比
		static float fCurveMultipleAcc = 1;
		static float fCurveMultipleDec = 1;
	
		static float fTimeTickAcc = 0;
		static float fTimeTickDec = 0;
		
		//参考梯形的加速度
		static float fAccAvg = 0;
		static float fDecAvg = 0;
		int i = 0;
		//抛物线查表法
		if(structParams->bStop)
		{
				return;
		}
		if(structParams->bBusy)
		{
				return;
		}
		if(!structParams->bRecalculated)
		{
				structParams->bBusy = true;
				structParams->bStop = false;
				structParams->iEncoderStartLocation = Location_Cnt;
				structParams->iEncoderTargetLocation = iTargetLocation;
				fTimeTickAcc = g_fAccTime / ACC_TIME_DIVISION;
				fTimeTickDec = g_fDecTime / ACC_TIME_DIVISION;
				//步进电机初始速度以某个速度开始
				structParams->iStartSpeed = g_iNominalStartSpeed;
				//步进电机收尾速度以某个速度结束
				structParams->iEndSpeed = g_iNominalEndSpeed;
				
				//给定的速度是10ms的脉冲数，转换成1s的脉冲数
				iSpeed *= 100;
				if(iSpeed < min(structParams->iStartSpeed, structParams->iEndSpeed))
				{
						structParams->iMaxSpeed = min(structParams->iStartSpeed, structParams->iEndSpeed);
						structParams->bPlateauAll = true;
				}
				else if(iSpeed > min(structParams->iStartSpeed, structParams->iEndSpeed) && iSpeed < max(structParams->iStartSpeed, structParams->iEndSpeed))
				{
						structParams->iMaxSpeed = max(structParams->iStartSpeed, structParams->iEndSpeed);
						structParams->bPlateauAll = true;
				}
				//给定速度大于起始速度且大于收尾速度时计算S曲线参数
				else if(iSpeed > structParams->iStartSpeed && iSpeed > structParams->iEndSpeed)
				{								
						//先以梯形为参考,一半加速时间时,Vm为给定速度一半且Vm - V0 = a*Tm, a = (Vm - V0) / Tm
						fAccAvg = (float)(iSpeed - structParams->iStartSpeed) / g_fAccTime;
						fDecAvg = (float)(structParams->iEndSpeed - iSpeed) / g_fDecTime;
						//计算加速段距离	(Vt ^ 2 - V0 ^ 2) / 2a = S, V0 = 0;
						structParams->fAccelerationDistance = (float)( pow(iSpeed, 2) - pow(structParams->iStartSpeed, 2) ) / (2 * fAccAvg);
						
						//计算减速段距离
						structParams->fDecelerationDistance = (float)( pow(structParams->iEndSpeed, 2) - pow(iSpeed, 2)) / (2 * fDecAvg);
						
						structParams->iMaxSpeed = iSpeed;
					
						structParams->iDirection = ((structParams->iEncoderTargetLocation > structParams->iEncoderStartLocation) ? 0 : 1);
					
						structParams->fMoveDistance = (float)abs(structParams->iEncoderTargetLocation - structParams->iEncoderStartLocation);
						
						//计算匀速段距离
						structParams->fPlateauDistance = structParams->fMoveDistance - structParams->fAccelerationDistance - structParams->fDecelerationDistance;
						structParams->iEncoderPlatDist = (uint32_t)structParams->fPlateauDistance;
						//计算匀速段小于0			
						printf("\r\nsl:%d", structParams->iEncoderStartLocation);
						printf("\r\ntl:%d", structParams->iEncoderTargetLocation);
						printf("\r\nfad:%f", structParams->fAccelerationDistance);
						printf("\r\nfdd:%f", structParams->fDecelerationDistance);
						if( structParams->fPlateauDistance < 0)
						{
								//梯形退化为三角形
								fZoomFactorAcc = (float)abs(structParams->iEncoderTargetLocation - structParams->iEncoderStartLocation) / 
															( structParams->fAccelerationDistance + structParams->fDecelerationDistance );
								
								printf("\r\nfz:%f", fZoomFactorAcc);
								structParams->fAccelerationDistance = structParams->fAccelerationDistance * fZoomFactorAcc;
								structParams->fDecelerationDistance = structParams->fDecelerationDistance * fZoomFactorAcc;
								//重新计算所能达到的速度 V0 * t + a' * t ^ 2 / 2 = AccDist
								//a' = 2 * (AccDist - V0 * t) / t ^ 2
								//Vt = V0 + a' * t = V0 + 2 * (AccDist - V0 * t) / t
								structParams->iMaxSpeed = structParams->iStartSpeed + 2 * (structParams->fAccelerationDistance - structParams->iStartSpeed * g_fAccTime) / g_fAccTime;
								if(structParams->iMaxSpeed < min(structParams->iStartSpeed, structParams->iEndSpeed))
								{
										structParams->iMaxSpeed = min(structParams->iStartSpeed, structParams->iEndSpeed);
										structParams->bPlateauAll = true;
								}
								else
								{
										structParams->fPlateauDistance = 0;
										structParams->iEncoderPlatDist = 0;
								}
						}
						
						structParams->iEncoderAccDist = (uint32_t)structParams->fAccelerationDistance;
						structParams->iEncoderDecDist = (uint32_t)structParams->fDecelerationDistance;
						
						if(!structParams->bPlateauAll)
						{								
								fCurveMultipleAcc = (float)(structParams->iMaxSpeed - structParams->iStartSpeed) / 10000;
								for(i = 0; i < ACC_TIME_DIVISION; i++)
								{
										structParams->arrAccDivisionTable[i][0] = (uint16_t)((float)2000000 / ((fZoomFactorAcc * (float)arrSpeedTable[i][0] * fCurveMultipleAcc + structParams->iStartSpeed) * 2));
										structParams->arrAccDivisionTable[i][1] = (uint16_t)((float)arrSpeedTable[i][0] * fTimeTickAcc);
										structParams->arrAccDivisionTable[i][0] = (uint16_t) ((float)structParams->arrAccDivisionTable[i][0] / FEEDBACK_CONST);
										printf("\r\n%d,%d", structParams->arrAccDivisionTable[i][0], structParams->arrAccDivisionTable[i][1]);
								}
								
								fCurveMultipleDec = (float)(structParams->iMaxSpeed - structParams->iEndSpeed) / 10000;
								printf("\r\n---");
								
								for(i = 0; i < ACC_TIME_DIVISION; i++)
								{
										structParams->arrDecDivisionTable[ACC_TIME_DIVISION - i - 1][0] = (uint16_t)((float)2000000 / ((fZoomFactorDec * (float)arrSpeedTable[i][0] * fCurveMultipleDec + structParams->iEndSpeed) * 2));
										structParams->arrDecDivisionTable[ACC_TIME_DIVISION - i - 1][1] = (uint16_t)((float)arrSpeedTable[i][0] * fTimeTickDec);
										structParams->arrDecDivisionTable[ACC_TIME_DIVISION - i - 1][0] = (uint16_t) ((float)structParams->arrDecDivisionTable[ACC_TIME_DIVISION - i - 1][0] / FEEDBACK_CONST);
										printf("\r\n%d,%d", structParams->arrDecDivisionTable[ACC_TIME_DIVISION - i - 1][0], structParams->arrDecDivisionTable[ACC_TIME_DIVISION - i - 1][1]);
								}
//								printf("\r\nspd:%f", fSpeed);
								printf("\r\nps:%d", structParams->iEncoderPlatDist);
								printf("\r\nas:%d", structParams->iEncoderAccDist);
								printf("\r\nds:%d", structParams->iEncoderDecDist);
								printf("\r\nsl:%d", structParams->iEncoderStartLocation);
								printf("\r\ntl:%d", structParams->iEncoderTargetLocation);
								printf("\r\ndir:%d", structParams->iDirection);
								printf("\r\nspd:%d", structParams->iMaxSpeed);
//								printf("\r\nJ:%f", structParams->dJerk);															
//								printf("\r\ndist:%d", structParams->iEncoderAccInflectDist);				
//								printf("\r\nfc:%f", FEEDBACK_CONST);
								//至此S曲线段参数已全部确定
								
//								//根据抛物线公式 V = aS * t^2 + V0 计算		
//								//加速段S曲线形状参数计算，为运算方便去掉起始速度的叠加
//								structParams->fCurveSpeedAcc = structParams->fMaxSpeed - structParams->dStartSpeed;	
//								
//								//计算中间速度, 即拐点速度Vm = (Vt - V0) / 2
//								structParams->dInflectionSpeed = structParams->fCurveSpeedAcc / 2;
//							
//								//急动度J, 加速度aS, aS = J*t, 加速度积分之后 Vm = J * Tm^2 / 2
//								//Vm = V / 2, Tm = T / 2, J = 2 * Vm / Tm = 2 * V / T
//								//计算达到中点速度时的距离, 对速度积分, s = J*t^3 / 6 = J * Tm^3 /6
//								structParams->dJerk = 2 * structParams->fCurveSpeedAcc / g_fAccTime;
//								structParams->iEncoderMoveDist = structParams->fMoveDistance;
//								structParams->iEncoderAccDist = structParams->fAccelerationDistance;
//								//加速段在加速度最大时走的距离
//								structParams->iEncoderAccInflectDist = structParams->dJerk * pow(g_fAccTime / 2, 3) / 6;
//								
//								structParams->iEncoderDecDist = structParams->fDecelerationDistance;
//								structParams->iEncoderDecInflectDist = structParams->iEncoderMoveDist - structParams->iEncoderAccInflectDist;																
								//以上计算的是以编码器计数为准
								//计算每次电机脉冲的急动度, 每一步都要比上一步多出的每秒脉冲数, FEEDBACK_CONST = 电机/编码器
//								structParams->dJerkPerPulse = structParams->dJerk / structParams->iEncoderAccInflectDist * FEEDBACK_CONST;
//								
//								dOC_Count = 2000000 / ((structParams->fCurveSpeedAcc * FEEDBACK_CONST) / 2 * 2);
//								if(dOC_CountStart > 65000)
//								{
//										structParams->iOC_Value = 65000;
//								}
//								else
//								{
//										structParams->iOC_Value = dOC_CountStart;
//								}
								
						}
				}
				else
				{
						structParams->iMaxSpeed = min(structParams->iStartSpeed, structParams->iEndSpeed);
						structParams->bPlateauAll = true;
				}
				
				structParams->bRecalculated = true;
				
				structParams->bAccAddStep = true;
				structParams->bDecAddStep = true;
				structParams->iDecStepComplete = structParams->iEncoderAccDist + structParams->iEncoderPlatDist;
		}
		
		StartMove(structParams->iDirection);	
		//End
		
//	不采用sigmoid曲线推算法	
//	printf("\r\nb:%d", structParams.bRecalculated);
//		if(!structParams->bRecalculated)
//		{
//				structParams->bStop = false;
//				structParams->iEncoderStartLocation = Location_Cnt;
//				structParams->iEncoderTargetLocation = fLocation;
//				//printf("\r\nstartcalc");
//				//步进电机初始速度以某个频率开始
//				structParams->fStartSpeed = g_fNominalStartSpeed;
//				//步进电机收尾速度以某个频率结束
//				structParams->fEndSpeed = g_fNominalEndSpeed;
//				//当S曲线使用梯形作为参考时使用的恒定加速度
//				structParams->fAccelerationAverage = g_fNominalAcceleration;
//				structParams->fDecelerationAverage = g_fNominalDeceleration;
//			
//				if(fSpeed < min(structParams->fStartSpeed, structParams->fEndSpeed))
//				{
//						fSpeedExpect = min(structParams->fStartSpeed, structParams->fEndSpeed);
//						structParams->bPlateauAll = true;
//				}
//				else if(fSpeed > min(structParams->fStartSpeed, structParams->fEndSpeed) && fSpeed < max(structParams->fStartSpeed, structParams->fEndSpeed))
//				{
//						fSpeedExpect = max(structParams->fStartSpeed, structParams->fEndSpeed);
//						structParams->bPlateauAll = true;
//				}
//				//给定速度大于起始速度且大于收尾速度时计算S曲线参数
//				else if(fSpeed > structParams->fStartSpeed && fSpeed > structParams->fEndSpeed)
//				{								
//						//先以梯形为参考
//						//计算加速段距离	(Vt ^ 2 - V0 ^ 2) / 2a = S, V0 = 0;
//						structParams->fAccelerationDistance = ( pow(fSpeed, 2) - pow(structParams->fStartSpeed, 2) ) / (2 * structParams->fAccelerationAverage);
//						//计算减速段距离
//						structParams->fDecelerationDistance = ( pow(structParams->fEndSpeed, 2) - pow(fSpeed, 2)) / (2 * structParams->fDecelerationAverage);
//						
//						structParams->fMaxSpeed = fSpeed;
//						
//						structParams->fMoveDistance = fabs(fLocation - structParams->fLastPosition);
//					
//						//计算匀速段距离
//						structParams->fPlateauDistance = structParams->fMoveDistance - structParams->fAccelerationDistance - structParams->fDecelerationDistance;
//					
////								printf("\r\nas:%f", structParams->fAccelerationDistance);
////								printf("\r\nds:%f", structParams->fDecelerationDistance);
////								printf("\r\nps:%f", structParams->fPlateauDistance);
//					
//						//计算匀速段小于0				
//						if( structParams->fPlateauDistance < 0)
//						{
//								//梯形退化为三角形
//								fZoomFactor = fabs(fLocation - structParams->fLastPosition) / ( structParams->fAccelerationDistance + structParams->fDecelerationDistance );
//								structParams->fAccelerationDistance = structParams->fAccelerationDistance * fZoomFactor;
//								structParams->fDecelerationDistance = structParams->fDecelerationDistance * fZoomFactor;
//								//重新计算所能达到的速度 Vt = sqrt( 2aS + V0 ^ 2)
//								structParams->fMaxSpeed = sqrt(2 * structParams->fAccelerationAverage * structParams->fAccelerationDistance + pow(structParams->fStartSpeed, 2));
//								if(structParams->fMaxSpeed < min(structParams->fStartSpeed, structParams->fEndSpeed))
//								{
//										fSpeed = min(structParams->fStartSpeed, structParams->fEndSpeed);
//										structParams->bPlateauAll = true;
//								}
//								else
//								{
//										structParams->fPlateauDistance = 0;
//								}
//						}
//						
//						if(!structParams->bPlateauAll)
//						{
//								//根据公式 V = (Vmax - V0) / ( 1 + exp( -aX + b ) ) + V0 计算		
//								//加速段S曲线形状参数计算，为运算方便去掉起始速度的叠加
//								structParams->fCurveSpeedAcc = fSpeed - structParams->fStartSpeed;	
//								
//								//以Vcurve = V / ( 1 + exp(-aX + b) )计算参数
//								//先以a = 1来确定b, 在X = 0处, Vcurve应当是一个极小值, 设为0.01, exp(b) = V / 0.01 - 1
//								structParams->fTransTimeAxisAcc = log(structParams->fCurveSpeedAcc / g_fMinimumValue - 1);
//								
//								//再根据梯形的加速度来确定a
//								//由于梯形加速度已经确定, 根据V = v0 + acc * t, v0 = 0, t = V / acc, 所以X = V / acc;
//								//在X = V / (acc * 1 / TimTick)处, V基本与Vcurve相等, 也就是 exp( -a * V / (acc * 1 / TimTick) + b)是一个极小值, 设为0.01
//								//exp( -a * V / (acc * 1 / TimTick) + b) = 0.01, -a * V / (acc * 1 / TimTick) + b = ln(0.01) = -4.605
//								//a = (4.605 + b) * (acc * 1 / TimTick) / V
//								structParams->fFlexTimeAxisAcc = (log(g_fMinimumValue) + structParams->fTransTimeAxisAcc) * structParams->fAccelerationAverage * 50 / structParams->fCurveSpeedAcc;
//								
//								//减速段S曲线形状参数计算
//								//去掉停止速度叠加后的公式为 Vcurve = V / (1 + exp(aX - b - B)),B为叠加加速段和匀速段后的平移量
//								structParams->fCurveSpeedDec = fSpeed - structParams->fEndSpeed;
//								
//								//先忽略平移量且先设Vcurve = V / (1 + exp(aX + b)), 在X = 0处, Vcurve为一个极小值, 设为0.01, 则exp(b) = V / 0.01 - 1
//								structParams->fTransTimeAxisDec = log(structParams->fCurveSpeedDec / g_fMinimumValue - 1);
//								//平移是个负数
//								structParams->fTransTimeAxisDec = structParams->fTransTimeAxisDec * -1;
//								//再根据梯形减速段的加速段来确定a
//								//V = v0 + dec * t, V = 0, t = - v0 / dec, 所以 X = -v0 / dec
//								//在 X = -v0 / (dec * 1 / TimeTick)处, Vcurve基本等于0, 设为0.01, 也就是 1 + exp( a * (-v0 / (dec * 1 / TimeTick)) + b) = v0 / 0.01
//								//a = (dec * 1 / TimeTick) * (ln(v0 / 0.01 - 1) - b) / -v0
//								structParams->fFlexTimeAxisDec = structParams->fDecelerationAverage * 50 * ( log(structParams->fCurveSpeedDec / g_fMinimumValue - 1) - structParams->fTransTimeAxisDec ) / (-structParams->fCurveSpeedDec);
//								
//								//至此S曲线段参数已全部确定
//								printf("\r\nas:%f", structParams->fAccelerationDistance);
//								printf("\r\nds:%f", structParams->fDecelerationDistance);
//								printf("\r\nps:%f", structParams->fPlateauDistance);
//								printf("\r\nfa:%f", structParams->fFlexTimeAxisAcc);
//								printf("\r\nta:%f", structParams->fTransTimeAxisAcc);
//								printf("\r\nfd:%f", structParams->fFlexTimeAxisDec);
//								printf("\r\ntd:%f", structParams->fTransTimeAxisDec);
//								//printf("\r\ne:%f", structParams->fCurveSpeedDec * 100 - 1);
//								
//								
//						}
//				}
//				else
//				{
//						fSpeedExpect = min(structParams->fStartSpeed, structParams->fEndSpeed);
//						structParams->bPlateauAll = true;
//				}
//				
//				structParams->bRecalculated = true;
//		}
		
		//不采用实时计算速度的方法
//		printf("\r\nt:%f", structParams->fCurveSpeedDec * 100 - 1);
		//计算当前期望速度值,由位置来确定
//		if(!structParams->bPlateauAll)
//		{
//				//printf("\r\ncalc");
//				if(fLocation > structParams->fLastPosition)
//				{
//						//加速段
//						if(structParams->fAccelerationDistance - fCurrentPosition > 0.01)
//						{
//								fSpeedExpect = structParams->fCurveSpeedAcc / (1 + exp(-structParams->fFlexTimeAxisAcc * structParams->iTimeTickAcc + structParams->fTransTimeAxisAcc)) + structParams->fStartSpeed;
//								structParams->iTimeTickAcc += 0.02;
//						}
//						//匀速段
//						else if( (structParams->fAccelerationDistance - fCurrentPosition < 0.01) && (structParams->fAccelerationDistance + structParams->fPlateauDistance - fCurrentPosition > 0.01) )
//						{
//								fSpeedExpect = structParams->fMaxSpeed;
//						}
//						//减速段
//						else if( (structParams->fAccelerationDistance + structParams->fPlateauDistance - fCurrentPosition < 0.01) && 
//										(structParams->fMoveDistance - fCurrentPosition > 0.01))
//						{
//								fSpeedExpect = structParams->fCurveSpeedDec / (1 + exp(structParams->fFlexTimeAxisDec * structParams->iTimeTickDec + structParams->fTransTimeAxisDec)) + structParams->fEndSpeed;
//								structParams->iTimeTickDec += 0.02;
//						}
//						else
//						{
//								fSpeedExpect = 0;
//						}
//				}	
//				else
//				{
//						//加速段
//						if(fCurrentPosition - structParams->fMoveDistance + structParams->fAccelerationDistance > 0.01)
//						{
//								fSpeedExpect = structParams->fCurveSpeedAcc / (1 + exp(-structParams->fFlexTimeAxisAcc * structParams->iTimeTickAcc + structParams->fTransTimeAxisAcc)) + structParams->fStartSpeed;
//								structParams->iTimeTickAcc += 0.02;
//								//printf("\r\nacc");
//						}
//						//匀速段
//						else if( (fCurrentPosition - structParams->fMoveDistance + structParams->fAccelerationDistance < 0.01) && 
//									(fCurrentPosition - structParams->fMoveDistance + structParams->fAccelerationDistance + structParams->fPlateauDistance > 0.01) )
//						{
//								fSpeedExpect = structParams->fMaxSpeed;
//						}
//						//减速段
//						else if( (fCurrentPosition - structParams->fMoveDistance + structParams->fAccelerationDistance + structParams->fPlateauDistance < 0.01) && 
//										(fCurrentPosition > 0.01))
//						{
//								fSpeedExpect = structParams->fCurveSpeedDec / (1 + exp(structParams->fFlexTimeAxisDec * structParams->iTimeTickDec + structParams->fTransTimeAxisDec)) + structParams->fEndSpeed;
//								structParams->iTimeTickDec += 0.02;
//								//printf("\r\ndec");
//						}
//						else
//						{
//								fSpeedExpect = 0;
//						}
//				}
//		}
//		
//		if(structParams->bStop)
//		{
//				fSpeedExpect = -1;
//		}
//		
//		structParams->fLastPosition = fCurrentPosition;
////		printf("\r\n%f", fSpeedExpect);
//		return fSpeedExpect;
}

void CurveBlockInit(CurveBlock *structBlock)
{
		structBlock->structParams.bStop = true;
		structBlock->m_pCurvePrepare = CurveBlockPrepare;
		structBlock->m_pCalcCurve = CalculateCurve;
		structBlock->m_pCurvePrepare(&structBlock->structParams);
		structBlock->m_pStop = CurveBlockStop;
		structBlock->m_pStop(&structBlock->structParams);
}

void MotorSpeedLocatin_Set (float speed, float lacation)
{
    float speed_step = 0.5;
	
    STEPMOTOR_TORQUE_Disable();
//    if ((speed - Vel_Exp_Val_tmp) > speed_step)
//		{
//				Vel_Exp_Val_tmp = Vel_Exp_Val_tmp + speed_step;
//    }
//    else if ((speed - Vel_Exp_Val_tmp) < -speed_step)
//		{
//				Vel_Exp_Val_tmp = Vel_Exp_Val_tmp - speed_step;
//    }
//    else
//    {
//    	Vel_Exp_Val_tmp = speed;
//    }
    CaptureNumber = Location_Cnt;
		MSF = structCurveBlock.structParams.iCurrentStepCount - structCurveBlock.structParams.iLastStepCount;
		structCurveBlock.structParams.iLastStepCount = structCurveBlock.structParams.iCurrentStepCount;
//    MSF = GetEncoder.V3;
    MSF = abs (MSF);

    //对速度进行累计,得到1s内的脉冲数
    SUM_Pulse += MSF;
    //位置环PID计算,根据计算结果判断电机运动方向
//    Dis_Exp_Val = IncPIDCalc (Location_Cnt, lacation, &sPID);
//    Motion_Dir = Dis_Exp_Val < 0 ? CCW : CW;
		Motion_Dir = Location_Cnt < lacation ? CW : CCW;
//    Dis_Exp_Val = abs (Dis_Exp_Val);
//    if (Vel_Exp_Val_tmp <= 0)
//    {
//		STEPMOTOR_OUTPUT_DISABLE(); //速度为零直接停止
//    }
//    else
    {
        //位置环输出作为速度环的输入,需要限制位置环的输出不会超过速度环目标值
//        if (Dis_Exp_Val >= Vel_Exp_Val_tmp)
//        {
//            Dis_Exp_Val = Vel_Exp_Val_tmp;
//        }
//        Vel_Exp_Val += IncPIDCalc (MSF, Dis_Exp_Val, &vPID);
				
				
				
        //当到达目标位置的时候,这时候已经电机非常慢了.为了减少超调,可以直接将速度环的输出清零
        if (Vel_Exp_Val <= 0.1)         
        {
						//Vel_Exp_Val = 0;          //这里不再做速度清零，以免期望速度过小时电机停止 YanBing
           	Vel_Exp_Val_tmp = 5;
						//Vel_Exp_Val_tmp = 1;
        }
        else if (Vel_Exp_Val > (float)motor_maxspeed)
				{
						Vel_Exp_Val = (float)motor_maxspeed;
        }
        
        if(Vel_Exp_Val / (MSF + 1) > 5)         //堵转后的处理，以免堵转后因频率过高导致电机只响不动 YanBing
        {
            Vel_Exp_Val = 1;
        }
        
//				if(structCurveBlock.m_pCalcCurve == NULL)
//				{
//						printf("\r\nnull");
//				}
//				else
				{
//					g_fSpeedExpect = structCurveBlock.m_pCalcCurve(&structCurveBlock.structParams, speed, lacation, Location_Cnt);
						structCurveBlock.m_pCalcCurve(&structCurveBlock.structParams, speed, lacation);
//						if(structCurveBlock.structParams.bBusy)
//						{
//								return;
//						}
				}
//        if(motor_limit_flag == 2)
//        {
//            if(Motion_Dir == CW)
//            {
//                STEPMOTOR_OUTPUT_DISABLE();
//                //motor_stop_flag = 1;
//                Vel_Exp_Val = 0;
//                //Dis_Target = Location_Cnt;
//                gCurrent_pos = Location_Cnt;
//            }
//        }
        
        /* 经过PID计算得到的结果是编码器的输出期望值的增量,
        需要转换为步进电机的控制量(频率值),这里乘上一个系数6400/2400
        */
 //       STEPMOTOR_Motion_Ctrl (Motion_Dir, Vel_Exp_Val * FEEDBACK_CONST); //乘上一个系数,6400/2400,将PID计算结果转换为步进电机的频率(速度)
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

//    if (Frequency <= 0)
//    {
//         STEPMOTOR_OUTPUT_DISABLE();
//        //STEPMOTOR_TORQUE_Enable();
//    }
//    else
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
//				return;
//        Step_Delay = (uint16_t)(FREQ_UINT / Frequency);
         STEPMOTOR_OUTPUT_ENABLE();
					return;
        // STEPMOTOR_TORQUE_Disable();
//        Toggle_Pulse = Step_Delay >> 1;//算出来的结果是周期
//        if(Toggle_Pulse < 30)   //避免出现过小的比较输出值，而频繁进入中断 YanBing
//        {
//            Toggle_Pulse = 30; 
//        }
	//	printf ("\r\n Toggle_Pulse:%d", Toggle_Pulse);	//test
    }
}

void STEPMOTOR_OUTPUT_DISABLE (void) //电机停止运转
{
    //SPI_DRV8711_Write (CTRL_Register_ADDR, drv8711_ctrl_value & 0xfffe);
    SPI_DRV8711_Write (CTRL_Register_ADDR, drv8711_ctrl_value | 0x0001);
    HAL_TIM_OC_Stop_IT (&htim2, TIM_CHANNEL_1);
    motor_stop_flag = 1;
}

void STEPMOTOR_OUTPUT_ENABLE (void) //电机运转
{
    SPI_DRV8711_Write (CTRL_Register_ADDR, drv8711_ctrl_value | 0x0001);
    HAL_TIM_OC_Start_IT ( &htim2,TIM_CHANNEL_1 );
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
