#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>
/* 私有宏定义 ----------------------------------------------------------------*/
#define SAMPLING                    0x01    // 采样标记
#define TXD                         0x02    // 发送数据标记
#define MAX_SPEED                   200
//#define abs(x)    ((x)<0?(-x):(x))
#define SENDBUFF_SIZE               100     // 串口DMA发送缓冲区大小

// 定义定时器预分频，定时器实际时钟频率为：72MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define STEPMOTOR_TIM_PRESCALER               119  // 实际时钟频率为：300KHz,降低定时器频率,避免计数器溢出 36000000/120=300K
// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define STEPMOTOR_TIM_PERIOD                  0xFFFF

//编码器及丝杠的一些参数
#define SECOND                                1000  // 常数:1s == 1000 ms
#define SAMPLING_PERIOD                       20    // 编码器采样周期 单位ms
#define FREQ_UINT                             (T1_FREQ/(SECOND/SAMPLING_PERIOD))    //对定时器的频率做单位换算,避免数值太大溢出
#define ENCODER_SPR              			  (encoder_lines*4)     		// 编码器单圈线数*4倍频
#define MPR                         		  5       		    // 步进电机旋转一圈,丝杠的距离;单位：mm/r 
#define PPM      							  (ENCODER_SPR/MPR)  		// 每mm内编码器的脉冲数;单位:Pules/mm
#define P_PERIOD                              ((float)PPM/(SECOND/SAMPLING_PERIOD))  // 每个采样周期对应的脉冲数 
#define MPP                                   ((float)(MPR)/ENCODER_SPR) // 编码器单步步进距离

//// 步进电机及驱动器的一些参数
#define T1_FREQ                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER+1)) // 定时器1频率F值
#define FSPR                                  200         //步进电机单圈步数
#define MICRO_STEP                            motor_step_value          // 步进电机驱动器细分数
#define SPR                                   ((float)(FSPR*MICRO_STEP))   // 旋转一圈需要的脉冲数

#define FEEDBACK_CONST                        (SPR/ENCODER_SPR)   //编码器和步进电机驱动器的比值
#define CCW                                   1
#define CW                                    0

#define TOGGLE_BUFFER_LEN											20
/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR;

extern uint16_t Toggle_Pulse;         // 比较输出周期，值越小输出频率越快 
extern uint16_t StepMotor_Pulse_cnt;
extern uint32_t tim_Pulse_count;

extern uint8_t  motor_reduction_ratio;

extern uint8_t Time_Flag;
extern uint32_t motor_pulse_total;
extern uint8_t aTxBuffer[SENDBUFF_SIZE];
extern int32_t CaptureNumber;
extern uint16_t SUM_Pulse;
extern uint16_t encoder_lines;
extern uint8_t drv_torque_value;
extern  int32_t MSF;
extern  float Vel_Exp_Val_tmp;

extern float Dis_Target;             // 目标位置所对应编码器脉冲值
extern float Vel_Target;             // 每单位采样周期内的脉冲数(频率)
extern float Vel_Exp_Val;
extern float Dis_Exp_Val;
/* 函数声明 ------------------------------------------------------------------*/
#define Pi      3.1415927
#define Beita     Pi/4

typedef struct
{
		//起始速度
		uint8_t iStartSpeed;
		//收尾速度
		uint8_t iEndSpeed;
		//急动度
		double dJerk;
		//TIM比较输出值
		uint16_t iOC_Value;
		//运动方向
		uint8_t iDirection;
		//当前所走距离计数
		int32_t iCurrentStepCount;
		int32_t iLastStepCount;
		//加速段距离
		float fAccelerationDistance;
		//减速段距离
		float fDecelerationDistance;
		//运动距离
		float fMoveDistance;		
		//重计算曲线
		bool bRecalculated;
		//运动块正忙标志
		bool bBusy;
		//全程匀速运动标志
		bool bPlateauAll;
		//停止标志
		bool bStop;
	
		//加速段距离-OC值表
		uint16_t arrAccDivisionTable[20][2];
		//减速段距离-OC值表
		uint16_t arrDecDivisionTable[20][2];
		//加速段表中距离累加值
		uint32_t iAccStepComplete;
		//减速段表中距离累加值
		uint32_t iDecStepComplete;
		//加速段表索引值
		uint8_t iAccStepIndex;
		//减速段表索引值
		uint8_t iDecStepIndex;
		
		//加速段需要加上表中的距离值
		bool bAccAddStep;
		//减速段需要加上表中的距离值
		bool bDecAddStep;
		//最大速度
		uint16_t iMaxSpeed;
		//加速段编码器距离
		uint32_t iEncoderAccDist;
		//减速段编码器距离
		uint32_t iEncoderDecDist;
		//
		float fPlateauDistance;
		uint32_t iEncoderPlatDist;
		int32_t iEncoderTargetLocation;
		int32_t iEncoderStartLocation;
		uint32_t iEncoderMoveDist;
//		float fTransTimeAxisAcc;
//		float fFlexTimeAxisAcc;
//		float fTransTimeAxisDec;
//		float fFlexTimeAxisDec;
//		float fAccelerationAverage;		
//		float fDecelerationAverage;		
//		float fTargetPosition;
//		float iTimeTickAcc;
//		float iTimeTickDec;
}CurveParams;

//#define min(a, b) 	( ((a) < (b)) ? (a) : (b) )
//#define max(a, b)		( ((a) > (b)) ? (a) : (b) )
void CurveBlockPrepare(CurveParams *structParams);
void CalculateCurve(CurveParams *structParams, uint16_t iSpeed, int32_t iTargetLocation);

typedef struct
{
		CurveParams structParams;
		uint16_t m_ToggleAcc[TOGGLE_BUFFER_LEN];
		uint16_t m_ToggleDec[TOGGLE_BUFFER_LEN];
		void (*m_pCurvePrepare) (CurveParams *structParams);
		void (*m_pCalcCurve) (CurveParams *structParams, uint16_t iSpeed, int32_t iTargetLocation);
		void (*m_pStop) (CurveParams *structParams);
}CurveBlock;

extern CurveBlock structCurveBlock;
void CurveBlockInit(CurveBlock *structBlock);
extern float g_fSpeedExpect;
void STEPMOTOR_DIR_FORWARD (void);
void STEPMOTOR_DIR_REVERSAL (void);

void STEPMOTOR_TORQUE_Disable (void);//失能力矩
void STEPMOTOR_TORQUE_Enable (void);//使能力矩

void STEPMOTOR_Motion_Ctrl (uint8_t Dir , float Speed);
void STEPMOTOR_OUTPUT_DISABLE (void);
void STEPMOTOR_OUTPUT_ENABLE (void);
void DevelopmentFramwork (void);
void Get_Motor_Statues (void);
void MotorSpeedLocatin_Set (float speed, float lacation);
void PID_Init (void);
#endif
