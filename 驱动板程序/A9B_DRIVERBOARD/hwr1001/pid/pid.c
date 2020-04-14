#include "includes.h"
#include "pid.h"

//绝对式PID算法
void PID_AbsoluteMode (PID_AbsoluteType* PID)
{
    if(PID->kp      < 0)    PID->kp      = -PID->kp;
    if(PID->ki      < 0)    PID->ki      = -PID->ki;
    if(PID->kd      < 0)    PID->kd      = -PID->kd;
    if(PID->errILim < 0)    PID->errILim = -PID->errILim;
    PID->errP = PID->errNow;  //读取现在的误差，用于kp控制
    PID->errI += PID->errNow; //误差积分，用于ki控制
    if(PID->errILim != 0)     //微分上限和下限
    {
        if(     PID->errI >  PID->errILim)    PID->errI =  PID->errILim;
        else if(PID->errI < -PID->errILim)    PID->errI = -PID->errILim;
    }
    PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制
    PID->errOld = PID->errNow; //保存现在的误差
    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出
}
  
//增量式PID算法
void PID_IncrementMode (PID_IncrementType* PID)
{
    float dErrP, dErrI, dErrD;

    if(PID->kp < 0)    PID->kp = -PID->kp;
    if(PID->ki < 0)    PID->ki = -PID->ki;
    if(PID->kd < 0)    PID->kd = -PID->kd;
    dErrP = PID->errNow - PID->errOld1;
    dErrI = PID->errNow;
    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;
    PID->errOld2 = PID->errOld1; //二阶误差微分
    PID->errOld1 = PID->errNow;  //一阶误差微分
    /*增量式PID计算*/
    PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
    if(PID->kp == 0 && PID->ki == 0 && PID->kd == 0)   PID->ctrOut = 0;
    else PID->ctrOut += PID->dCtrOut;
}
