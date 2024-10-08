//
// Created by 19734wtf on 2023/3/27.
//
#include <stdint-gcc.h>
#include "pid.h"
#include "usart.h"
#include "DeepMotor.h"

PIDTypeDef AngleLoop[7];
PIDTypeDef SpeedLoop[7];
PIDTypeDef Torque[7];

void PID_Init(PIDTypeDef *pid)
{
    pid->Setpoint=0.0f;
    pid->SumError=0.0f;

    pid->P=0.0f;
    pid->I=0.0f;
    pid->D=0.0f;

    pid->Last_error=0.0f;
    pid->LLast_error=0.0f;
}

//PID基础配置
void PID_Set_KP_KI_KD(PIDTypeDef *pid,float kp,float ki,float kd)
{
    pid->P=kp;pid->I=ki;pid->D=kd;
}

//PID位置环目标确定
void SetPoint(PIDTypeDef *pid,float want,uint8_t id)
{
    pid->Setpoint = want + begin_pos[id];    //在初始零点的基础上进行位置控制
}
void SetPoint_IMU(PIDTypeDef *pid,float want)
{
    pid->Setpoint=want;
}
//PID速度环目标确定
void SetPoint_Speed(PIDTypeDef *pid,float want)
{
    pid->Setpoint=want;
}

//位置式PID计算(用于位置（角度）环，其输出作为速度环的目标值)
void PID_PosLocCalc(PIDTypeDef *pid, float feedbackpos,uint8_t id)//位置式
{
    float Now_Point,Now_Error,d_Error;
    Now_Point = feedbackpos;
    Now_Error=pid->Setpoint-Now_Point;
    pid->SumError+=Now_Error;//这部分进行了累加，从而导致积分饱和现象。
    //积分限幅（而增量式PID不需要积分限幅）积分限幅有两种思路，一种是限制sum（相对限制），另一种是限制I*sum（绝对限制），后者我认为更加合理，但考虑到新老代码的兼容性，仍然采用前者。
    if(pid->SumError     >  pid->SumError_limit) pid->SumError= pid->SumError_limit;
    else if(pid->SumError< -pid->SumError_limit) pid->SumError=-pid->SumError_limit;

    d_Error = Now_Error - pid->Last_error;//误差之差，表示微分。
    pid->Last_error=Now_Error;

    pid->Iout = pid->I * pid->SumError;

    pid->Out_put = pid->P * Now_Error +
                   pid->Iout +
                   pid->D * d_Error;

    //限幅输出
    if(pid->Out_put     > pid->Output_limit) pid->Out_put= pid->Output_limit;
    else if(pid->Out_put<-pid->Output_limit) pid->Out_put=-pid->Output_limit;
}

void PID_PosLocCalc_IMU(PIDTypeDef *pid, int32_t feedbackpos)//位置式
{
    float Now_Point,Now_Error,d_Error;
    Now_Point = (float )feedbackpos;
    Now_Error=pid->Setpoint-Now_Point;
    pid->SumError+=Now_Error;//这部分进行了累加，从而导致积分饱和现象。
    //积分限幅（而增量式PID不需要积分限幅）积分限幅有两种思路，一种是限制sum（相对限制），另一种是限制I*sum（绝对限制），后者我认为更加合理，但考虑到新老代码的兼容性，仍然采用前者。
    if(pid->SumError     >  pid->SumError_limit) pid->SumError= pid->SumError_limit;
    else if(pid->SumError< -pid->SumError_limit) pid->SumError=-pid->SumError_limit;

    d_Error = Now_Error - pid->Last_error;//误差之差，表示微分。
    pid->Last_error=Now_Error;

    pid->Iout = pid->I * pid->SumError;
    if(pid->Iout      > pid->Iout_limit) pid->Iout = pid->Iout_limit;
    else if(pid->Iout < -pid->Iout_limit) pid->Iout = -pid->Iout_limit;

    pid->Out_put = pid->P * Now_Error +
                   pid->Iout +
                   pid->D * d_Error;

    //限幅输出
    if(pid->Out_put     > pid->Output_limit) pid->Out_put= pid->Output_limit;
    else if(pid->Out_put<-pid->Output_limit) pid->Out_put=-pid->Output_limit;
}

//增量式PID计算（用于速度环，其输出作为电调的控制电流值）
void PID_IncCalc(PIDTypeDef *pid,int16_t feedbackspeed)//增量式
{
    float p_Error,Now_Error,d_Error,i_Error,Now_Point;
    Now_Point = (float )feedbackspeed;
    Now_Error=pid->Setpoint-Now_Point;
    p_Error=Now_Error - pid->Last_error;//p分量
    i_Error=Now_Error;//I分量
    d_Error=Now_Error - 2*pid->Last_error + pid->LLast_error;//D分量(LLast_error是增量式PID的标志)
    pid->Last_error=Now_Error;
    pid->LLast_error=pid->Last_error;
    pid->Out_put += (pid->P*p_Error+  //与位置式的最大区别在于这里用的是“+=”而非“=”。
                              pid->I*i_Error+  //积分被累加到了输出上，因此只需输出限幅，而无需积分限幅。
                              pid->D*d_Error);
    //限幅输出
    if(pid->Out_put>pid->Output_limit) pid->Out_put=pid->Output_limit;
    else if (pid->Out_put<-pid->Output_limit) pid->Out_put=-pid->Output_limit;
/*
	为什么速度环采用增量式PID？
		1.因为速度环往往是要求保持一定的速度，当我们需要维持一定的速度时，就需要维持一定的电流，
		一旦当前速度与目标速度产生偏差，增量式PID就会对应产生一个增量输出，从而对电流值进行调节。
		因而，可以说，如果说位置式pid的输出对应着当前值与目标值的偏差量，那么增量式pid则对应着随时
		发生的变化量。即，位置式只要有误差就有输出，而增量式只有误差发生变化才有输出。
		2.
*/
}

/*!
 * 修改伪8自由度的8个电机的PID参数
 * @param pos_kp
 * @param pos_kd
 */
void ChangeGainOfPID(float pos_kp,float pos_kd,float sp_kp,float sp_ki)
{
    for (uint8_t i=1;i<7;i++)
    {
        PID_Set_KP_KI_KD(&AngleLoop[i],pos_kp,0,pos_kd);
        PID_Set_KP_KI_KD(&SpeedLoop[i],sp_kp,sp_ki,0);
    }
}

/*!
 * 初始化伪8自由度8个电机的PID参数
 */
void Six_PID_Init(void )
{
    for(uint8_t i=1;i<7;i++)
    {
        PID_Init(&AngleLoop[i]);
        PID_Init(&SpeedLoop[i]);
        PID_Init(&Torque[i]);
    }
}
