/*
 * pid.h
 *
 *  Created on: Jun 16, 2024
 *      Author: yu
 */

#ifndef INC_PID_H_
#define INC_PID_H_


#include "bsp_can.h"
#include "main.h"
#include "stdbool.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;

	float separationThreshold;//分离阈值
	float Integral;//积分
	float Input;//输入
	float Output;//输出
	float Integral_Max;//积分上限
	float Dead_Band;//死区
	float Max_out;//最大输出
	float Max_input;//最大输入
	float E[3];//误差值
	float D_last;
	uint8_t Mode;//0为位置式 1为增量式

}PidTypedef;//PidTypedef要定义在头文件中，且源文件要包含头文件

//定义一个斜坡函数用的结构体
typedef struct RampGenerator
{
	float CurrentValue;
	float TargetValue;
	float Step;//步长，控制每个周期改变数值的大小
	bool isbusy;//指示斜坡发生器是否在调整中

}RampGenerator;

float funca_abs(float value);

extern PidTypedef speed_pid[6];
extern PidTypedef position_pid[2];
void PID_Calc(PidTypedef *Pid,float return_value,float set_value);
//void Pid_Init(PidTypedef *Pid ,float kp,float ki,float kd,float max_out,float dead_band,float integral,float max_input,uint8_t mode);
void PID_Cascade(PidTypedef *Pid,float return_value,float set_value);
void PID_Calc_Position(PidTypedef *Pid,float return_value,float set_value);//外环位置
void Pid_Init(PidTypedef *Pid ,float kp,float ki,float kd,float max_out,float dead_band,float integral,float max_input,uint8_t mode,float integral_max);
float rampIterate(RampGenerator*ramp);
void rampInit(RampGenerator*ramp,float startvalue,float targetvalue,float time,float cycletime);
void xielv(float output);
void zhuanxianghuan(float zxh_value,float Ke);
float first_order_filter(float data);
void num_xielv(float num_value,float hk);
int turn(float speeed_left, float speeed_right);
#endif /* INC_	PID_H_ */
