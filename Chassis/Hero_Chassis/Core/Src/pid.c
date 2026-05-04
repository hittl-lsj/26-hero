/*
 * pid.c
 *
 *  Created on: Jun 16, 2024
 *      Author: yu
 */


#include "pid.h"
#include "bsp_can.h"
#include "stdio.h"

PidTypedef speed_pid[6]={0};
PidTypedef position_pid[2]={0};

float Current_value;
float Last_value;
float k;


float zxh_output;
void Pid_Init(PidTypedef *Pid ,float kp,float ki,float kd,float max_out,float dead_band,float integral,float SeparationThreshold,uint8_t mode,float integral_max)
{
	Pid->Kp=kp;
	Pid->Ki=ki;
	Pid->Kd=kd;
	Pid->Input=0;
	Pid->Output=0;
	Pid->Integral=integral;//积分
	Pid->Dead_Band=dead_band;//死驱
	Pid->Max_out=max_out;//最大输出
//	Pid->Max_input=max_input;//最大输入
	Pid->Integral=0;
	Pid->E[0]=0;
	Pid->E[1]=0;
	Pid->E[2]=0;
	Pid->D_last=0;
	Pid->Integral_Max=integral_max;//最大积分
	Pid->separationThreshold=SeparationThreshold;

}

float func_Limit(float value,float max,float min)
{
	if(value>max)
	{
		return value=max;
	}
	else if(value<min)
	{
		return value=min;
	}
	else
		return value;
}

float funca_abs(float value)
{
	if(value>=0)
		return value;
	else
		return -value;
}
float func_abs(float value)
{
	if(value>=0)
		return value;
	else
		return -value;
}

void PID_Calc(PidTypedef *Pid,float return_value,float set_value)//速度环
{
	float p=0,i=0,d=0;
	    // 保存前一次和前两次误差
	    Pid->E[0] = Pid->E[1];
	    Pid->E[1] = Pid->E[2];

	    Pid->E[2]=set_value-return_value;//	Pid->Integral+=Pid->E[2];
	if(Pid->Mode==0)//位置环
	{

			Pid->Integral+=Pid->Ki*Pid->E[2];//Ki乘误差，然后让它累加得到积分
			Pid->Integral=func_Limit(Pid->Integral,Pid->Integral_Max,-(Pid->Integral_Max));


			p=Pid->Kp*(Pid->E[2]);
			i=Pid->Integral;
			d=(Pid->Kd)*(Pid->E[2]-Pid->E[1]);
			Pid->D_last=d;//不知道有什么用处
			Pid->Output=(int16_t)func_Limit(p+i+d,Pid->Max_out,-(Pid->Max_out));

	}
	else if(Pid->Mode==1)//速度环
	{
		if(func_abs(Pid->E[2])>=Pid->Dead_Band)//当偏差值大于等于这个死区，进入这个if
		{
			p=(Pid->Kp)*(Pid->E[2]-Pid->E[1]);
			i=(Pid->Ki)*(Pid->E[2]);
			d=(Pid->Kd)*(Pid->E[2]-2*(Pid->E[1])+Pid->E[0]);//注意没写*也会出错
			Pid->Output=func_Limit(Pid->Output,Pid->Max_out,-(Pid->Max_out));
		}
	}
	else
		Pid->Output=0;

}

int Position_Moto;
int Moto;



void PID_Calc_Position(PidTypedef *Pid,float return_value,float set_value)
{
		float p1=0,i1=0,d1=0;
		    // 保存前一次和前两次误差
		    Pid->E[0] = Pid->E[1];
		    Pid->E[1] = Pid->E[2];

		Pid->E[2]=func_Limit(set_value, Pid->Max_input,-(Pid->Max_input))-return_value;//当前的误差
		if(Pid->Mode==0)//位置环
		{
			if(func_abs(Pid->E[2])<=Pid->Integral)//如果当前误差绝对值小于积分
			{
				Pid->Integral=Pid->Integral+((Pid->Ki)*(Pid->E[2])+(Pid->Ki)*(Pid->E[1]))/2;//暂时不知道它有何用处
			}
			else
			{
				Pid->Integral=Pid->Integral;
			}
			Pid->Integral=func_Limit(Pid->Integral,Pid->Integral_Max,(-Pid->Integral_Max));



			p1=Pid->Kp*(Pid->E[2]);
			i1=Pid->Integral;
			d1=(Pid->Kd)*(Pid->E[2]-Pid->E[1]);
			Pid->D_last=d1;//不知道有什么用处
			Pid->Output=p1+i1+d1;

		}
}

//float num_value;
float retain_value;
float join_value=0;


void num_xielv(float num_value,float hk)
{
	float Last_num_value;
	retain_value=num_value;
	if(num_value>0)
	{
		join_value+=hk;
		if(join_value>retain_value)
		{
			join_value=retain_value;
		}
	}
	if(num_value<0)
	{
		join_value-=hk;
		if(join_value<retain_value)
		{
			join_value=retain_value;
		}
	}
//	if(num_value==0)
//	{
//		if((Last_num_value-num_value)>0)
//		{
//
//		}
//	}
//
//	Last_num_value=num_value;
}


//一个周期内对斜坡函数发生器的更新
float rampIterate(RampGenerator*ramp)
{
	if(ramp->isbusy)
	{
		if(ramp->CurrentValue<ramp->TargetValue)//如果当前值小于目标值
		{
			ramp->CurrentValue+=ramp->Step;//累加
			if(ramp->CurrentValue>ramp->TargetValue)//如果超了
			{
				ramp->CurrentValue=ramp->TargetValue;//就直接等于
			}

		}
		else if(ramp->CurrentValue>ramp->TargetValue)//如果当前值小于目标值
		{
			ramp->CurrentValue-=ramp->Step;//累减
			if(ramp->CurrentValue<ramp->TargetValue)//如果小了，就直接等于
			{
				ramp->CurrentValue=ramp->TargetValue;
			}
		}
		//判断是否达到目标
		if(ramp->CurrentValue==ramp->TargetValue)
		{
			ramp->isbusy=false;//达到目标，当作不忙碌
		}

	}

	return ramp->CurrentValue;
}

//初始化，斜坡发生器
void rampInit(RampGenerator*ramp,float startvalue,float targetvalue,float time,float cycletime)
{
	//将值先给出去
	ramp->CurrentValue=startvalue;
	ramp->TargetValue=targetvalue;

	if(time!=0&&cycletime!=0)
	{
		ramp->Step=(targetvalue-startvalue)*(cycletime/time);//计算步长
	}
	else
	{
		ramp->Step=0;//让步长为0
	}
	ramp->isbusy=true;//为真忙
}

//计算输出的斜坡函数
void xielv(float output)
{
			  Current_value=output;
			  k=(Current_value-Last_value)/0.001;
			  if(k>1)
			  {
				  k=0.1;
			  }
			  else if(k<-1)
			  {
				  k=-0.1;
			  }
			  output=0.001*k+Last_value;
			  Last_value=Current_value;


}
//计算转向差
void zhuanxianghuan(float zxh_value,float Ke)
{
	float speed_bias;
	speed_bias=Engine[2].speeed-Engine[0].speeed;
	zxh_output=Ke*speed_bias;
}
float final = 0.0F;
float A = 0.25F;    //a的取值为0~1
float first_order_filter(float data)
{
	final = A*data + (1-A)*final;    //两次数据乘上各自的权重
	return  (final);
}

float speeed_Kp;
float input_right;
//得到的转每分，改为码每秒
int turn(float speeed_left, float speeed_right)
{
	float turn, speeed_bias;
	speeed_bias = speeed_left - speeed_right;//转每分
		// 单位换算      //
	input_right=2593.8166666666666666*speeed_bias;
	turn = speeed_Kp * speeed_bias;
	return turn;
}








//void zhuanxianghuan_yaw()
//{
//	yaw_angle=a.yaw;
//	yaw_bias=yaw_angle-Last_yaw_angle;
//	yaw_output=ka*yaw_bias+kb;
//	if(yaw_bias>0)
//	{
//		left_output=-yaw_bias/2;
//		right_output=yaw_bias/2;
//	}
//	if(yaw_bias<0)
//	{
//		left_output=yaw_bias/2;
//		right_output=-yaw_bias/2;
//
//	}
//
//
//	Last_yaw_angle=yaw_angle;
//}























