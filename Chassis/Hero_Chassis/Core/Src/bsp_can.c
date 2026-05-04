#include "bsp_can.h"
#include "pid.h"
#include "can.h"
#include "gpio.h"
//定义can数据缓存
uint8_t Data[8]={0};
Motor_t myMotor;
int32_t current_angle[6]={0};
int32_t Last_angle[6]={0};
int32_t err[6]={0};
int32_t a[6];

extern f;
//初始化can
void can_Init(CAN_HandleTypeDef*hcan)
{
	//配置can的过滤器
		  CAN_FilterTypeDef  can_filter;
		  can_filter.FilterBank = 0;                       // filter 0
		  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
		  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
		  can_filter.FilterIdHigh = 0;
		  can_filter.FilterIdLow  = 0;
		  can_filter.FilterMaskIdHigh = 0;
		  can_filter.FilterMaskIdLow  = 0;                // set mask 0 to receive all can id
		  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
		  can_filter.FilterActivation = ENABLE;           // enable can filter
		  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode

		  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter


	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//开启在fifo0中的接收中断


}
//将电压控制指令发送到can总线上
void boxSend1(int16_t I1,int16_t I2,int16_t I3,int16_t I4)//电压控制1，范围在-25000到25000
{
	static CAN_TxHeaderTypeDef txheader;//创建发送报文结构体
 	txheader.DLC=8;
	txheader.IDE=CAN_ID_STD;
	txheader.RTR=CAN_RTR_DATA;
	txheader.StdId=0x200;//电流控制-16384到16384
	Data[0]=(uint8_t)(I1>>8);//高八位
	Data[1]=(uint8_t)I1;//低八位

	Data[2]=(uint8_t)(I2>>8);
	Data[3]=(uint8_t)I2;

	Data[4]=(uint8_t)(I3>>8);
	Data[5]=(uint8_t)I3;

	Data[6]=(uint8_t)(I4>>8);
	Data[7]=(uint8_t)I4;

	HAL_CAN_AddTxMessage(&hcan1,&txheader,Data,(uint32_t*)CAN_TX_MAILBOX0);
}
void boxSend2(int16_t I1,int16_t I2,int16_t I3,int16_t I4)//电压控制1，范围在-25000到25000
{
	static CAN_TxHeaderTypeDef txheader;//创建发送报文结构体
 	txheader.DLC=8;
	txheader.IDE=CAN_ID_STD;
	txheader.RTR=CAN_RTR_DATA;
	txheader.StdId=0x1FF;//电流控制-16384到16384
	Data[0]=(uint8_t)(I1>>8);//高八位
	Data[1]=(uint8_t)I1;//低八位

	Data[2]=(uint8_t)(I2>>8);
	Data[3]=(uint8_t)I2;

	Data[4]=(uint8_t)(I3>>8);
	Data[5]=(uint8_t)I3;

	Data[6]=(uint8_t)(I4>>8);
	Data[7]=(uint8_t)I4;

	HAL_CAN_AddTxMessage(&hcan1,&txheader,Data,(uint32_t*)CAN_TX_MAILBOX0);
}
//将can数据解析为电机数据
void motorset(Motor_t*Receive,uint8_t Data[]) //
{
	Receive->angle=(Data[0]<<8)|Data[1];//转子机械角度
	Receive->speeed=(Data[2]<<8)|Data[3];//转速
	Receive->torque=(Data[4]<<8)|Data[5];//输出转矩
	Receive->temp=Data[6];//温度
}
Motor_t Engine[6]={0};//控制四个电机
//将角度转换为多圈角度
void Null_Point(int32_t *D_value)
{
	if (*D_value>4096)	*D_value=*D_value-8192;
	if (*D_value<-4096)	*D_value=*D_value+8192;

}
//计算多圈角度
void angle_multiround_calc(uint8_t id)
{
	     current_angle[id]=Engine[id].angle;//得到当前角度
	     err[id]=current_angle[id]-Last_angle[id];//角度�??????
	     Null_Point(&err[id]);//零点判断
	     Engine[id].angle_multiround+=err[id];//得到当前位置
	     Engine[id].angle_multiround=Engine[id].angle_multiround-a[id];
	     Last_angle[id]=current_angle[id];//角度更新
	     if(f-Engine[id].angle_multiround<2&&f-Engine[id].angle_multiround>-2)
	     {
	    	 speed_pid->E[0]=0;
	    	 speed_pid->E[1]=0;
	    	 speed_pid->E[2]=0;
	    	 position_pid->E[2]=0;
	    	 position_pid->E[1]=0;
	    	 position_pid->E[0]=0;
	     }


}

//在fifo0的中断回调里面完成数据接收
static int flag=0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)
{
	if(hcan->Instance==CAN1)
	{

		if(flag<2)
			flag++;
		if(flag==1)
		{
			a[1]=Engine[1].angle;//将第一次编码值给到a
			a[2]=Engine[2].angle;//将第二次编码值给到a
			a[3]=Engine[3].angle;//将第三次编码值给到a
			a[4]=Engine[4].angle;//将第四次编码值给到a
		}
		else
		{
				    uint8_t ReceiveData[8]={0};
					static CAN_RxHeaderTypeDef Rxheader;//创建接收报文结构体
					HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rxheader,ReceiveData);//将信息接收到fifo0邮箱
					switch(Rxheader.StdId)//判断ID
					{
					case 0x201:
					{
						motorset(&Engine[0],ReceiveData);
						angle_multiround_calc(0);

						break;
					}
					case 0x202:
					{
						motorset(&Engine[1],ReceiveData);//处理接收到的数据，拿出来
						angle_multiround_calc(1);

						break;
					}
					case 0x203:
					{
						motorset(&Engine[2],ReceiveData);
						angle_multiround_calc(2);
						break;
					}
					case 0x204:
					{
						motorset(&Engine[3],ReceiveData);//处理接收到的数据，拿出来  //纵轴数据
						angle_multiround_calc(3);
						break;
					}
					case 0x205:
					{
						motorset(&Engine[4],ReceiveData);//处理接收到的数据，拿出来  //横轴数据
						angle_multiround_calc(4);
						break;
					}
					case 0x206:
					{
						motorset(&Engine[5],ReceiveData);//处理接收到的数据，拿出来  //旋转轴数据
						angle_multiround_calc(5);
						break;
					}

					}

		}
	}
}






/*
 * bsp_can.c
 *
 *  Created on: Jun 3, 2024
 *      Author: yu
 */

