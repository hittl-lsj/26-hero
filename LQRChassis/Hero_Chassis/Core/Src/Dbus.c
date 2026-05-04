/*
 * Dbus.c
 *
 *  Created on: Jul 17, 2024
 *      Author: yu
 */


#include "Dbus.h"
#include "string.h"
#include "memory.h"




RC_Ctrl_t rc_ctrl;
extern UART_HandleTypeDef huart3;//要写一个这个，不然会报错
uint16_t DR16_Received;
//官方解码
void dbus_to_rc(volatile const uint8_t *dbus_buf,RC_Ctrl_t *rc_ctrl)//volatile 表示重新读取，const 只读
{
	if(dbus_buf==NULL||rc_ctrl==NULL)
	{
		return;
	}

	rc_ctrl->rc.ch0=(dbus_buf[0]|(dbus_buf[1]<<8))&0x07ff;
	rc_ctrl->rc.ch1=((dbus_buf[1]>>3)|(dbus_buf[2]<<5))&0x7ff;
	rc_ctrl->rc.ch2=((dbus_buf[2]>>6)|(dbus_buf[3]<<2)|(dbus_buf[4]<<10))&0x07ff;
	rc_ctrl->rc.ch3=((dbus_buf[4]>>1)|(dbus_buf[5]<<7))&0x07ff;

	rc_ctrl->rc.s1=((dbus_buf[5]>>4)&0x0003);
	rc_ctrl->rc.s2=((dbus_buf[5]>>4)&0x000C)>>2;

	rc_ctrl->mouse.x=dbus_buf[6]|(dbus_buf[7]<<8);
	rc_ctrl->mouse.y=dbus_buf[8]|(dbus_buf[9]<<8);
	rc_ctrl->mouse.z=dbus_buf[10]|(dbus_buf[11]<<8);
	rc_ctrl->mouse.press_1=dbus_buf[12];
	rc_ctrl->mouse.press_r=dbus_buf[13];

	rc_ctrl->v=dbus_buf[14]|(dbus_buf[15]<<8);
	rc_ctrl->rc.ch4=dbus_buf[16]|(dbus_buf[17]<<8);

	//让它开始为0
	rc_ctrl->rc.ch0-=RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch1-=RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch2-=RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch3-=RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch4-=RC_CH_VALUE_OFFSET;


	RC_data_is_error();//用来进行出错处理


}


//遥控器数据出错处理
uint8_t RC_data_is_error(void)
{
	//用goto语句，方便统一处理遥控器变量数据归0
	if(RC_abs(rc_ctrl.rc.ch0)>RC_CHANNAL_ERROR_VALUE)
	{
		goto error;//goto表示跳转到error里面
	}
	if(RC_abs(rc_ctrl.rc.ch1)>RC_CHANNAL_ERROR_VALUE)
	{
		goto error;//goto表示跳转到error里面
	}
	if(RC_abs(rc_ctrl.rc.ch2)>RC_CHANNAL_ERROR_VALUE)
	{
		goto error;//goto表示跳转到error里面
	}
	if(RC_abs(rc_ctrl.rc.ch3)>RC_CHANNAL_ERROR_VALUE)
	{
		goto error;//goto表示跳转到error里面
	}
//遥控器开关值  1-3，没有0
	if(rc_ctrl.rc.s1==0)
	{
		goto error;
	}
	if(rc_ctrl.rc.s2==0)
	{
		goto error;
	}
	return 0;



	error:rc_ctrl.rc.ch0=0;
	rc_ctrl.rc.ch1=0;
	rc_ctrl.rc.ch2=0;
	rc_ctrl.rc.ch3=0;
	rc_ctrl.rc.ch4=0;
	rc_ctrl.rc.s1=RC_SW_DOWN;
	rc_ctrl.rc.s2=RC_SW_DOWN;
	rc_ctrl.mouse.x=0;
	rc_ctrl.mouse.y=0;
	rc_ctrl.mouse.z=0;
	rc_ctrl.mouse.press_1=0;
	rc_ctrl.mouse.press_r=0;
	rc_ctrl.v=0;
	return 1;//返回值成功
}


static int16_t RC_abs(int16_t value)//取绝对值  //static 定义在文件内部，不能被其他文件所访问
{
	if(value>0)
	{
		return value;
	}
	else
	{
		return -value;
	}
}


HAL_StatusTypeDef c;
void uart_dma_init()
{
	memset(DBUS_rx_buf,0,36);//将前36个字节清零
	c=HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)&DBUS_rx_buf[0],36);
}
