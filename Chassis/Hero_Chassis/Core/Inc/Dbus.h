/*
 * Dbus.h
 *
 *  Created on: Jul 17, 2024
 *      Author: yu
 */

#ifndef INC_DBUS_H_
#define INC_DBUS_H_

#include "main.h"

#define RC_CH_VALUE_MIN ((uint16_t)364)//最小
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)//中间值
#define RC_CH_VALUE_MAX ((uint16_t)1684)//最大

#define RC_CHANNAL_ERROR_VALUE 700 //宏定义出错值为700，阈值

#define RC_SW_DOWN ((uint16_t)2)

#define RC_SW_UP ((uint16_t)1)//不知道干嘛的
#define RC_SW_MID ((uint16_t)3)//不知道干嘛的

#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

#define SBUS_RX_BUF_NUM 36u //无符号整数

typedef struct
{
	struct
	{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;

		int16_t ch4;//云台角度拨轮

		uint8_t s1;
		uint8_t s2;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;

		uint8_t press_1;
		uint8_t press_r;
	}mouse;

	union//各个成员共享一块空间
	{
		uint16_t v;
		struct
		{
			uint8_t key_w :1;//表示这个字段占一位的空间
			uint8_t key_s :1;
			uint8_t key_a :1;
			uint8_t key_d :1;
			uint8_t key_shift :1;
			uint8_t key_ctrl :1;
			uint8_t key_q :1;
			uint8_t key_e :1;
			uint8_t key_r :1;
			uint8_t key_f :1;
			uint8_t key_g :1;
			uint8_t key_z :1;
			uint8_t key_x :1;
			uint8_t key_c :1;
			uint8_t key_v :1;
			uint8_t key_b :1;
		}key;
	};
}RC_Ctrl_t;
extern uint8_t DBUS_rx_buf[SBUS_RX_BUF_NUM];

extern RC_Ctrl_t rc_ctrl;
extern uint16_t DR16_Received;
void dbus_to_rc(volatile const uint8_t *dbus_buf,RC_Ctrl_t *rc_ctrl);//volatile 表示重新读取，const 只读
static int16_t RC_abs(int16_t value);
uint8_t RC_data_is_error(void);
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart);
void uart_dma_init();
#endif /* INC_DBUS_H_ */
