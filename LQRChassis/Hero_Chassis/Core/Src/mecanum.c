/*
 * mecanum.c
 *
 *  Created on: Aug 21, 2024
 *      Author: yu
 */
#include "main.h"
#include "gpio.h"
#include "mecanum.h"

#define WHEEL_PERIMETER 0.4
#define CHASSIS_DECELE_RATIO 19
extern  l_up_real;
extern R__up_real;
//将底盘的线速度和角速度转换为四个麦克纳姆轮的实际转速
void mecanum_calc(Chassis_Speed *speed, int16_t *out_speed)
{
    int16_t wheel_rpm[4];   //int16
    float wheel_rpm_ratio;
//计算轮子转速比例因子   //WHEEL_PERIMETER 轮子的周长 //CHASSIS_DECELE_RATIO 减速比
    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f);//转换单位m/s变为转每
    
//麦克纳姆轮的运动学模型
    wheel_rpm[0] = (-speed->vx + speed->vy + speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[1] = (speed->vx + speed->vy + speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[2] = (speed->vx - speed->vy + speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[3] = (-speed->vx - speed->vy + speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;

			//将wheel_rpm复制到out_speed
			    memcpy(out_speed, wheel_rpm, 4 * sizeof(int16_t));

}

//将out_speed里面的给到motor里面
void mecanum_Set_Motor_Speed(int16_t *out_speed)
{
    L1 = out_speed[0];
    L2 = out_speed[1];
    R1 = out_speed[2];
    R2 = out_speed[3];

}




