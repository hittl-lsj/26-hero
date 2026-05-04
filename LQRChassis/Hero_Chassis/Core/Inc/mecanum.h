/*
 * mecanum.h
 *
 *  Created on: Aug 21, 2024
 *      Author: yu
 */

#ifndef INC_MECANUM_H_
#define INC_MECANUM_H_
extern float vr1,vr2,vr3,vr4;
//轮子到底盘中心的距离
#define LENGTH_A 0
#define LENGTH_B 0.4
typedef struct
{
    float vx;
    float vy;
    float vw;
} Chassis_Speed;

extern float L1,L2,R1,R2;
void mecanum_calc(Chassis_Speed *speed, int16_t *out_speed);
void mecanum_Set_Motor_Speed(int16_t *out_speed);
#endif /* INC_MECANUM_H_ */
