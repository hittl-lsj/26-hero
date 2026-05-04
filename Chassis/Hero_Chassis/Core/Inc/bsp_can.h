/*
 * bsp_can.h
 *
 *  Created on: Jun 3, 2024
 *      Author: yu
 */

#ifndef INC_BSP_CAN_H_
#define INC_BSP_CAN_H_




#define MotorParameter//定一个结构体装电机参数
#include "can.h"//


/**
 * @brief 电机数据结构体
 * @details 用于存储电机的状态信息和控制参数，包含电机反馈数据和控制数据
 */
typedef struct
{
    /**
     * @brief 电机角度
     * @details 电机当前机械角度，范围通常为0-8191（12位编码器）
     */
    int16_t angle;            
    
    /**
     * @brief 电机速度
     * @details 电机当前转速，单位通常为RPM或编码器脉冲数/秒
     */
    int16_t speeed;           
    
    /**
     * @brief 电机转矩
     * @details 电机当前输出转矩或力矩电流
     */
    int16_t torque;           
    
    /**
     * @brief 电机温度
     * @details 电机当前温度，单位为摄氏度
     */
    int16_t temp;             

    /**
     * @brief 电机多圈角度
     * @details 电机累计多圈绝对角度，考虑了圈数的角度值
     */
    int32_t angle_multiround; 
    
    /**
     * @brief 电机圈数计数
     * @details 记录电机转过的完整圈数，用于计算多圈角度
     */
    int32_t round_cnt;        
    
    /**
     * @brief 实际转矩电流
     * @details 电机当前实际的转矩电流值
     */
    int16_t Torque_current;   
    
    /**
     * @brief 转矩电流给定值
     * @details 期望输出给电机的目标转矩电流值
     */
    int16_t Torque_current_set;
    
    /**
     * @brief 霍尔传感器值
     * @details 电机霍尔传感器的反馈数据，用于初始定位或故障诊断
     */
    int16_t huoer;            

} Motor_t;                    // 电机数据结构体类型定义





void can_Init();
void boxSend1(int16_t I1,int16_t I2,int16_t I3,int16_t I4);
void boxSend2(int16_t I1,int16_t I2,int16_t I3,int16_t I4);
void motorset(Motor_t*Receive,uint8_t Data[]);
void boxReceive(void);
void angle_multiround_calc(uint8_t id);
void Null_Point(int32_t *D_value);
extern Motor_t Engine[6];


#endif /* INC_BSP_CAN_H_ */
