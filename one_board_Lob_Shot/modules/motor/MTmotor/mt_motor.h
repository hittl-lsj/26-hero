#ifndef MTMOTOR_H
#define MTMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"


#define MT_MOTOR_CNT 4

#define MT_P_MIN  (-12.5f)
#define MT_P_MAX  12.5f
#define MT_V_MIN  (-45.0f)
#define MT_V_MAX  45.0f
#define MT_T_MIN  (-24.0f)
#define MT_T_MAX   24.0f


/* MT电机CAN反馈信息*/
typedef struct
{
    uint8_t id;          //can_id 
    float last_position;  
    float position;     //当前角度（rad）
    float velocity;        // 转速（rad/s）
    float torque;        // 转矩（N/m）
    float angle_single_round; // 输出轴单圈角度  
} MT_Motor_Measure_s;

typedef enum
{
    MTMOTOR_MODE_MIT = 1,
    MTMOTOR_MODE_POS_VEL 
}MTMotor_WorkMode_e;

//MT电机发送数据
typedef struct
{
    struct
    {
        uint16_t position_des;
        uint16_t velocity_des;
        uint16_t torque_des;
        uint16_t Kp;
        uint16_t Kd;
    }MIT;
    struct
    {
        int32_t position_des;
        uint16_t velocity_des;
        int32_t temp_position_des;
    }POS_VEL;
}MTMotor_Send_s;


typedef struct
{
    MTMotor_WorkMode_e work_mode;
    MTMotor_Send_s mt_send;       // 发送数据缓存区
    MT_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;    // 电机控制器

    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    __uint32_t lost_cnt;


} MTMotorInstance;



MTMotorInstance *MTMotorInit(Motor_Init_Config_s *config,MTMotor_WorkMode_e work_mode);
void MTMotorSetRef(MTMotorInstance *motor, float ref);
void MTMotorEnable(MTMotorInstance *motor);
void MTMotorStop(MTMotorInstance *motor);//不使用使能模式是因为需要收到反馈
void MTMotorOuterLoop(MTMotorInstance *motor, Closeloop_Type_e type);
void MTMotorControlInit();
extern  MTMotorInstance *mt_motor_instance[MT_MOTOR_CNT]; 
float multi_to_single_angle_pi(float multi_angle_rad);
MTMotorInstance * Get_Motor(MTMotorInstance* mt_motor_instance);
void MTMotorMITControl(MTMotorInstance *motor);
void MTMotorPosControl(MTMotorInstance *motor,int32_t pos, uint16_t velocity);
void MTMotorSend(MTMotorInstance* motor);
#endif