在原来框架基础上
将pid传参放入DMMotorTask
在.h文件将
typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
}DMMotorInstance;改写
使用框架提供的电机控制器
Motor_Controller_s motor_controller;    // 电机控制器
初次使用时除正常注册电机外需在
application\robot_task.h的OSTaskInit（）加入DMMotorControlInit();
在 DMMotorControlInit();调用osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 256);加入DMMotortast  