#include "mt_motor.h"
#include "memory.h"
#include "general_def.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
MTMotorInstance *mt_motor_instance[MT_MOTOR_CNT] = {NULL}; 
static osThreadId mt_task_handle[MT_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */

MTMotorInstance * Get_Motor(MTMotorInstance* mt_motor_instance)
{
   return mt_motor_instance;
}

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
// 将多圈角度转换为单圈角度（-π到π之间）
float multi_to_single_angle_pi(float multi_angle_rad) 
{
    const float two_pi = 2.0f * PI;
    float single_angle = fmodf(multi_angle_rad, two_pi);  
    if (single_angle > PI) 
    {
        single_angle -= two_pi;
    } else if (single_angle < -PI) 
    {
        single_angle += two_pi;
    } 
    return single_angle;
}
//在轮腿上用mit模式
static void MTMotorDecode(CANInstance *motor_can)
{

    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    MTMotorInstance *motor = (MTMotorInstance *)motor_can->id;
    MT_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    // can_id
    measure->id=rxbuff[0];
    //上次位置
    measure->last_position = measure->position;
    //当前位置
    tmp = (uint16_t)((rxbuff[1]<<8)|rxbuff[2]);
    measure->position = uint_to_float(tmp, MT_P_MIN, MT_P_MAX, 16);//弧度制
    //当前速度
    tmp=(uint16_t)((rxbuff[3]<<4) | rxbuff[4]>>4);
    measure->velocity = uint_to_float(tmp, MT_P_MIN, MT_P_MAX, 12);
    //当前转矩
    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);//先把rxbuff高四位置0
    measure->torque = uint_to_float(tmp, MT_P_MIN, MT_P_MAX, 12);

    measure->angle_single_round= multi_to_single_angle_pi(measure->position);
}

static void MTMotorLostCallback(void *motor_ptr)
{
}

MTMotorInstance *MTMotorInit(Motor_Init_Config_s *config,MTMotor_WorkMode_e work_mode)
{
    MTMotorInstance *motor = (MTMotorInstance *)malloc(sizeof(MTMotorInstance));
    memset(motor, 0, sizeof(MTMotorInstance));
    
    motor->work_mode=work_mode;
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = MTMotorDecode;
    config->can_init_config.id = motor;
    if(work_mode==MTMOTOR_MODE_MIT)
    {
     config->can_init_config.rx_id = config->can_init_config.tx_id +0x500;
     config->can_init_config.tx_id = 0x400 + config->can_init_config.tx_id; 
    }
    else if(work_mode==MTMOTOR_MODE_POS_VEL)
    {
     config->can_init_config.rx_id = config->can_init_config.tx_id +0x240;
     config->can_init_config.tx_id = 0x140 + config->can_init_config.tx_id; 
    }
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = MTMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,  //100ms
    };
    motor->motor_daemon = DaemonRegister(&conf);

    MTMotorEnable(motor);
    // MTMotorSetMode(MT_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    mt_motor_instance[idx++] = motor;
    return motor;
}

 void MTMotorTask(void const *argument)
{
 
    MTMotorInstance *motor = (MTMotorInstance *)argument;

    while (1)
    {

     switch (motor->work_mode)
     {
        case MTMOTOR_MODE_MIT:
        MTMotorMITControl(motor);
        break;
     
        case MTMOTOR_MODE_POS_VEL://不好用 不用了
        MTMotorPosControl(motor,motor->mt_send.POS_VEL.temp_position_des,3000);
        break;

     default:
        break;
     }
     
     MTMotorSend(motor);

     osDelay(2);

    }

}

void MTMotorSend(MTMotorInstance* motor)
{
    switch(motor->work_mode)
    {
        //MIT模式
       case MTMOTOR_MODE_MIT :
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor->mt_send.MIT.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor->mt_send.MIT.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor->mt_send.MIT.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor->mt_send.MIT.velocity_des & 0xF) << 4) | (motor->mt_send.MIT.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor->mt_send.MIT.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor->mt_send.MIT.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor->mt_send.MIT.Kd & 0xF) << 4) | (motor->mt_send.MIT.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor->mt_send.MIT.torque_des);
       break;

       //位置速度模式
       case MTMOTOR_MODE_POS_VEL :
       motor->motor_can_instace-> tx_buff[0] = (uint8_t)0xA4;
       motor->motor_can_instace-> tx_buff[1] = (uint8_t)0x00;
       motor->motor_can_instace-> tx_buff[2] = (uint8_t)(motor->mt_send.POS_VEL.velocity_des);
       motor->motor_can_instace-> tx_buff[3] = (uint8_t)(motor->mt_send.POS_VEL.velocity_des>>8);
       motor->motor_can_instace-> tx_buff[4] = (uint8_t)(motor->mt_send.POS_VEL.position_des*100);
       motor->motor_can_instace-> tx_buff[5] = (uint8_t)((motor->mt_send.POS_VEL.position_des*100)>>8);
       motor->motor_can_instace-> tx_buff[6] = (uint8_t)((motor->mt_send.POS_VEL.position_des*100)>>16);
       motor->motor_can_instace-> tx_buff[7] = (uint8_t)((motor->mt_send.POS_VEL.position_des*100)>>24);
       break;
    
  default:
        break;
    }

     CANTransmit(motor->motor_can_instace, 1);

}




void MTMotorControlInit()
{
    char mt_task_name[5] = "mt";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char mt_id_buff[2] = {0};
        __itoa(i, mt_id_buff, 10);
        strcat(mt_task_name, mt_id_buff);
        osThreadDef(mt_task_name, MTMotorTask, osPriorityNormal, 0, 128);
        mt_task_handle[i] = osThreadCreate(osThread(mt_task_name), mt_motor_instance[i]);
    }
}

void MTMotorSetRef(MTMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void MTMotorEnable(MTMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void MTMotorStop(MTMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void MTMotorOuterLoop(MTMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void MTMotorPosControl(MTMotorInstance *motor,int32_t pos, uint16_t velocity)
{

    motor->mt_send.POS_VEL.position_des =(int32_t)pos;
    motor->mt_send.POS_VEL.velocity_des =(uint16_t)velocity;

    if(motor->stop_flag == MOTOR_STOP)
    motor->mt_send.POS_VEL.velocity_des = 0;

}

void MTMotorMITControl(MTMotorInstance *motor)
{
    // float  pid_ref, set;
    // Motor_Control_Setting_s *setting= &motor->motor_settings;; // 电机控制参数

    // pid_ref = motor->pid_ref;
        

    float pid_measure,pid_ref,set;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    MT_Motor_Measure_s *measure;           // 电机测量值

    motor_setting = &(motor->motor_settings);
    motor_controller = &motor->motor_controller;
    measure = &motor->measure;
    pid_ref = motor_controller->pid_ref;

    // pid_ref会顺次通过被启用的闭环充当数据的载体

    // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
    if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
    {
        if (motor_setting->angle_feedback_source == OTHER_FEED)
            pid_measure = *motor_controller->other_angle_feedback_ptr;
        else
        {
            pid_measure = measure->position;
            if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                pid_measure *= -1;
        }
        // 更新pid_ref进入下一个环
        pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
    }

    // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
    if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
            pid_ref += *motor_controller->speed_feedforward_ptr;

        if (motor_setting->speed_feedback_source == OTHER_FEED)
            pid_measure = *motor_controller->other_speed_feedback_ptr;
        else // MOTOR_FEED
        {
            pid_measure = measure->velocity;
            if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                pid_measure *= -1;
        }

        // 更新pid_ref进入下一个环
        pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
    }

    
    set = pid_ref;
    if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    {
        set *= -1;
    }
       
        LIMIT_MIN_MAX(set, MT_T_MIN, MT_T_MAX);
        motor->mt_send.MIT.position_des = float_to_uint(0, MT_P_MIN, MT_P_MAX, 16);
        motor->mt_send.MIT.velocity_des = float_to_uint(0, MT_V_MIN, MT_V_MAX, 12);
        motor->mt_send.MIT.torque_des   = float_to_uint(pid_ref, MT_T_MIN, MT_T_MAX, 12);
        motor->mt_send.MIT.Kp = float_to_uint(0, 0, 500, 12);
        motor->mt_send.MIT.Kd = float_to_uint(0, 0, 5, 12);
        if(motor->stop_flag == MOTOR_STOP)
      {
         motor->mt_send.MIT.torque_des = float_to_uint(0, MT_T_MIN, MT_T_MAX, 12);
      }
}