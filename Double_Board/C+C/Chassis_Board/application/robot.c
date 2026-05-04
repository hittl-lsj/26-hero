/*
 * @Author: 曾令誉 18365412404@163.com
 * @Date: 2024-10-08 08:10:50
 * @LastEditors: 曾令誉 18365412404@163.com
 * @LastEditTime: 2025-02-12 15:31:56
 * @FilePath: \HY_OmniStandard_2024-master\application\robot.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#include "dmmotor.h"
#endif


void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDInit();
    GimbalInit();
    ShootInit();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
#endif

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

// void RobotTask()
// {
// #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
//     RobotCMDTask();
//     GimbalTask();
//     ShootTask();
//     // //加入运行达妙电机
//     // DMMotorTask();
// #endif

// #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
//     ChassisTask();
// #endif

// }
void RobotTask()
{
    static uint32_t robot_counter;

    if(robot_counter%1==0)
    {
        //500hz
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
        ChassisTask();
#endif
    }

    if(robot_counter%2==0)
    {
        //250hz
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
        RobotCMDTask();
        GimbalTask();
        ShootTask();
#endif
    }

    robot_counter++;

}