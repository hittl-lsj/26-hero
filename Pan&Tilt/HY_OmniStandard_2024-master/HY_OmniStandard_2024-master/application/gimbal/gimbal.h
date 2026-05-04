/*
 * @Author: 曾令誉 18365412404@163.com
 * @Date: 2025-03-01 19:37:56
 * @LastEditors: 曾令誉 18365412404@163.com
 * @LastEditTime: 2025-03-11 13:04:18
 * @FilePath: \HY_OmniStandard_2024-master\application\gimbal\gimbal.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef GIMBAL_H
#define GIMBAL_H


/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H