#ifndef LQR_SPEED_CONTROLLER_H
#define LQR_SPEED_CONTROLLER_H


#include <stdint.h>

// LQR速度控制器结构体
typedef struct {
    // 配置参数
    float k_speed;          // 速度反馈增益
    float k_integral;       // 积分反馈增益
    float max_output;       // 最大输出限制
    float max_integral;     // 积分限幅值
    
    // 运行状态
    float integral;         // 积分项累积
    float prev_error;       // 上一次误差
    uint32_t update_time;   // 上次更新时间(ms)
} LQR_SpeedController;

/**
 * @brief 初始化LQR速度控制器
 * @param ctrl 控制器指针
 * @param k_speed 速度增益
 * @param k_integral 积分增益
 * @param max_output 最大输出限制
 * @param max_integral 积分限幅
 */
void lqr_speed_init(LQR_SpeedController* ctrl, 
                   float k_speed, float k_integral,
                   float max_output, float max_integral);

/**
 * @brief 更新LQR速度控制计算(需按固定周期调用)
 * @param ctrl 控制器指针
 * @param target_speed 目标速度
 * @param current_speed 当前速度
 * @param delta_time 距离上次调用的时间间隔(秒)
 * @return 控制输出值
 */
float lqr_speed_update(LQR_SpeedController* ctrl, 
                      float target_speed, float current_speed,
                      float delta_time);

/**
 * @brief 自动计算时间间隔的更新函数(推荐使用)
 * @param ctrl 控制器指针
 * @param target_speed 目标速度
 * @param current_speed 当前速度
 * @param current_time 当前时间戳(毫秒)
 * @return 控制输出值
 */
float lqr_speed_update_auto(LQR_SpeedController* ctrl,
                           float target_speed, float current_speed,
                           uint32_t current_time);

/**
 * @brief 重置控制器状态(积分清零)
 * @param ctrl 控制器指针
 */
void lqr_speed_reset(LQR_SpeedController* ctrl);

/**
 * @brief 设置控制器参数
 * @param ctrl 控制器指针
 * @param k_speed 速度增益
 * @param k_integral 积分增益
 */
void lqr_speed_set_gains(LQR_SpeedController* ctrl, 
                        float k_speed, float k_integral);

/**
 * @brief 设置输出和积分限幅
 * @param ctrl 控制器指针
 * @param max_output 最大输出
 * @param max_integral 最大积分
 */
void lqr_speed_set_limits(LQR_SpeedController* ctrl,
                         float max_output, float max_integral);

#ifdef __cplusplus
}
#endif

#endif // LQR_SPEED_CONTROLLER_H