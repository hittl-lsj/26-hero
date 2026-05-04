#include "LQR.h"
#include <math.h>

// 内部函数声明
static float anti_windup(float integral, float max_integral);
static float output_limit(float output, float max_output);
// 初始化LQR速度控制器
void lqr_speed_init(LQR_SpeedController* ctrl, 
                   float k_speed, float k_integral,
                   float max_output, float max_integral) {
    if (ctrl == NULL) return;
    // 初始化参数
    ctrl->k_speed = k_speed;
    ctrl->k_integral = k_integral;
    ctrl->max_output = max_output;
    ctrl->max_integral = max_integral;
    ctrl->integral = 0.0f;
    ctrl->prev_error = 0.0f;
    ctrl->update_time = 0;
}
// 更新LQR速度控制计算(需按固定周期调用)

float   lqr_speed_update(LQR_SpeedController* ctrl, 
                      float target_speed, float current_speed,
                      float delta_time) {
    if (ctrl == NULL || delta_time <= 0.0f) {
        return 0.0f;
    }
    
    // 计算速度误差
    float error = target_speed - current_speed;
    
    // 积分项更新(梯形积分提高精度)
    ctrl->integral += (error + ctrl->prev_error) * delta_time * 0.5f;
    ctrl->prev_error = error;
    
    // 抗积分饱和
    ctrl->integral = anti_windup(ctrl->integral, ctrl->max_integral);
    
    // LQR控制律: u = k_speed * error + k_integral * integral
    float control_output = ctrl->k_speed * error + ctrl->k_integral * ctrl->integral;
    
    // 输出限幅
    return output_limit(control_output, ctrl->max_output);
}
// 自动更新函数(推荐使用)
float lqr_speed_update_auto(LQR_SpeedController* ctrl,
                           float target_speed, float current_speed,
                           uint32_t current_time) {
    if (ctrl == NULL) return 0.0f;
    
    float delta_time = 0.01f; // 默认10ms
    
    if (ctrl->update_time > 0) {
        // 计算实际时间间隔(秒)
        delta_time = (current_time - ctrl->update_time) * 0.001f;
        // 限制时间间隔在合理范围内(1ms ~ 100ms)
        if (delta_time < 0.001f) delta_time = 0.001f;
        if (delta_time > 0.1f) delta_time = 0.1f;
    }
    
    ctrl->update_time = current_time;
    return lqr_speed_update(ctrl, target_speed, current_speed, delta_time);
}
// 重置控制器状态(积分清零)
void lqr_speed_reset(LQR_SpeedController* ctrl) {
    if (ctrl == NULL) return;
    
    ctrl->integral = 0.0f;
    ctrl->prev_error = 0.0f;
    ctrl->update_time = 0;
}
// 设置控制器参数
void lqr_speed_set_gains(LQR_SpeedController* ctrl, 
                        float k_speed, float k_integral) {
    if (ctrl == NULL) return;
    
    ctrl->k_speed = k_speed;
    ctrl->k_integral = k_integral;
}
// 设置输出限幅和积分限幅
void lqr_speed_set_limits(LQR_SpeedController* ctrl,
                         float max_output, float max_integral) {
    if (ctrl == NULL) return;
    
    ctrl->max_output = max_output;
    ctrl->max_integral = max_integral;
    
    // 立即应用新的积分限幅
    ctrl->integral = anti_windup(ctrl->integral, max_integral);
}

// 内部函数实现
static float anti_windup(float integral, float max_integral) {
    if (integral > max_integral) return max_integral;
    if (integral < -max_integral) return -max_integral;
    return integral;
}
// 输出限幅
static float output_limit(float output, float max_output) {
    if (output > max_output) return max_output;
    if (output < -max_output) return -max_output;
    return output;
}