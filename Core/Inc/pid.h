#ifndef __PID_H_
#define __PID_H_

#include "main.h" // 包含你的主头文件
#include "stm32f4xx.h" // 包含 STM32 的头文件

// 电机 PID 控制器结构体
typedef struct _motor_pid_t
{
    float kp;       // 比例系数
    float ki;       // 积分系数
    float kd;       // 微分系数
    float i_max;    // 积分限幅
    float out_max;  // 输出限幅

    float ref;      // 目标速度 (RPM)
    float fdb;      // 反馈速度 (RPM)
    float err[2];   // 当前误差和上次误差

    float p_out;    // 比例输出
    float i_out;    // 积分输出
    float d_out;    // 微分输出
    float output;   // PID 总输出
} motor_pid_t;

// PID 初始化函数
void pid_init(motor_pid_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

// PID 计算函数
float pid_calc(motor_pid_t *pid, float ref, float fdb);

// 限制函数，将 x 限制在 min 和 max 之间
#define LIMIT_MIN_MAX(x, min, max) \
    if (x < min) {                 \
        x = min;                     \
    } else if (x > max) {          \
        x = max;                     \
    }

#endif
