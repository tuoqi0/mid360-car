#ifndef GOAL_RECEIVER_H
#define GOAL_RECEIVER_H

#include <stdint.h>
#include "pid.h"

// 参数定义 - 根据差速控制调整
#define KP_FORWARD          2.0f    // 前进控制增益
#define KP_YAW             1.5f    // 转向控制增益
#define MAX_LINEAR_SPEED   1.0f    // 最大线速度 m/s
#define MAX_ANGULAR_SPEED  2.0f    // 最大角速度 rad/s
#define WHEEL_RADIUS       0.20f  // 车轮半径 m
#define GEAR_RATIO         1.0f    // 齿轮比
#define WHEEL_BASE         0.3f    // 轮距 m (左右轮之间的距离)

// 卡尔曼滤波参数
#define Q_ACCEL         0.001f    // 过程噪声 (加速度)
#define Q_BIAS          0.00001f  // 过程噪声 (零偏)
#define R_ACCEL_MEAS    0.01f     // 测量噪声 (加速度计)

// 目标信息结构体
typedef struct {
    float longitudinal_distance;  // 纵向距离 (米)
    float lateral_distance;       // 横向距离 (米) - 保留但不再使用
    float heading_error_deg;      // 航向误差 (度)
    uint8_t control_mode;         // 控制模式
    uint32_t received_count;      // 接收计数
    uint32_t error_count;         // 错误计数
} GoalInfo_t;

// 函数声明
void GoalReceiver_Init(void);
void GoalReceiver_ProcessData(uint8_t* data, uint32_t length);
void GoalReceiver_MotorControlUpdate(float* target_speeds, float* gyro_data, uint32_t dt_ms);
void GoalReceiver_MotorControlUpdate_WithDeceleration(float* target_speeds, const float* gyro_data, const uint32_t dt_ms);
void GoalReceiver_KalmanFilterUpdate(const float* accel_data, float* true_accel, float* bias);
void GoalReceiver_GetDecelerationInfo(float* decel_forward, float* decel_lateral, float* decel_angle);
void GoalReceiver_SetDecelerationParams(float decel_distance, float stop_distance, float min_speed);
GoalInfo_t* GoalReceiver_GetGoalInfo(void);
void GoalReceiver_ResetStats(void);
motor_pid_t* GoalReceiver_GetMotorPid(int motor_index);
motor_pid_t* GoalReceiver_GetWzPid(void);
void GoalReceiver_SetCurrentAngle(float angle);
void GoalReceiver_SetAngleError(float error);
float GoalReceiver_GetCurrentAngle(void);

#endif /* GOAL_RECEIVER_H */