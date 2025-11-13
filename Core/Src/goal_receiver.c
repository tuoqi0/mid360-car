#include "goal_receiver.h"
#include <string.h>
#include <math.h>

// 数据帧结构
#pragma pack(push, 1)
typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t data_length;
    uint8_t command;
    int16_t forward;
    int16_t lateral;
    int16_t yaw;
    uint8_t mode;
    uint8_t checksum;
    uint8_t footer1;
    uint8_t footer2;
} GoalFrame_t;
#pragma pack(pop)

// 状态机状态
typedef enum {
    STATE_HEADER1,
    STATE_HEADER2,
    STATE_LENGTH,
    STATE_COMMAND,
    STATE_FORWARD_H,
    STATE_FORWARD_L,
    STATE_LATERAL_H,
    STATE_LATERAL_L,
    STATE_YAW_H,
    STATE_YAW_L,
    STATE_MODE,
    STATE_CHECKSUM,
    STATE_FOOTER1,
    STATE_FOOTER2
} ParserState_t;

// 全局变量
static ParserState_t parser_state = STATE_HEADER1;
static GoalFrame_t current_frame;
static uint8_t data_index = 0;
static GoalInfo_t goal_info = {0};

// PID控制器
static motor_pid_t motor_pids[4];  // 四个电机的PID控制器
static motor_pid_t wz_pid;         // Wz轴PID控制器
static motor_pid_t pos_x_pid, pos_y_pid;  // 位置环PID

// 卡尔曼滤波状态
static float P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}}; // 初始状态协方差矩阵

// Wz控制相关变量
static float current_angle = 0.0f;     // 当前方向角度
static float current_angle_err = 0.0f; // 零飘
static float target_wz = 0.0f;         // 目标Wz值

// 减速控制参数
static  float   DECELERATION_DISTANCE = 0.5f;  // 0.5米开始减速
static  float STOP_DISTANCE = 0.05f;         // 0.05米停止
static  float MIN_SPEED_SCALE = 0.1f;        // 最小速度比例

// 计算校验和
uint8_t GoalReceiver_CalculateChecksum(const uint8_t* data, const uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

// 卡尔曼滤波更新 (单轴) -
void GoalReceiver_KalmanFilterUpdate(const float* accel_data, float* true_accel, float* bias) {
    for (int axis = 0; axis < 3; axis++) {
        float measured_accel = accel_data[axis];
        float *true_accel_ptr = &true_accel[axis];
        float *bias_ptr = &bias[axis];

        // 1. 预测步骤
        float F[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}}; // 状态转移矩阵
        float x_hat[2] = {*true_accel_ptr, *bias_ptr}; // 状态向量

        // 预测状态
        float x_hat_predicted[2];
        x_hat_predicted[0] = F[0][0] * x_hat[0] + F[0][1] * x_hat[1];
        x_hat_predicted[1] = F[1][0] * x_hat[0] + F[1][1] * x_hat[1];

        // 预测协方差矩阵
        float P_predicted[2][2];
        P_predicted[0][0] = F[0][0] * P[0][0] * F[0][0] + F[0][1] * P[1][0] * F[0][0] +
                           F[0][0] * P[0][1] * F[1][0] + F[0][1] * P[1][1] * F[1][0] + Q_ACCEL;
        P_predicted[0][1] = F[0][0] * P[0][0] * F[0][1] + F[0][1] * P[1][0] * F[0][1] +
                           F[0][0] * P[0][1] * F[1][1] + F[0][1] * P[1][1] * F[1][1];
        P_predicted[1][0] = F[1][0] * P[0][0] * F[0][0] + F[1][1] * P[1][0] * F[0][0] +
                           F[1][0] * P[0][1] * F[1][0] + F[1][1] * P[1][1] * F[1][0];
        P_predicted[1][1] = F[1][0] * P[0][0] * F[0][1] + F[1][1] * P[1][0] * F[0][1] +
                           F[1][0] * P[0][1] * F[1][1] + F[1][1] * P[1][1] * F[1][1] + Q_BIAS;

        // 2. 更新步骤
        float H[1][2] = {{1.0f, 1.0f}}; // 测量矩阵
        float z = measured_accel; // 测量值

        // 计算残差
        float y = z - (H[0][0] * x_hat_predicted[0] + H[0][1] * x_hat_predicted[1]);

        // 计算残差协方差
        float S = H[0][0] * P_predicted[0][0] * H[0][0] + H[0][1] * P_predicted[1][0] * H[0][0] +
                 H[0][0] * P_predicted[0][1] * H[0][1] + H[0][1] * P_predicted[1][1] * H[0][1] + R_ACCEL_MEAS;

        // 计算卡尔曼增益
        float K[2];
        K[0] = (P_predicted[0][0] * H[0][0] + P_predicted[0][1] * H[0][1]) / S;
        K[1] = (P_predicted[1][0] * H[0][0] + P_predicted[1][1] * H[0][1]) / S;

        // 更新状态估计
        *true_accel_ptr = x_hat_predicted[0] + K[0] * y;
        *bias_ptr = x_hat_predicted[1] + K[1] * y;

        // 更新协方差矩阵
        P[0][0] = (1 - K[0] * H[0][0]) * P_predicted[0][0] - K[0] * H[0][1] * P_predicted[1][0];
        P[0][1] = (1 - K[0] * H[0][0]) * P_predicted[0][1] - K[0] * H[0][1] * P_predicted[1][1];
        P[1][0] = -K[1] * H[0][0] * P_predicted[0][0] + (1 - K[1] * H[0][1]) * P_predicted[1][0];
        P[1][1] = -K[1] * H[0][0] * P_predicted[0][1] + (1 - K[1] * H[0][1]) * P_predicted[1][1];
    }
}

// 差速轮运动学计算 - 修改为差速控制
static void calculate_differential_wheels(const float vx, const float omega, float* wheels) {
    // 差速控制计算
    // 基础速度 = 前进后退控制
    float base_speed = vx;

    // 转向控制 = 旋转控制
    float turn_speed = omega * (WHEEL_BASE / 2.0f);

    // 左右轮速度计算
    // 轮子顺序: 左前(0), 右前(1), 左后(2), 右后(3)
    wheels[0] = base_speed - turn_speed;   // 左前轮
    wheels[1] = base_speed + turn_speed;   // 右前轮
    wheels[2] = base_speed - turn_speed;   // 左后轮
    wheels[3] = base_speed + turn_speed;   // 右后轮
}

// 计算减速因子（基于距离）
static float calculate_deceleration_factor(float distance) {
    float abs_distance = fabsf(distance);

    if (abs_distance <= STOP_DISTANCE) {
        // 在停止距离内，速度为0
        return 0.0f;
    } else if (abs_distance <= DECELERATION_DISTANCE) {
        // 在减速区域内，线性减速
        float factor = (abs_distance - STOP_DISTANCE) / (DECELERATION_DISTANCE - STOP_DISTANCE);
        // 确保最小速度，避免卡在目标点附近
        return fmaxf(factor, MIN_SPEED_SCALE);
    } else {
        // 在减速区域外，全速
        return 1.0f;
    }
}

// 处理完整数据帧
static void process_complete_frame(GoalFrame_t* frame) {
    // 单位转换：厘米->米，0.01度->度
    goal_info.longitudinal_distance = frame->forward * 0.01f;
    goal_info.lateral_distance = frame->lateral * 0.01f;
    goal_info.heading_error_deg = frame->yaw * 0.01f;
    goal_info.control_mode = frame->mode;
    goal_info.received_count++;
}

// 数据接收处理
void GoalReceiver_ProcessData(uint8_t* data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = data[i];

        switch (parser_state) {
            case STATE_HEADER1:
                if (byte == 0xAA) {
                    parser_state = STATE_HEADER2;
                }
                break;

            case STATE_HEADER2:
                if (byte == 0x55) {
                    parser_state = STATE_LENGTH;
                } else {
                    parser_state = STATE_HEADER1;
                    goal_info.error_count++;
                }
                break;

            case STATE_LENGTH:
                if (byte == 8) {
                    current_frame.data_length = byte;
                    parser_state = STATE_COMMAND;
                } else {
                    parser_state = STATE_HEADER1;
                    goal_info.error_count++;
                }
                break;

            case STATE_COMMAND:
                if (byte == 0x01) {
                    current_frame.command = byte;
                    parser_state = STATE_FORWARD_H;
                } else {
                    parser_state = STATE_HEADER1;
                    goal_info.error_count++;
                }
                break;

            case STATE_FORWARD_H:
                ((uint8_t*)&current_frame.forward)[1] = byte;
                parser_state = STATE_FORWARD_L;
                break;

            case STATE_FORWARD_L:
                ((uint8_t*)&current_frame.forward)[0] = byte;
                parser_state = STATE_LATERAL_H;
                break;

            case STATE_LATERAL_H:
                ((uint8_t*)&current_frame.lateral)[1] = byte;
                parser_state = STATE_LATERAL_L;
                break;

            case STATE_LATERAL_L:
                ((uint8_t*)&current_frame.lateral)[0] = byte;
                parser_state = STATE_YAW_H;
                break;

            case STATE_YAW_H:
                ((uint8_t*)&current_frame.yaw)[1] = byte;
                parser_state = STATE_YAW_L;
                break;

            case STATE_YAW_L:
                ((uint8_t*)&current_frame.yaw)[0] = byte;
                parser_state = STATE_MODE;
                break;

            case STATE_MODE:
                current_frame.mode = byte;
                parser_state = STATE_CHECKSUM;
                break;

            case STATE_CHECKSUM:
                current_frame.checksum = byte;
                parser_state = STATE_FOOTER1;
                break;

            case STATE_FOOTER1:
                if (byte == 0x0D) {
                    current_frame.footer1 = byte;
                    parser_state = STATE_FOOTER2;
                } else {
                    parser_state = STATE_HEADER1;
                    goal_info.error_count++;
                }
                break;

            case STATE_FOOTER2:
                if (byte == 0x0A) {
                    current_frame.footer2 = byte;

                    // 验证校验和
                    uint8_t calc_checksum = GoalReceiver_CalculateChecksum(
                        (uint8_t*)&current_frame.data_length, 8
                    );

                    if (calc_checksum == current_frame.checksum) {
                        process_complete_frame(&current_frame);
                    } else {
                        goal_info.error_count++;
                    }
                } else {
                    goal_info.error_count++;
                }
                parser_state = STATE_HEADER1;
                break;
        }
    }
}

// 原始电机控制更新（无减速）- 修改为差速控制
void GoalReceiver_MotorControlUpdate(float* target_speeds, float* gyro_data, uint32_t dt_ms) {
    // 将目标信息转换为底盘运动指令 - 去掉横向移动
    float vx = KP_FORWARD * goal_info.longitudinal_distance;
    float omega = KP_YAW * goal_info.heading_error_deg * 3.14159f / 180.0f;

    // Wz 控制
    if (dt_ms > 0) {
        float dt_sec = dt_ms / 1000.0f;
        current_angle += (gyro_data[2] * dt_sec) - (dt_sec * current_angle_err);
        float wz_compensation = pid_calc(&wz_pid, target_wz, current_angle);
        omega += wz_compensation;
    }

    // 限制最大速度
    vx = fmaxf(fminf(vx, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED);
    omega = fmaxf(fminf(omega, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED);

    // 计算四个轮子的速度 - 使用差速控制
    float wheel_speeds[4];
    calculate_differential_wheels(vx, omega, wheel_speeds);

    // 转换为电机RPM
    for (int i = 0; i < 4; i++) {
        target_speeds[i] = wheel_speeds[i] * 60.0f / (2 * 3.14159f * WHEEL_RADIUS) * GEAR_RATIO;
    }
}

// 改进的电机控制更新（带减速）- 修改为差速控制
void GoalReceiver_MotorControlUpdate_WithDeceleration(float* target_speeds, const float* gyro_data, const uint32_t dt_ms) {
    // 计算各个方向的减速因子 - 去掉横向移动减速
    float decel_forward = calculate_deceleration_factor(goal_info.longitudinal_distance);

    // 计算角度误差的减速因子（使用纵向距离作为参考）
    float decel_angle = calculate_deceleration_factor(fabsf(goal_info.longitudinal_distance));

    // 将目标信息转换为底盘运动指令，并应用减速因子 - 去掉横向移动
    float vx = KP_FORWARD * goal_info.longitudinal_distance * decel_forward;
    float omega = KP_YAW * goal_info.heading_error_deg * 3.14159f / 180.0f * decel_angle;

    // Wz 控制
    if (dt_ms > 0) {
        float dt_sec = dt_ms / 1000.0f;
        current_angle += (gyro_data[2] * dt_sec) - (dt_sec * current_angle_err);
        float wz_compensation = pid_calc(&wz_pid, target_wz, current_angle);
        omega += wz_compensation;
    }

    // 限制最大速度
    vx = fmaxf(fminf(vx, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED);
    omega = fmaxf(fminf(omega, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED);

    // 计算四个轮子的速度 - 使用差速控制
    float wheel_speeds[4];
    calculate_differential_wheels(vx, omega, wheel_speeds);

    // 转换为电机RPM
    for (int i = 0; i < 4; i++) {
        target_speeds[i] = wheel_speeds[i] * 60.0f / (2 * 3.14159f * WHEEL_RADIUS) * GEAR_RATIO;
    }
}

// 获取减速状态信息 - 修改为只包含前进和角度
void GoalReceiver_GetDecelerationInfo(float* decel_forward, float* decel_lateral, float* decel_angle) {
    if (decel_forward) *decel_forward = calculate_deceleration_factor(goal_info.longitudinal_distance);
    if (decel_lateral) *decel_lateral = 0.0f;  // 横向移动已禁用

    if (decel_angle) *decel_angle = calculate_deceleration_factor(fabsf(goal_info.longitudinal_distance));
}

// 设置减速参数
void GoalReceiver_SetDecelerationParams(const float decel_distance, float stop_distance, float min_speed) {
    DECELERATION_DISTANCE = decel_distance;
    STOP_DISTANCE = stop_distance;
    MIN_SPEED_SCALE = min_speed;
}

// 初始化函数
void GoalReceiver_Init(void) {
    parser_state = STATE_HEADER1;
    memset(&current_frame, 0, sizeof(current_frame));
    memset(&goal_info, 0, sizeof(goal_info));
    data_index = 0;

    // 重置卡尔曼滤波协方差矩阵
    P[0][0] = 1.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 1.0f;

    // 重置Wz控制变量
    current_angle = 0.0f;
    current_angle_err = 0.0f;
    target_wz = 0.0f;

    // 初始化四个底盘电机的 PID 控制器
    for (int i = 0; i < 4; i++) {
        pid_init(&motor_pids[i], 3.5f, 0.01f, 0.0f, 30000.0f, 16384.0f);
    }

    // Wz 正对方向pid控制
    pid_init(&wz_pid, 1.5f, 0.00f, 0.0f, 500.0f, 660.0f);

    // 位置环PID初始化
    pid_init(&pos_x_pid, 0.6f, 0.01f, 0.2f, 1000.0f, 300.0f); // X方向位置环
    pid_init(&pos_y_pid, 0.6f, 0.01f, 0.2f, 1000.0f, 300.0f); // Y方向位置环
}

// 获取目标信息
GoalInfo_t* GoalReceiver_GetGoalInfo(void) {
    return &goal_info;
}

// 重置统计信息
void GoalReceiver_ResetStats(void) {
    goal_info.received_count = 0;
    goal_info.error_count = 0;
}

// 获取电机PID控制器指针
motor_pid_t* GoalReceiver_GetMotorPid(int motor_index) {
    if (motor_index >= 0 && motor_index < 4) {
        return &motor_pids[motor_index];
    }
    return NULL;
}

// 获取Wz PID控制器指针
motor_pid_t* GoalReceiver_GetWzPid(void) {
    return &wz_pid;
}

// 设置Wz控制角度（用于校准）
void GoalReceiver_SetCurrentAngle(float angle) {
    current_angle = angle;
}

// 设置Wz零飘
void GoalReceiver_SetAngleError(float error) {
    current_angle_err = error;
}

// 获取当前角度
float GoalReceiver_GetCurrentAngle(void) {
    return current_angle;
}