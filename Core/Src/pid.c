#include "pid.h"

// PID 初始化函数
void pid_init(motor_pid_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->i_max   = i_max;
    pid->out_max = out_max;

    // 初始化误差
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->i_out = 0.0f; // 初始化积分输出
}

// PID 计算函数
float pid_calc(motor_pid_t *pid, float ref, float fdb)
{
    pid->ref = ref;         
    pid->fdb = fdb;          

    // 计算误差
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;

    // 计算 PID 各项输出
    pid->p_out  = pid->kp * pid->err[0];
    pid->i_out += pid->ki * pid->err[0];
    pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);

    // 积分限幅
    LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

    // 计算总输出
    pid->output = pid->p_out + pid->i_out + pid->d_out;

    // 输出限幅
    LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);

    return pid->output;
}
