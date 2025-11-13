/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* DM4340 motor parameters */
/* Math constants */
#define PI 3.141592653589793f
#define PI_2 6.283185307179586f

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    /* DM4340 motor IDs */
    DM4340_M1_MATER  = 0x01,
    DM4340_M2_MATER  = 0x02,
    DM4340_M3_MATER  = 0x03,
    DM4340_M1 = 0x11,
    DM4340_M2 = 0x12,
    DM4340_M3 = 0x13,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;           //转子机械角度
    int16_t speed_rpm;      //转子转速
    int16_t given_current;  //实际转矩电流
    uint8_t temperate;      //电机温度
    int16_t last_ecd;       //上次转子机械角度
} motor_measure_t;


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000]
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_chassis1(int16_t motor1, int16_t motor2);
extern void CAN_cmd_chassis2(int16_t motor3, int16_t motor4);
/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/* DM4340电机数据结构 (从vb_3wheel迁移) */
typedef struct 
{
    int id;                // 帧ID
    int state;             // 状态

    float f_kp;            // 位置环增益
    float f_p;             // 位置环偏差
    float f_kd;            // 速度环增益
    float f_v;            // 速度环偏差
    float f_t;            // 转矩

    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float esc_back_position; // 返回的位置
    float esc_back_speed;    // 反馈速度
    float esc_back_current;  // 反馈电流/扭矩
    float esc_back_angle;    // 反馈角度
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;

    float esc_back_position_last; // 上一次返回的位置
    int64_t circle_num;           // 旋转圈数
    float serial_position;        // 总码盘值
    float serial_angle;           // 总角度
    float serial_angle_last;      // 上一次的总角度
    float real_angle;             // 上电零点第一次反馈位置esc_back_position

    float target_speed; // 目标速度
    float set_speed;    // 设置速度
    double target_position; // 目标位置
    float target_angle;     // 目标角度
    int target_state;

    float out_current; // 输出电流
} s_motor_data_t;

// 达妙电机控制函数

/**
  * @brief          control motor enable/disable with specified CAN ID and mode offset
  * @param[in]      can_id: motor CAN ID
  * @param[in]      mode_offset: mode offset ID (0x00, 0x100, 0x200, 0x300)
  * @param[in]      enable: True to enable, False to disable
  * @retval         none
  */
/**
  * @brief          控制电机使能/失能
  * @param[in]      can_id: 电机CAN ID
  * @param[in]      mode_offset: 模式偏移ID (0x00, 0x100, 0x200, 0x300)
  * @param[in]      enable: True为使能，False为失能
  * @retval         none
  */
extern void CAN_cmd_motor_control(uint16_t can_id, uint16_t mode_offset, bool enable);
extern void CAN_cmd_motor_set_zero(uint16_t can_id);
/**
  * @brief          control motor position and velocity in position-velocity mode
  * @param[in]      can_id: motor CAN ID
  * @param[in]      pos: target position (float, little-endian)
  * @param[in]      vel: target velocity (float, little-endian)
  * @retval         none
  */
/**
  * @brief          位置速度模式控制电机
  * @param[in]      can_id: 电机CAN ID
  * @param[in]      pos: 目标位置(浮点数，小端序)
  * @param[in]      vel: 目标速度(浮点数，小端序)
  * @retval         none
  */
extern void CAN_cmd_motor_pos_vel_control( float pos, float vel);

/* DM4340 motor control functions */
extern void MD4340_motor_PID_Control(uint32_t id, float torq);
extern s_motor_data_t DM4340_Date[3]; // DM4340电机数据数组声明
extern void MD_CanReceive(s_motor_data_t *motor, uint8_t RxDate[8]);

/* 封装的DM4340控制接口 */
void DM4340_Control_Init(uint8_t motor_id);  // 初始化PID参数
void DM4340_Set_Target_Angle(uint8_t motor_id, float angle); // 设置目标角度
void DM4340_Control_Loop(uint8_t motor_id);  // 执行控制循环

#endif
