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

#include "CAN_receive.h"
#include "main.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* 数值限制函数 */
static float constrain(float value, float min, float max)
{
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

// 帧率统计结构体
typedef struct {
    uint32_t DM4340_M1;
    uint32_t DM4340_M2;
    uint32_t DM4340_M3;
} FPS_t;

static FPS_t FPS = {0};

/* 无符号整型转浮点型 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
  * @brief          Convert float to unsigned int with range mapping
  * @param[in]      x: input value
  * @param[in]      x_min: minimum range
  * @param[in]      x_max: maximum range
  * @param[in]      bits: bit width
  * @retval         converted value
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x - x_min;
    return (uint16_t)((offset / span) * ((1 << bits) - 1));
}



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
//解码电机信息
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
// 电机反馈状态结构体
typedef struct {
    bool enabled;               // 电机是否启用
    bool feedback_received;     // 是否接收到反馈
    uint32_t last_send_time;    // 上一次发送时间
    float last_pos;             // 上一次位置
    float last_vel;             // 上一次速度
} motor_feedback_state_t;

// 电机反馈状态数组(支持3个电机)
static motor_feedback_state_t motor_feedback_states[3] = {0};

/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[7];
s_motor_data_t DM4340_Date[3]; // DM4340电机数据数组，支持3个电机

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header_can1, rx_header_can2;
    uint8_t rx_data_can1[8];
    uint8_t rx_data_can2[8];

    // CAN1接收数据处理
    if (hcan->Instance == CAN1){
      HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can1, rx_data_can1); // 从FIFO中接收消息至rx_header_can1

      switch (rx_header_can1.StdId)
      {
          case CAN_3508_M1_ID:
          case CAN_3508_M2_ID:
          case CAN_3508_M3_ID:
          case CAN_3508_M4_ID:
          case CAN_YAW_MOTOR_ID:
          case CAN_PIT_MOTOR_ID:
          case CAN_TRIGGER_MOTOR_ID:
          {
              static uint8_t i = 0;
              //get motor id
              i = rx_header_can1.StdId - CAN_3508_M1_ID;
              get_motor_measure(&motor_chassis[i], rx_data_can1);
              break;
          }
          default:
          {
              break;
          }
      }
    }
    
    ////////CAN2接收数据处理//////////
    if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header_can2, rx_data_can2); // 从FIFO中接收消息至rx_header_can2
        switch (rx_header_can2.StdId)
        {


        default:
        {
            break;
        }
        }
    }
}





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
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

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
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


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
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;

    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;

    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;

    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


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
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

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
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


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
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


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
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}



// 达妙电机控制函数


/// @brief 使用pid输出力矩的方式控制，即电流环控制
/// @param hcan can输出句柄
/// @param id HT电机的id号
/// @param _torq 输入的前馈力矩



/**
  * @brief          DM motor data receive
  * @param[in]      motor: motor data structure
  * @param[in]      RxDate: received CAN data
  * @retval         none
  */
/**
  * @brief          DM电机数据解包
  * @param[in]      motor: 电机数据结构体
  * @param[in]      RxData: 接收的CAN数据
  * @retval         none
  */

//    motor->id = (RxData[0])&0x0F;
//    motor->state = (RxData[0])>>4;
//    motor->p_int = (RxData[1]<<8)|RxData[2];
//    motor->v_int = (RxData[3]<<4)|(RxData[4]>>4);
//    motor->t_int = ((RxData[4]&0xF)<<8)|RxData[5];
//
//    // 位置解包 (-12.5,12.5)
//    motor->esc_back_position = uint_to_float(motor->p_int, DM4340_P_MIN, DM4340_P_MAX, 16);
//
//    // 速度解包 (-45.0,45.0)
//    motor->esc_back_speed = uint_to_float(motor->v_int, DM4340_V_MIN, DM4340_V_MAX, 12);
//
//    // 转矩解包 (-18.0,18.0)
//    motor->esc_back_current = uint_to_float(motor->t_int, DM4340_T_MIN, DM4340_T_MAX, 12);
//
//    // 温度解包
//    motor->Tmos = (float)(RxData[6]);
//    motor->Tcoil = (float)(RxData[7]);
//
//    // 计算总角度
//    if(motor->esc_back_position_last - motor->esc_back_position > 4096)
//        motor->circle_num++;
//    else if(motor->esc_back_position - motor->esc_back_position_last > 4096)
//        motor->circle_num--;
//
//    motor->serial_position = motor->circle_num * 8192 + motor->esc_back_position;
//    motor->serial_angle = motor->serial_position / 8192 * 2 * PI;
//    motor->esc_back_position_last = motor->esc_back_position;
//    motor->p_int = (RxData[0] << 8) | RxData[1];
//    motor->v_int = (RxData[2] << 4) | (RxData[3] >> 4);
//    motor->t_int = ((RxData[3] & 0x0F) << 8) | RxData[4];
//
//    motor->esc_back_position = uint_to_float(motor->p_int, DM4340_P_MIN, DM4340_P_MAX, 16);
//    motor->esc_back_speed = uint_to_float(motor->v_int, DM4340_V_MIN, DM4340_V_MAX, 12);
//    motor->esc_back_current = uint_to_float(motor->t_int, DM4340_T_MIN, DM4340_T_MAX, 12);
//
//    motor->Tmos = (float)RxData[6];
//    motor->state = RxData[0] >> 4;




/// @brief DM电机使能函数
/// @param can_id CAN ID
/// @param mode_offset 地址偏移量
/// @param enable 使能/失能
void CAN_cmd_motor_control(uint16_t can_id, uint16_t mode_offset, bool enable)
{
    uint32_t send_mail_box;
    uint8_t motor_index = can_id - 1; // 电机ID转换为数组索引(1->0, 2->1, 3->2)

    if (enable) {
        // 检查电机是否已经使能并收到反馈
        if (motor_feedback_states[motor_index].enabled &&
            motor_feedback_states[motor_index].feedback_received) {

            if (DM4340_Date[motor_index].real_angle == 0.0){    //检测是否为特定值
                CAN_cmd_motor_set_zero(can_id);                 //第一次启动设为零点
                DM4340_Date[motor_index].real_angle = DM4340_Date[motor_index].esc_back_position;
            }
            return; // 已经使能并收到反馈，不再发送
        }

        // 检查上次发送时间，避免频繁发送
        if (HAL_GetTick() - motor_feedback_states[motor_index].last_send_time < 100) {
            return; // 发送间隔太短，跳过
        }

        // 更新状态
        motor_feedback_states[motor_index].enabled = true;
        motor_feedback_states[motor_index].feedback_received = false;
        motor_feedback_states[motor_index].last_send_time = HAL_GetTick();
    } else {
        // 失能电机时重置状态
        motor_feedback_states[motor_index].enabled = false;
        motor_feedback_states[motor_index].feedback_received = false;
    }

    gimbal_tx_message.StdId = can_id + mode_offset;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;

    /* 填充前7字节为0xFF */
    memset(gimbal_can_send_data, 0xFF, 7);
    /* 根据enable参数设置第8字节 */
    gimbal_can_send_data[7] = enable ? 0xFC : 0xFD;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}



void CAN_cmd_motor_set_zero(uint16_t can_id)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = can_id;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;

    /* 填充前7字节为0xFF */
    memset(gimbal_can_send_data, 0xFF, 7);
    /* 根据enable参数设置第8字节 */
    gimbal_can_send_data[7] =0xFE;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}





/* 封装的DM4340控制接口实现 */

// PID参数定义


/**
  * @brief  DM4340控制初始化
  * @param  motor_id: 电机ID (0-2)
  */


/**
  * @brief  设置DM4340目标角度
  * @param  motor_id: 电机ID (0-2)
  * @param  angle: 目标角度(弧度)
  */

/**
  * @brief  DM4340控制循环
  * @param  motor_id: 电机ID (0-2)
  */
