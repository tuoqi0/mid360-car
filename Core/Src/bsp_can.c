#include "bsp_can.h"
#include "can.h"



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


void can_filter_init(void)//筛选器配置
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;                //开启该过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;       //筛选器模式是ID掩码模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;      //筛选器位宽
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;                           //选择过滤器0
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      //把接收到的报文放入到FIFO0中
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);//把过滤器配置配置给can1外设
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);




    //can2
    can_filter_st.FilterActivation = ENABLE;                //开启该过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;       //筛选器模式是ID掩码模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;      //筛选器位宽
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;                           //选择过滤器0
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      //把接收到的报文放入到FIFO0中
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);//把过滤器配置配置给can1外设
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
