#include "motor_vesc.h"

/*          需要操作的电机在这里注册            */
// 101, ?, 123, ?

#ifdef Steer_Wheel_1
MotorVESC motor_vesc_1;
#endif 
#ifdef Steer_Wheel_2
MotorVESC motor_vesc_1;
#endif 
#ifdef Steer_Wheel_3
MotorVESC motor_vesc_1;
#endif 

float read_current = 0;
float read_duty = 0;
int read_rpm = 0;
int test_rpm_value = 0;
float test_duty_value = 0.1;

int configure_fdcan2_filter(uint32_t can_id1, uint32_t can_id2);

void setMotorDuty(MotorVESC* self, float duty)
{
    self->sendCanTXBuffer(self, CAN_PACKET_SET_DUTY, duty);
}

void setMotorRPM(MotorVESC* self, int RPM)
{
    self->sendCanTXBuffer(self, CAN_PACKET_SET_RPM, (float)RPM);
}

void sendCanTXBuffer(MotorVESC* self, CanPacketType cmd_type, float values)
{
    static uint32_t txmailbox;		        // CAN 邮箱
    FDCAN_TxHeaderTypeDef TxMsg;		        // TX 消息

    // 配置标准CAN参数
    TxMsg.Identifier = (cmd_type << 8 | self->motor_can_id);    // 低8位为CAN_ID，高21位为指令ID
    TxMsg.IdType = FDCAN_EXTENDED_ID;	  // 标准ID
    TxMsg.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
    TxMsg.DataLength = (uint8_t)8;				  // 数据长度

    // 配置FDCAN参数
    TxMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMsg.BitRateSwitch = FDCAN_BRS_OFF;		   // 关闭速率切换
    TxMsg.FDFormat = FDCAN_CLASSIC_CAN;			   // 传统的CAN模式
    TxMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
    TxMsg.MessageMarker = 0;

    uint8_t txbuf[8] = {0};
    switch (cmd_type)
    {
        case CAN_PACKET_SET_DUTY:
        {
            int32_t data;
            data = (int32_t)(values * 100000) ;
            txbuf[0] = data >> 24 ;
            txbuf[1] = data >> 16 ;
            txbuf[2] = data >> 8 ;
            txbuf[3] = data ;
            break;
        }
        case CAN_PACKET_SET_RPM:
        {
            int32_t data;
            data = (int32_t)(values) ;
            txbuf[0] = data >> 24 ;
            txbuf[1] = data >> 16 ;
            txbuf[2] = data >> 8 ;
            txbuf[3] = data ;
            break;
        }
        default:
            break;
    }

    HAL_FDCAN_AddMessageToTxFifoQ(self->targ_can_n, &TxMsg, txbuf);
}

void MotorVESC_Init(MotorVESC* self, FDCAN_HandleTypeDef* can_n, int motor_id, int motor_can_id)
{
    self->motor_id = motor_id;
    self->motor_can_id = motor_can_id;
    self->targ_can_n = can_n;
    self->setMotorDuty = setMotorDuty;
    self->setMotorRPM = setMotorRPM;
    self->sendCanTXBuffer = sendCanTXBuffer;
}

void CAN_Interrupt_Enable(FDCAN_HandleTypeDef *can_n)
{
    if (HAL_FDCAN_ActivateNotification(can_n, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
        // 中断配置失败处理
    }
}

/// @name motor_vesc_get_rpm
/// @brief 返回指定标签电机的真实转速
/// @param motor_id ：0-3
int motor_vesc_get_rpm(int motor_id)
{
    if (motor_id == motor_vesc_1.motor_id)
    {
        read_rpm = motor_vesc_1.motor_rpm_real;
        return read_rpm;
    }
    
    else return 0;
}

/**
 * @name motor_vesc_init
 * @brief 这个函数用于初始化
 * @note 函数内配置了 can2过滤器，启动了 can，启用了 can中断
 * @warning 
 */
void motor_vesc_init(FDCAN_HandleTypeDef *hcan)
{
    #ifdef Steer_Wheel_1
        MotorVESC_Init(&motor_vesc_1, hcan, 0, 120);
    #endif 
    #ifdef Steer_Wheel_2
        MotorVESC_Init(&motor_vesc_1, hcan, 0, 123);
    #endif 
    #ifdef Steer_Wheel_3
        MotorVESC_Init(&motor_vesc_1, hcan, 0, 122);
    #endif 
    
    bsp_can_init(hcan);
}

void bsp_can_init(FDCAN_HandleTypeDef *hcan)
{
    
    if (hcan == &hfdcan2)
    {
        configure_fdcan2_filter(Steer_Control * My_Steer_ID, Steer_LinkConfirm * My_Steer_ID);
        HAL_FDCAN_Start(hcan);
        CAN_Interrupt_Enable(hcan);
    }
    else
    {
        can_filter_config(hcan);
        HAL_FDCAN_Start(hcan);
        CAN_Interrupt_Enable(hcan);
    }
    
    
}

/**
 * @name motor_vesc_handle
 * @brief 这个函数处理了收到的信息并回复电机
 * @note 
 * @details 本函数内完成了回复电机（设置电机速度等）的操作；如果有需要的操作可以在此更改
 * @warning 
 */
void motor_vesc_handle(MotorVescRecvData vesc_recvs)
{
    // 解码数据来自于CAN总线上的谁
    uint8_t can_id = vesc_recvs.rx_header.Identifier & 0xff;
    MotorVESC* targ_motor_vesc;

    // 和哪个电机匹配就和谁发
    if (can_id == motor_vesc_1.motor_can_id)
    { 
        targ_motor_vesc = &motor_vesc_1;
    }

    {
        targ_motor_vesc->setMotorRPM(targ_motor_vesc, test_rpm_value);

        // 获取电调上报的消息类型
        CanPacketType vesc_status_type = (CanPacketType)(vesc_recvs.rx_header.Identifier >> 8);

        // 解码
        switch (vesc_status_type)
        {
            case CAN_PACKET_STATUS: // 第一类上报
            {
                targ_motor_vesc->motor_duty_real = (vesc_recvs.recv_data[6] * 256 + vesc_recvs.recv_data[7]) / 1000.0;

                targ_motor_vesc->current_real = ((int16_t)(vesc_recvs.recv_data[4] << 8 | vesc_recvs.recv_data[5])) / 10.0;

                targ_motor_vesc->motor_rpm_real = (int32_t)(vesc_recvs.recv_data[0] << 24 | vesc_recvs.recv_data[1] << 16
                    | vesc_recvs.recv_data[2] << 8 | vesc_recvs.recv_data[3]);
            }
        }
    }
}

/// @brief 
/// @param can_n 
void can_filter_config(FDCAN_HandleTypeDef *can_n)
{
    FDCAN_FilterTypeDef FdcanRxFilter;
    FdcanRxFilter.IdType = FDCAN_EXTENDED_ID;					   // 标准ID

    FdcanRxFilter.FilterIndex = 0;
    
    FdcanRxFilter.FilterType = FDCAN_FILTER_MASK;				   // 滤波器类型
    FdcanRxFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;		   // 过滤器0关联到FIFO0
    FdcanRxFilter.FilterID1 = 0x0000;							   // 32位ID
    FdcanRxFilter.FilterID2 = 0x0000;							   // 如果FDCAN配置为传统模式的话，这里是32位掩码
    
    if (HAL_FDCAN_ConfigFilter(can_n, &FdcanRxFilter) != HAL_OK) // 滤波器初始化
    {
        Error_Handler();
    }
}



/**
 * 配置FDCAN2过滤器，使用两个独立过滤器分别严格匹配两个CAN ID
 * @param can_id1 第一个允许通过的CAN ID
 * @param can_id2 第二个允许通过的CAN ID
 * @return 配置成功返回0，失败返回-1
 */
int configure_fdcan2_filter(uint32_t can_id1, uint32_t can_id2)
{
    FDCAN_FilterTypeDef sFilterConfig;
    int result = 0;

    // 配置全局过滤器
    // 丢弃未被过滤器接收的标准帧和扩展帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                FDCAN_REJECT,          // 标准帧未过滤时的处理
                                FDCAN_REJECT,          // 扩展帧未过滤时的处理
                                ENABLE,   // 丢弃远程帧
                                ENABLE);  // 丢弃扩展远程帧

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;       // 扩展ID模式
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;   // 掩码模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 关联到RX FIFO 0

    // 配置第一个过滤器（索引14）匹配can_id1
    sFilterConfig.FilterIndex = 2;                  // 过滤器索引14
    sFilterConfig.FilterID1 = can_id1;              // 匹配ID1
    sFilterConfig.FilterID2 = 0x1FFFFFFF;           // 掩码：全部位必须匹配
    // sFilterConfig.FilterID2 = 0x0;           // 掩码：全部位必须匹配
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    // 配置第二个过滤器（索引15）匹配can_id2
    sFilterConfig.FilterIndex = 3;                  // 过滤器索引15
    sFilterConfig.FilterID1 = can_id2;              // 匹配ID2
    sFilterConfig.FilterID2 = 0x1FFFFFFF;           // 掩码：全部位必须匹配
    // sFilterConfig.FilterID2 = 0x0;           // 掩码：全部位必须匹配
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    return result;
}



float limit_abs(float targ_num, float limit)
{
    if (targ_num > limit)
    {
        return limit;
    }
    if (targ_num < -limit)
    {
        return -limit;
    }
    return targ_num;
}