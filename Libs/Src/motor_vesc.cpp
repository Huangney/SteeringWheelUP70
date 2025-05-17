#include "motor_vesc.h"


/*          需要操作的电机在这里注册            */
// 101, ?, 123, ?


#ifdef Steer_Wheel_1
MotorVESC motor_vesc_1(&hfdcan1, 0, 120);
#endif 
#ifdef Steer_Wheel_2
MotorVESC motor_vesc_1(&hfdcan1, 0, 123);
#endif 
#ifdef Steer_Wheel_3
MotorVESC motor_vesc_1(&hfdcan1, 0, 122);
#endif 

float read_current = 0;
float read_duty = 0;
int read_rpm = 0;
int test_rpm_value = 0;
float test_duty_value = 0.1;


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
    can_filter_config(hcan);
    HAL_FDCAN_Start(hcan);
    CAN_Interrupt_Enable(hcan);
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
        targ_motor_vesc->setMotorRPM(1000);

        // 获取电调上报的消息类型
        CanPacketType vesc_status_type = (CanPacketType)(vesc_recvs.rx_header.Identifier >> 8);

        // 解码
        switch (vesc_status_type)
        {
            case CAN_PACKET_STATUS: // 第一类上报
            {
                targ_motor_vesc->motor_duty_real = (vesc_recvs.recv_data[6] * 256 + vesc_recvs.recv_data[7]) / 1000.0;

                read_current = (vesc_recvs.recv_data[4] * 256 + vesc_recvs.recv_data[5]) / 10.0;

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

    if (can_n == &hfdcan2) {
        FdcanRxFilter.FilterIndex = 14;
    } else {
        FdcanRxFilter.FilterIndex = 0;
    }
    
	FdcanRxFilter.FilterType = FDCAN_FILTER_MASK;				   // 滤波器类型
    FdcanRxFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;		   // 过滤器0关联到FIFO0
	FdcanRxFilter.FilterID1 = 0x0000;							   // 32位ID
	FdcanRxFilter.FilterID2 = 0x0000;							   // 如果FDCAN配置为传统模式的话，这里是32位掩码
    
	if (HAL_FDCAN_ConfigFilter(can_n, &FdcanRxFilter) != HAL_OK) // 滤波器初始化
	{
		Error_Handler();
	}
}



