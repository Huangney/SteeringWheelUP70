#include "motor_vesc.h"


void MotorVESC::setMotorDuty(float duty)
{
    sendCanTXBuffer(CAN_PACKET_SET_DUTY, duty);
}

void MotorVESC::setMotorRPM(int RPM)
{
    sendCanTXBuffer(CAN_PACKET_SET_RPM, RPM);
}

MotorVESC::MotorVESC(FDCAN_HandleTypeDef* can_n, int motor_id, int motor_can_id)
{
    this->motor_id = motor_id;
    this->motor_can_id = motor_can_id;
    targ_can_n = can_n;
}

void MotorVESC::sendCanTXBuffer(CanPacketType cmd_type, float values)
{
    static uint32_t txmailbox;		        // CAN 邮箱
    FDCAN_TxHeaderTypeDef TxMsg;		        // TX 消息

    // 配置标准CAN参数
	TxMsg.Identifier = (cmd_type << 8 | motor_can_id);    // 低8位为CAN_ID，高21位为指令ID
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

    HAL_FDCAN_AddMessageToTxFifoQ(targ_can_n, &TxMsg, txbuf);
}

float limit_abs(float targ_num, float limit)
{
    if (targ_num > limit)
    {
        return limit;
    }
    if (targ_num < -limit)
    {
        return limit;
    }
    return targ_num;
}

