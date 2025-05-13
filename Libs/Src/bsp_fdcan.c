/*
 * @Author: scy
 * @Date: 2024-02-07 17:31:48
 * @LastEditors: scy
 * @LastEditTime: 2024-02-25 11:42:35
 * @FilePath: \MDK-ARMf:\Intelligent Car\competition item\Robocon\sterring wheel\BSP\Src\bsp_fdcan.c
 * @Description:
 */
#include "bsp_fdcan.h"


/**
 * @description: 接收过滤器（全部接收）
 * @param {FDCAN_HandleTypeDef} *_hfdcan使用的FDCAN
 * @return {*}无
 */
void fdcan_filter_init_recv_all(FDCAN_HandleTypeDef *_hfdcan)
{

	FDCAN_FilterTypeDef FdcanRxFilter;
	FdcanRxFilter.IdType = FDCAN_STANDARD_ID;					   // 标准ID
	FdcanRxFilter.FilterIndex = 0;								   // 滤波器索引
	FdcanRxFilter.FilterType = FDCAN_FILTER_MASK;				   // 滤波器类型
	FdcanRxFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;		   // 过滤器0关联到FIFO0
	FdcanRxFilter.FilterID1 = 0x0000;							   // 32位ID
	FdcanRxFilter.FilterID2 = 0x0000;							   // 如果FDCAN配置为传统模式的话，这里是32位掩码
	if (HAL_FDCAN_ConfigFilter(_hfdcan, &FdcanRxFilter) != HAL_OK) // 滤波器初始化
	{
		Error_Handler();
	}
}

/**
 * @description: 发送信息
 * @param {FDCAN_HandleTypeDef} *_hfdcan使用的FDCAN
 * @param {uint8_t} *msg发送数据的指针
 * @param {uint32_t} len发送数据的长度
 * @return {*}1：发送成功 0：发送错误
 */
uint8_t fdcan_C620_send_msg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t len, int motor_id)
{
	FDCAN_TxHeaderTypeDef FdcanTxHeader;
	if (motor_id > 0x204)
	{
		FdcanTxHeader.Identifier = 0x1FF;			  // 32位ID
	}
	else
	{
		FdcanTxHeader.Identifier = 0x200;			  // 32位ID
	}
	FdcanTxHeader.IdType = FDCAN_STANDARD_ID;	  // 标准ID
	FdcanTxHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
	FdcanTxHeader.DataLength = len;				  // 数据长度
	FdcanTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	FdcanTxHeader.BitRateSwitch = FDCAN_BRS_OFF;		   // 关闭速率切换
	FdcanTxHeader.FDFormat = FDCAN_CLASSIC_CAN;			   // 传统的CAN模式
	FdcanTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
	FdcanTxHeader.MessageMarker = 0;
	
	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FdcanTxHeader, msg) != HAL_OK)
		return 0;
	return 1;
}

