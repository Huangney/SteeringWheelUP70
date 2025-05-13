/*
 * @Author: scy
 * @Date: 2024-02-07 17:31:48
 * @LastEditors: scy
 * @LastEditTime: 2024-02-25 11:42:59
 * @FilePath: \MDK-ARMf:\Intelligent Car\competition item\Robocon\sterring wheel\BSP\Inc\bsp_fdcan.h
 * @Description: 
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32g4xx_hal.h"
#include "fdcan.h"

void fdcan_filter_init_recv_all(FDCAN_HandleTypeDef *_hfdcan);
uint8_t fdcan_C620_send_msg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t len, int motor_id);

#endif 
