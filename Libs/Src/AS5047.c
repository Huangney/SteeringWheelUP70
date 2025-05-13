/*
 * @Author: Agilawood
 * @file: AS5047磁编码器封装库
 * @LastEditors: Agilawood
 * @LastEditTime: 2024-11-29
 * @FilePath: E:\up70_2025\AS5047\MDK-ARM\AS5047.uvprojx-μVision
 * @Note: 
 * 1.SPI通信使用CPOL=0,SPHA=1；
 * 2.SPI波特率<10MHz
 * 3.CRC Calculation 不要开，因为32会在原始数据末尾发CRC，而AS5407的CRC应在最高位
 * 4.详细查看AS5407的手册，TEST脚接地，并且最好采用3V3供电模式
 * 5.本文件应采用UTF-8编码
 * Copyright (c) 2024 by Agilawood, All Rights Reserved.
 */
#include "AS5047.h"
// uint8_t bitsread[16];

float angle_deg = 0;
int angle_code = 0;
int angle_code_raw = 0;
float angle_deg_raw = 0;
/**
 * @description: 偶校验位计算，even:偶数，parity：奇偶校验
 * @param data_2_byte {uint16_t} 需要进行偶校验的数据（2 Byte）
 * @return 奇偶校验位，非偶为0，反之为1，放在数据帧第15位
 * @note 理解时只需关注最低位。循环检测data_2_byte的每一位，在最低位处理。当最低位是1时，将parity_bit_value变为0x0001。只有当data_2_byte中有偶数个1时，parity_bit_value才为0x0000
 */
uint16_t evenParityBitCal(uint16_t data_2_byte)
{
    uint16_t parity_bit_value = 0; // 存储偶校验值
    while (data_2_byte != 0)       // 检查数据每一位，直到该数据的位数被全部右移
    {
        parity_bit_value ^= data_2_byte;
        data_2_byte >>= 1;
    }
    return (parity_bit_value & 0x1); // 取数据最低位，为0则偶校验正确
}

// spi读写一个字节
uint16_t spiHandleOneByte(uint16_t _txdata)
{
    uint16_t rxdata = 0;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // NSS位(或CS)由高变低是数据传输的起始信号
    if (HAL_SPI_TransmitReceive(&AS5047_SPI, (uint8_t *)&_txdata, (uint8_t *)&rxdata, 1, 1000) != HAL_OK)//主从机读写一字节数据
    {
        rxdata = 0;
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // 数据传输终止
    return rxdata;
}

// 读取磁编数据，没做偶校验
uint16_t readAS5047()
{
    uint16_t data = 0;
    // 进行两次spi处理数据帧操作，data在下一帧CMD时被读得
    // uint16_t data_frame = ANGLECOM | 0xC000; // 即0011 1111 1111 1111 | 0100 0000 0000 0000，读指令 bit14 置1
    // spiHandleOneByte(data_frame);//数据帧1：发送CMD
    // data = spiHandleOneByte(data_frame); // 数据帧2：获取data
    data = spiHandleOneByte(0xFFFF);//查看寄存器，这里有巧合，直接给0xFFFF就可以，并且直接读同一帧的数据也没有大问题
    //     data = spiHandleOneByte(0xFFFF);
	// 	int temp_i = 0;
	// 	int data_temp = data;
	// while(temp_i++<13)
	// {
	// 	bitsread[temp_i] = data_temp & 1;
	// 	data_temp = data_temp >> 1;
	// }
    data &= 0x3fff;                      // 将数据按位与0011 1111 1111 1111,取低十四位值
    return data;
}

/**s
 * @description: 读取磁编寄存器数据（全面版），不用也可以
 * @param register_add 要读取的寄存器地址
 * @return data ，寄存器中的数据
 * @note
 */
uint16_t readAS5047_CRC(uint16_t register_add)
{
    uint16_t data = 0;
    register_add |= 0x4000; // 读指令，14bit置1

    /* 对第一帧数据(CMD)做偶校验 */
    if (evenParityBitCal(register_add) != 0)
    {
        register_add |= 0x8000; // 如果前15位1的个数为奇数，则bit15置1
    }

    spiHandleOneByte(register_add); // 发送CMD,不必管返回的数据
                                    //     data = spiHandleOneByte(register_add);//发送指令，获取的data对应上一帧的指令，但经测试这句不好使，用下面发空指令的
    data = spiHandleOneByte(NOP | 0x4000); // 发送一条空指令，读取上一次指令返回的数据

    data &= 0x3fff; // 有效数据存储在低14bit，取低十四位bit
    return data;
}

/**
 * @brief  电机角度接口函数
 * @note 使用前首先需要得到舵轮零位的磁编数据
 * @param motor_angle 接收
 * @param steer_zero_angle 舵轮零位时磁编传的数据
 * @retval {*}
 */
void getMotorAngle(uint16_t *motor_angle, uint16_t steer_zero_angle)
{
    uint16_t rx_buffer; // 用于接收磁编的数据
    angle_code_raw = readAS5047();
    angle_deg_raw = angle_code_raw / 16384.0 * 360.0;
    rx_buffer = angle_code_raw;
    //    rx_buffer = readAS5047_CRC(0x3FFF);
    if (rx_buffer >= steer_zero_angle)
    {
        rx_buffer -= steer_zero_angle;
    }
    else
    {
        rx_buffer = rx_buffer + 16383 - steer_zero_angle;
    }

    *motor_angle = rx_buffer;
}


/// @name 获得劣弧值（度）
/// @brief 获得两个角度的劣弧差值
/// @param angle_deg_real 
/// @param angle_deg_targ 
/// @return 
float get_minor_arc(float angle_deg_real, float angle_deg_targ)
{
    float angle_error = angle_deg_targ - angle_deg_real;

    if (angle_error > 180 || angle_error < -180)
    {
        if (angle_error > 180)
        {
            angle_error -= 360;
        }
        else
        {
            angle_error += 360;
        }
    }
    return angle_error;
}
