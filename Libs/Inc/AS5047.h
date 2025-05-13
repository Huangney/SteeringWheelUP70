#ifndef __AS5047_H__
#define __AS5047_H__

#include "spi.h"
#include "stdint.h"
// #include "main.h"

#define AS5047_SPI hspi1

/* AS5047 寄存器地址 */
#define NOP 0x0000
#define ERRFL 0x0001
#define PROG 0x0003
#define DIAAGC 0x3FFC
#define MAG 0x3FFD
#define ANGLEUNC 0x3FFE
#define ANGLECOM 0x3FFF

#define ZPOSM 0x0016
#define ZPOSL 0x0017
#define SETTINGS1 0x0018
#define SETTINGS2 0x0019

void getMotorAngle(uint16_t* motor_angle, uint16_t steer_zero_angle);
uint16_t readAS5047(void);
uint16_t readAS5047_CRC(uint16_t register_add);
float get_minor_arc(float angle_deg_real, float angle_deg_targ);

extern int test_in_1;
extern float angle_deg;
extern int angle_code;
extern float angle_deg_raw;

#endif
