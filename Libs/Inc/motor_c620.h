#ifndef MOTOR_C620_H
#define MOTOR_C620_H

#include "stm32g4xx_hal.h"
#include "bsp_fdcan.h"
#include "pids.h"

#define ABS(x) ((x > 0) ? (x) : (-x))
#define FILTER_BUF_LEN 5
//电机反馈信息结构体变量定义
typedef struct
{
	int16_t speed_rpm;
	int16_t real_current;
	int16_t given_current;
	uint8_t hall;
	uint16_t angle;		 //[0,8191]
	uint16_t last_angle;
	uint16_t offset_angle;
	int32_t round_cnt;
	int32_t total_angle;
	uint8_t buf_idx;
	uint16_t angle_buf[FILTER_BUF_LEN];
	uint16_t fited_angle;
	uint32_t msg_cnt;
	FDCAN_HandleTypeDef *my_fdcan;

	Pids motor_pid;
	uint8_t preheated;
	int motor_id;

} moto_measure_t;

extern int16_t M3508TargetSpd;
extern int16_t M2006TargetPos;

extern moto_measure_t M2006; // M2006反馈信息结构体
extern moto_measure_t M3508; // M3508反馈信息结构体

extern float targ_current_1;	// 便于debug和调试

void motor_c620_init(FDCAN_HandleTypeDef *hfdcan);
void get_moto_measure(moto_measure_t *ptr, uint8_t *Data);
void get_moto_offset(moto_measure_t *ptr, uint8_t *Data);
void get_total_angle(moto_measure_t *p);
void set_moto_current(FDCAN_HandleTypeDef *hfdcan, int16_t motor_0, int16_t motor_1);
void motor_c620_preheat();
void motor_c620_set_rpm(int motor_1_rpm, int motor_2_rpm, int motor_3_rpm, int motor_4_rpm, int current_lim);
#endif 
