/*
 * @Author: scy
 * @Date: 2024-02-07 17:31:48
 * @LastEditors: scy
 * @LastEditTime: 2024-02-25 11:42:35
 * @FilePath: \MDK-ARMf:\Intelligent Car\competition item\Robocon\sterring wheel\BSP\Src\bsp_fdcan.c
 * @Description:
 */
#include "motor_c620.h"
#include "bsp_fdcan.h"

int16_t M3508TargetSpd=0;//M3508电机目标速度
int16_t M2006TargetPos=0;//M2006电机目标角度
moto_measure_t M2006; // M2006反馈信息结构体
moto_measure_t M3508; // M3508反馈信息结构体
uint16_t MyId;

float targ_current_1;
float preheated_time = 0;


void motor_c620_init(FDCAN_HandleTypeDef *hfdcan)
{
	M3508.my_fdcan = hfdcan;
	
	#ifdef Steer_Wheel_1
	M3508.motor_pid = pids_create_init(3, 2.5, 0.001, 0.001, 6000, 0.25, 0);
	#endif 
	#ifdef Steer_Wheel_2
	M3508.motor_pid = pids_create_init(3, 2.5, 0.001, 0.001, 6000, 0.25, 0);
	#endif 
	#ifdef Steer_Wheel_3
	M3508.motor_pid = pids_create_init(3, 2.5, 0, 0.001, 6000, 0.25, 0);
	#endif 

	

}

void motor_c620_set_rpm(int motor_1_rpm, int motor_2_rpm, int motor_3_rpm, int motor_4_rpm, int current_lim)
{
	float targ_current_temp = M3508.motor_pid.calc_output_incremental(&M3508.motor_pid, (motor_1_rpm - M3508.speed_rpm), 16000);

	if (targ_current_temp > current_lim)
	{
	targ_current_temp = current_lim;
	}
	if (targ_current_temp < -current_lim)
	{
	targ_current_temp = -current_lim;
	}
	
	targ_current_1 = targ_current_temp;

	set_moto_current(M3508.my_fdcan, targ_current_1, targ_current_1);
}

void motor_c620_preheat()
{
	if (preheated_time < 0.250)
	{
		motor_c620_set_rpm(200, 200, 200, 200, 1500);
	}
	else if (preheated_time > 0.250 && preheated_time < 0.500)
	{
		motor_c620_set_rpm(-200, -200, -200, -200, 1500);
	}
	else
	{
		// 预热时间为0.5秒
		M3508.preheated = 1;
	}
	
	preheated_time += M3508.motor_pid.delta_t;
}

/**
 * @description: 获取电机反馈信息
 * @param {moto_measure_t} *ptr电机反馈信息结构体指针
 * @param {uint8_t} *Data接收到的数据
 * @return {*}无
 */
void get_moto_measure(moto_measure_t *ptr, uint8_t *Data)
{
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]);
	ptr->real_current = (int16_t)(Data[2] << 8 | Data[3]);
	ptr->speed_rpm = ptr->real_current;
	ptr->given_current = (int16_t)(Data[4] << 8 | Data[5]) / -5;
	ptr->hall = Data[6];
	if (ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt--;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}




/**
 * @description: 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
 * @param {moto_measure_t} *ptr电机结构体指针
 * @param {uint8_t} *Data接收到的数据
 * @return {*}无
 */
void get_moto_offset(moto_measure_t *ptr, uint8_t *Data)
{
	ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]);
	ptr->offset_angle = ptr->angle;
}



/**
 * @description: 获取电机转过的总角度
 * @param {moto_measure_t} *p电机反馈信息结构体指针
 * @return {*}无
 */
void get_total_angle(moto_measure_t *p)
{

	int res1, res2, delta;
	if (p->angle < p->last_angle)
	{											// 可能的情况
		res1 = p->angle + 8192 - p->last_angle; // 正转，delta=+
		res2 = p->angle - p->last_angle;		// 反转	delta=-
	}
	else
	{											// angle > last
		res1 = p->angle - 8192 - p->last_angle; // 反转	delta -
		res2 = p->angle - p->last_angle;		// 正转	delta +
	}
	// 不管正反转，肯定是转的角度小的那个是真的
	if (ABS(res1) < ABS(res2))
		delta = res1;
	else
		delta = res2;
	p->total_angle += delta;
	p->last_angle = p->angle;
}




/**
 * @description: 发送电机电流控制
 * @param {FDCAN_HandleTypeDef} *hfdcan使用的FDCAN
 * @param {uint16_t} m2006
 * @param {uint16_t} m3508
 * @return {*}无
 */
void set_moto_current(FDCAN_HandleTypeDef *hfdcan, int16_t motor_0, int16_t motor_1)
{
	uint8_t motor_current_data[8] = {0};
	motor_current_data[0] = motor_0 >> 8;
	motor_current_data[1] = motor_0;
	motor_current_data[2] = motor_1 >> 8;
	motor_current_data[3] = motor_1;

	motor_current_data[4] = motor_0 >> 8;
	motor_current_data[5] = motor_0;
	motor_current_data[6] = motor_1 >> 8;
	motor_current_data[7] = motor_1;

	fdcan_C620_send_msg(hfdcan, motor_current_data, FDCAN_DLC_BYTES_8, M3508.motor_id);
}





/**
 * @description: FDCAN接收中断回调函数
 * @param {FDCAN_HandleTypeDef} *hfdcan
 * @param {uint32_t} RxFifo0ITs
 * @return {*}
 */
// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
// {
// 	if (hfdcan == &hfdcan1)
// 	{
// 		uint8_t Data[8];
// 		FDCAN_RxHeaderTypeDef Fdcan1RxHeader;
// 		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &Fdcan1RxHeader, Data);
// 		switch (Fdcan1RxHeader.Identifier)
// 		{
// 		case 0X201:
// 			M2006.msg_cnt++ <= 50 ? get_moto_offset(&M2006, Data) : get_moto_measure(&M2006, Data);
// 		break;
// 		case 0X202:
// 			M3508.msg_cnt++ <= 50 ? get_moto_offset(&M3508, Data) : get_moto_measure(&M3508, Data);
// 		break;
// 		}
// 	}
// }
