/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// #define Steer_Wheel_1
#define Steer_Wheel_2
// #define Steer_Wheel_3


#ifdef Steer_Wheel_1
#define My_Steer_ID 1
#endif 

#ifdef Steer_Wheel_2
#define My_Steer_ID 2
#endif 

#ifdef Steer_Wheel_3
#define My_Steer_ID 3
#endif 



// extern 只作打印用途
extern int targ_speed;
extern float minor_angle_ouput;
extern uint16_t steer_zero_angle;
extern float recv_angle_deg;
extern float set_angle_degree;
extern float angle_degree_debug;

// 以下extern可以找到在stm32it中找到，用于保护丢失信号
extern int safe_guard_timer;
extern int recv_speed_rpm;

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define SAFE_GUARD_TIME 200





//#define STEER_DEBUG

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
