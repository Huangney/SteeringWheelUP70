/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_vesc.h"
#include "string.h"
#include "pids.h"
#include "AS5047.h"
#include "WS2812_yx.h"
#include "motor_c620.h"
#include "algorithm.h"


// 1，2，3，4
// 还得改VESC的ID
/**
 * 
 * 舵轮ID按如下方式：
 *    前方
 * 
 * 1        4
 * 
 *    车体
 * 
 * 2        3
 * 
 * 
 */
// 舵轮的 ID 和 DEBUG 在.h文件中定义的

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



#ifdef STEER_DEBUG
// #define STEER_SPEED_DEBUG
#define STEER_POSITION_DEBUG
#endif

// 舵轮的 舵向轮向优先级 优化方式
int StrWel_Priority = TARGBASE_STEERCALC;

// can总线接收的次数
int recv_times = 0;
// 从主控板接收信息的次数
int master_recv_times = 0;

// 设置的要求的目标舵轮角度
float set_angle_degree = 60.0;
// 角度环DEBUG用的变量
float angle_degree_debug = 30;



#ifdef Steer_Wheel_1
uint16_t steer_zero_angle = 3034;
int led_band_bias = -2;
#endif 
#ifdef Steer_Wheel_2
uint16_t steer_zero_angle = 14202;
int led_band_bias = -8;
#endif 
#ifdef Steer_Wheel_3
uint16_t steer_zero_angle = 7054;
int led_band_bias = -8;
#endif 



int targ_speed;
int targ_current;
float minor_angle_ouput;

// MASTER_CAN 接收到 的数据
uint8_t MasterCanData[8];

// 安全员计数器
int safe_guard_timer;

// 从主控板传来的目标速度
int recv_speed_rpm = 0;
// 从主控板传来的目标角度（0~16384）
int recv_angle_code = 0;
float recv_angle_deg = 0;
float last_recv_angle_deg = 0;    // 上一次的目标转动值

// 测试3508的ID用的
int read_3508_id;

// 3508 的pid
Pids m3508_posi_pid;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void WS2812_Refresh()
{
  HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)WS2812buf2send, 24 * (LED_Nums + 1));
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  recv_times++;

  // 来自 3508 和 VESC 的消息
  if (hfdcan->Instance == hfdcan1.Instance)
	{
		uint8_t Data[8];
		FDCAN_RxHeaderTypeDef RxMessage;
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxMessage, Data);

		read_3508_id = RxMessage.Identifier;
    if((RxMessage.Identifier >> 8) == 0x02)   // 如果是收到的 C620 的消息
    {
      // 注意，因为这里只控制了一个3508，所以可以这样子写
      if (M3508.motor_id == 0)
      {
        M3508.motor_id = RxMessage.Identifier;
      }
      
			if(RxMessage.Identifier == M3508.motor_id)
      M3508.msg_cnt++ <= 50 ? get_moto_offset(&M3508, Data) : get_moto_measure(&M3508, Data);

      // 预热3508
      if (M3508.preheated == 0)
      {
        motor_c620_preheat();
      }

      // 正式执行
      else
      {
        #ifndef STEER_DEBUG
        if (safe_guard_timer > 0)
        {
          set_angle_degree = recv_angle_deg;
          minor_angle_ouput = get_minor_arc(angle_deg, set_angle_degree);
          targ_speed = m3508_posi_pid.calc_output(&m3508_posi_pid, minor_angle_ouput, 8000);
          motor_c620_set_rpm(targ_speed, targ_speed, targ_speed, targ_speed, 16000);
        }
        else
        {
          set_moto_current(M3508.my_fdcan, 0, 0);
        }
				#else
          // 速度环的DEBUG，请在main.c的上面激活
          #ifdef STEER_SPEED_DEBUG
				  motor_c620_set_rpm(targ_speed, targ_speed, targ_speed, targ_speed, 16000);
          #endif

          // 位置（角度）环的DEBUG，请在main.c的上面激活
          #ifdef STEER_POSITION_DEBUG
          minor_angle_ouput = get_minor_arc(angle_deg, angle_degree_debug);
          targ_speed = m3508_posi_pid.calc_output(&m3508_posi_pid, minor_angle_ouput, 8000);
          motor_c620_set_rpm(targ_speed, targ_speed, targ_speed, targ_speed, 16000);
          #endif

        #endif
      }
    }
    


    else if ((RxMessage.Identifier & 0x900) == 0x900)
    {
      MotorVescRecvData vesc_recvs; // 新建接收用的结构体
      vesc_recvs.rx_header = RxMessage;
      for (int i = 0; i < 8; i++)
      {
        vesc_recvs.recv_data[i] = Data[i];
      }
      motor_vesc_handle(vesc_recvs);
    }
	}

  /******     hcan2 来自主机的消息    *********/      

  if (hfdcan->Instance == hfdcan2.Instance)   // 如果符合目标CAN通道
  {
		FDCAN_RxHeaderTypeDef RxMessage;
		HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxMessage, MasterCanData);
    master_recv_times++;

    #ifndef STEER_DEBUG
    if (RxMessage.Identifier == My_Steer_ID + 114)    // 确认自己的ID正确
    {
      // 告诉安全员自己收到了
      safe_guard_timer = SAFE_GUARD_TIME;
      // 计算收到的数据
      recv_speed_rpm = (int)(MasterCanData[0] << 24 | MasterCanData[1] << 16 | MasterCanData[2] << 8 | MasterCanData[3]);
      recv_angle_code = (int)(MasterCanData[4] << 24 | MasterCanData[5] << 16 | MasterCanData[6] << 8 | MasterCanData[7]);
      
      // 在刷新值之前，保留上一次的值用于优化
      last_recv_angle_deg = recv_angle_deg;

      // 舵轮自身坐标系和解算是反的，反一下就行
      recv_angle_deg = (360.0 - (recv_angle_code / 16384.0 * 360));


      /****   计算舵向最优的转向方式  ****/
      if (StrWel_Priority == REALBASE_STEERCALC)    // 基于现实的优化
      {
        algo_get_steerBetter_vec(test_rpm_value, angle_deg, &recv_speed_rpm, &recv_angle_deg);
      }
      else if (StrWel_Priority == TARGBASE_STEERCALC)    // 基于目标的优化
      {
        algo_get_steerBetter_vec(test_rpm_value, last_recv_angle_deg, &recv_speed_rpm, &recv_angle_deg);
      }
      else
      {
        // 不做优化，无事发生
      }
    }
    #endif
  }
  

  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t msg_test[9] = "fuckfuck";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN2_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  // 轮1
	m3508_posi_pid = pids_create_init(256, 64, 0, 0.001, 8000, 0.15, 0);
  
  
  // 请在其他部分初始化好了之后，再启动CAN总线
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim17);

  motor_c620_init(&hfdcan1);


  HAL_Delay(1000);
  motor_vesc_init(&hfdcan1);
  motor_vesc_init(&hfdcan2);
  
  WS2812_InitBuffer();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // WS2812_Wonder(0.5);
    

    WS2812_Angles(40, 0, 112, 6, 0.5, angle_deg_raw, 1, led_band_bias);
    WS2812_Angles_Add(122, 103, 0, 6, 0.5, (180 - angle_deg_raw), 0, led_band_bias);
    WS2812_Refresh();

    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
