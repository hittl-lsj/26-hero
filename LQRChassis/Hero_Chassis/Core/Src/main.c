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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "Dbus.h"
#include "mecanum.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LQR.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t this_time_rx_len=0;
uint8_t DBUS_rx_buf[SBUS_RX_BUF_NUM];
float f=0;
float RC=160.0;
float L1,L2,R1,R2;
float l_up=0,R__up=0;
float l_up_real=0,R__up_real=0;
Chassis_Speed speed_3;
int16_t out_speed[4];


// float LQROUTpot;
// //测试
//     float target_speed_0 = 1000.0f;  // 电机0目标速度
//     float target_speed_1 = 800.0f;   // 电机1目标速度
//     float target_speed_2 = 600.0f;   // 电机2目标速度
//     float target_speed_3 = 400.0f;   // 电机3目标速度
//   LQR_SpeedController motor_ctrl;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);
  can_Init(&hcan1);
//Chassis speed_pid初始化
  Pid_Init(&speed_pid[0],2,0.0001,1.5,13000,0,0,3000,0,1300);//内环PID的初始化
  Pid_Init(&speed_pid[1],2,0.0001,1.149,13000,0,1200,3000,0,1200);//内环PID的初始化
  Pid_Init(&speed_pid[2],2,0.0001,1.5,13000,0,0,3000,0,1300);
  Pid_Init(&speed_pid[3],2,0.0001,1.5,13000,0,0,3000,0,1300);

//升降机构pid初始化
  Pid_Init(&speed_pid[4],2,0.0001,1.5,13000,0,0,3000,0,1300);
  Pid_Init(&speed_pid[5],2,0.0001,1.5,13000,0,0,3000,0,1300);
  Pid_Init(&position_pid[0],0.1,0,0,6000,100,0,3000,0,10000);
  Pid_Init(&position_pid[1],0.1,0,0,6000,100,0,3000,0,10000);

  //lqr_speed_init(&motor_ctrl, 0.5f, 0.1f, 1000.0f, 500.0f);
  HAL_TIM_Base_Start_IT(&htim1);
  uart_dma_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //boxSend(1000,1000,1000,100000);
	  HAL_Delay(1);
 	  speed_3.vx=(rc_ctrl.rc.ch0/660.0)*RC;
 	  speed_3.vy=(rc_ctrl.rc.ch1/660.0)*RC;
 	  speed_3.vw=(rc_ctrl.rc.ch2/660.0)*RC;
    if(rc_ctrl.rc.s2==2){
      l_up_real=0;
      R__up_real=0;
    }else if(rc_ctrl.rc.s2==1){
      l_up_real=-720;
      R__up_real=720;
    }
 	  mecanum_calc(&speed_3, out_speed);
 	  mecanum_Set_Motor_Speed(out_speed);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		memset(DBUS_rx_buf,0,36);//将前36个字节清�???????????????????????????????????
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*)&DBUS_rx_buf[0],36);

	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//处理接收完成事件
{

	if(huart==&huart3)
		{
			HAL_UART_DMAStop(huart);
			if(Size>=18)
			{
				dbus_to_rc(DBUS_rx_buf,&rc_ctrl);
				DR16_Received += 1;
			}
			HAL_UARTEx_ReceiveToIdle_DMA(&huart3, DBUS_rx_buf,36);
		}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim==&htim1)
	{

    l_up=l_up_real*436.85;
    R__up=R__up_real*436.85;
		PID_Calc(&speed_pid[0],Engine[0].speeed,L1);
		PID_Calc(&speed_pid[1],Engine[1].speeed,L2);
		PID_Calc(&speed_pid[2],Engine[2].speeed,R1);
		PID_Calc(&speed_pid[3],Engine[3].speeed,R2);

    //升降机构pid计算
    PID_Calc(&position_pid[0],Engine[4].angle_multiround,l_up);
    PID_Calc(&position_pid[1],Engine[5].angle_multiround,R__up);
    PID_Calc(&speed_pid[4],Engine[4].speeed,position_pid[0].Output);
    PID_Calc(&speed_pid[5],Engine[5].speeed,position_pid[1].Output);

    boxSend1(speed_pid[0].Output,speed_pid[1].Output,speed_pid[2].Output,speed_pid[3].Output);
    boxSend2(speed_pid[4].Output,speed_pid[5].Output,0,0);
	}

}

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
