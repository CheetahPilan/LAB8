/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char ADatabuffer[32] = {0};
char BDatabuffer[32] = {0};
int16_t inputchar = 0;
u_int8_t State_Now = 0;
uint64_t Time = 0;
uint64_t Delay = 500;
float Frequency = 1;
float Period;
int Mode = 0;
uint8_t Swtich[2]={0};
enum State
{
	State_Start = 000,
	State_Menu = 001,
	State_Menu1 = 010,
	State_Menu2 = 011,
	State_Choose_Menu = 100,
	State_Choose_Menu1 = 101,
	State_Choose_Menu2 = 110
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int16_t UARTRecieve();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  char Menu1[] = "a:SpeedLED Up\r\ns:SpeedLED Down\r\nd:On/Off\r\nx:Exit\r\n";
  char Menu2[] = "Show button status\r\nx:Exit\r\n";
  char Menu[] = "Menu1[0]\r\nMenu2[1]\r\n";
  char Buttonpress[] = "Press\r\n";
  char ButtonUnpress[] = "Unpress\r\n";


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_UART_Receive_IT(&huart2, (u_int8_t*)BDatabuffer, 32);
	  inputchar = UARTRecieve();
	  Swtich[0]=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if(inputchar > 0)
	  {
		  sprintf(ADatabuffer,"%c\r\n",inputchar);
		  HAL_UART_Transmit(&huart2, (u_int8_t*)BDatabuffer, strlen(BDatabuffer),10);
	  }
	  switch (State_Now)
	  {
		case State_Start:
			HAL_UART_Transmit(&huart2, (u_int8_t*)Menu, strlen(Menu),10);
			State_Now = State_Menu;
			break;
		case State_Menu:
			if(inputchar == '0')
			{
				State_Now = State_Menu1;
			}
			else if(inputchar == '1')
			{
				State_Now = State_Menu2;
			}
			else
			{
				State_Now = State_Menu;
			}
			break;
		case State_Menu1:
			HAL_UART_Transmit(&huart2, (u_int8_t*)Menu1, strlen(Menu1),10);
			State_Now = State_Choose_Menu1;
			break;
		case State_Menu2:
			HAL_UART_Transmit(&huart2, (u_int8_t*)Menu2, strlen(Menu2),10);
			State_Now = State_Choose_Menu2;
			break;
		case State_Choose_Menu1:
			if(inputchar == 'a')
			{
				Frequency += 1;
				Period = 1/Frequency;
				Delay = Period*500;
			}
			else if(inputchar == 's')
			{
				Frequency -= 1;
				Period = 1/Frequency;
				Delay = Period*500;
				if(Frequency < 0)
				{
					Frequency = 0;
				}
			}
			else if(inputchar == 'd')
			{
				if(Mode == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0)
				{
					Mode = 1;
				}
				else if(Mode == 1)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
					Mode = 0;
				}
			}
			else if(inputchar == 'x')
			{
				State_Now = State_Start;
			}
			break;
		case State_Choose_Menu2:
			if(Swtich[0] == 0 && Swtich[1]==1)
			{
				HAL_UART_Transmit(&huart2, (u_int8_t*)Buttonpress, strlen(Buttonpress),10);
			}
			else if (Swtich[0] == 1 && Swtich[1]==0)
			{
				HAL_UART_Transmit(&huart2, (u_int8_t*)ButtonUnpress, strlen(ButtonUnpress),10);
			}
			Swtich[1]=Swtich[0];
			if(inputchar == 'x')
			{
				State_Now = State_Start;
			}
			break;
	}
	  if(Mode == 1)
	  {
		  if(HAL_GetTick() - Time > Delay)
		  	 {
		  	  	Time = HAL_GetTick();
		  	  	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  	 }
	  }


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int16_t UARTRecieve()
{
	static uint32_t DataPosition = 0;
	int16_t Data = -1;
	if(huart2.RxXferSize - huart2.RxXferCount != DataPosition)
	{
		Data = BDatabuffer[DataPosition];
		DataPosition = (DataPosition + 1) % huart2.RxXferSize;
	}
	return Data;

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	sprintf(ADatabuffer, "%s\r\n", BDatabuffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)ADatabuffer, strlen(ADatabuffer), 1000);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
