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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	//--------------------------------------------------------------------------------------------------
	uint32_t ButtonTimeStamp = 0;

	GPIO_PinState Switch_1_State[2]; // now,last
	uint16_t LED1_frequency = 1; // 0.5Hz
	uint32_t LED1_TimeStamp = 0;

	GPIO_PinState Switch_2_State[2]; // now,last
	uint8_t Button2_Pressed = 0;

	GPIO_PinState Switch_3_State[2]; // now,last
	uint32_t LED3_TimeStamp = 0;
	uint8_t LED3_Mode = 1; //Normal
	uint16_t LED3_Time_1 = 1500;
	uint16_t LED3_Time_2 = 500;
	uint16_t LED3_ON = LED3_Time_2;
	uint16_t LED3_OFF = LED3_Time_1;

	GPIO_PinState Switch_4_State[2]; // now,last
	uint32_t LED4_TimeStamp = 0;
	uint16_t LED4_FullBilnk = 1;
	uint16_t LED4_TimeON = 0;
	uint16_t LED4_PWM = 0;

	//--------------------------------------------------------------------------------------------------
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		uint16_t LED1_HalfPeriod = 1000 / (LED1_frequency);
		if (HAL_GetTick() - ButtonTimeStamp >= 150) {
			// switch1 control
			Switch_1_State[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
			if (Switch_1_State[1] == GPIO_PIN_SET
					&& Switch_1_State[0] == GPIO_PIN_RESET) {
				// Change Half Period of LED 1
				switch (LED1_frequency) {
				case (1):
					LED1_frequency = 2;
					break;
				case (2):
					LED1_frequency = 4;
					break;
				case (4):
					LED1_frequency = 6;
					break;
				default:
					LED1_frequency = 1;
					break;
				}
			}
			Switch_1_State[1] = Switch_1_State[0];

			//switch2 control
			Switch_2_State[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
			if (Switch_2_State[1] == GPIO_PIN_SET
					&& Switch_2_State[0] == GPIO_PIN_RESET) {
				// Change LED 2
				if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
				} else {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
				}
			}
			Switch_2_State[1] = Switch_2_State[0];

			//switch3 control
			Switch_3_State[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
			if (Switch_3_State[1] == GPIO_PIN_SET
					&& Switch_3_State[0] == GPIO_PIN_RESET) {
				// Change LED 3
				if (LED3_Mode == 0) {
					LED3_ON = LED3_Time_1;
					LED3_OFF = LED3_Time_2;
					LED3_Mode = 1;
				} else {
					LED3_ON = LED3_Time_2;
					LED3_OFF = LED3_Time_1;
					LED3_Mode = 0;
				}

			}
			Switch_3_State[1] = Switch_3_State[0];

			//switch4 control
			Switch_4_State[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
			if (Switch_4_State[1] == GPIO_PIN_SET
					&& Switch_4_State[0] == GPIO_PIN_RESET) {
				// Change LED 4
				switch (LED4_PWM) {
				case (0):
					LED4_TimeON = LED4_FullBilnk * 0;
					LED4_PWM = 25;
				case (25):
					LED4_TimeON = LED4_FullBilnk * 25;
					LED4_PWM = 50;
				case (50):
					LED4_TimeON = LED4_FullBilnk * 50;
					LED4_PWM = 75;
				case (75):
					LED4_TimeON = LED4_FullBilnk * 75;
					LED4_PWM = 100;
				default:
					LED4_TimeON = LED4_FullBilnk;
					LED4_PWM = 0;
				}

			}
			Switch_4_State[1] = Switch_4_State[0];
		}
		// Run LED
		if (HAL_GetTick() - LED1_TimeStamp >= LED1_HalfPeriod) {
			LED1_TimeStamp = HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			}
		}

		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET)) {
			if (HAL_GetTick() - LED3_TimeStamp >= LED3_ON) {
				LED3_TimeStamp = HAL_GetTick();
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			}
		}
		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)) {
			if (HAL_GetTick() - LED3_TimeStamp >= LED3_OFF) {
				LED3_TimeStamp = HAL_GetTick();
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			}
		}

		if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET)) {
			if (HAL_GetTick() - LED4_TimeStamp
					>= LED4_TimeON) {
				LED4_TimeStamp = HAL_GetTick();
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
			}
		}
		if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)) {
			if (HAL_GetTick() - LED4_TimeStamp >= (LED4_FullBilnk - LED4_TimeON)) {
				LED4_TimeStamp = HAL_GetTick();
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			}
		}
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_7 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA7 PA9 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_7 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB3 PB4 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
