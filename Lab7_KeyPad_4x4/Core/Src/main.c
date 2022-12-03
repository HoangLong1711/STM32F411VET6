/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_text.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define C1_PORT GPIOB
#define C1_PIN GPIO_PIN_0

#define C2_PORT GPIOB
#define C2_PIN GPIO_PIN_2

#define C3_PORT GPIOE
#define C3_PIN GPIO_PIN_8

#define C4_PORT GPIOE
#define C4_PIN GPIO_PIN_10

#define R1_PORT GPIOE
#define R1_PIN GPIO_PIN_12

#define R2_PORT GPIOE
#define R2_PIN GPIO_PIN_14

#define R3_PORT GPIOB
#define R3_PIN GPIO_PIN_10

#define R4_PORT GPIOB
#define R4_PIN GPIO_PIN_12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char key;
char stringArray[40] = "";
int count = 0;
char read_keypad(void) {
	/* Make ROW 1 LOW and all other ROWs HIGH */

	HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_SET);

	if ((HAL_GPIO_ReadPin(R1_PORT, R1_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R1_PORT, R1_PIN)) == 0)
			;
		return '1';
	}
	if ((HAL_GPIO_ReadPin(R2_PORT, R2_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R2_PORT, R2_PIN)) == 0)
			;
		return '4';
	}
	if ((HAL_GPIO_ReadPin(R3_PORT, R3_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R3_PORT, R3_PIN)) == 0)
			;
		return '7';
	}
	if ((HAL_GPIO_ReadPin(R4_PORT, R4_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R4_PORT, R4_PIN)) == 0)
			;
		return '*';
	}

	HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_SET);

	if ((HAL_GPIO_ReadPin(R1_PORT, R1_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R1_PORT, R1_PIN)) == 0)
			;
		return '2';
	}
	if ((HAL_GPIO_ReadPin(R2_PORT, R2_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R2_PORT, R2_PIN)) == 0)
			;
		return '5';
	}
	if ((HAL_GPIO_ReadPin(R3_PORT, R3_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R3_PORT, R3_PIN)) == 0)
			;
		return '8';
	}
	if ((HAL_GPIO_ReadPin(R4_PORT, R4_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R4_PORT, R4_PIN)) == 0)
			;
		return '0';
	}

	HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_SET);  // Pull the R4 High
	if ((HAL_GPIO_ReadPin(R1_PORT, R1_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R1_PORT, R1_PIN)) == 0)
			;
		return '3';
	}
	if ((HAL_GPIO_ReadPin(R2_PORT, R2_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R2_PORT, R2_PIN)) == 0)
			;
		return '6';
	}
	if ((HAL_GPIO_ReadPin(R3_PORT, R3_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R3_PORT, R3_PIN)) == 0)
			;
		return '9';
	}
	if ((HAL_GPIO_ReadPin(R4_PORT, R4_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R4_PORT, R4_PIN)) == 0)
			;
		return '#';
	}

	HAL_GPIO_WritePin(C1_PORT, C1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin(C2_PORT, C2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin(C3_PORT, C3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin(C4_PORT, C4_PIN, GPIO_PIN_RESET);  // Pull the R4 High
	if ((HAL_GPIO_ReadPin(R1_PORT, R1_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R1_PORT, R1_PIN)) == 0)
			;
		return 'A';
	}
	if ((HAL_GPIO_ReadPin(R2_PORT, R2_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R2_PORT, R2_PIN)) == 0)
			;
		return 'B';
	}
	if ((HAL_GPIO_ReadPin(R3_PORT, R3_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R3_PORT, R3_PIN)) == 0)
			;
		return 'C';
	}
	if ((HAL_GPIO_ReadPin(R4_PORT, R4_PIN)) == 0)
			{
		while ((HAL_GPIO_ReadPin(R4_PORT, R4_PIN)) == 0)
			;
		return 'D';
	}
	return '/';
}
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
  /* USER CODE BEGIN 2 */
  lcd_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		key = read_keypad();
		if (key != '/') {
			stringArray[count] = key;
			lcd_puts(1, 0, (int8_t*) stringArray);
			count++;
		}
		lcd_puts(0, 0, (int8_t*) "Test LCD 1602");
		HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_1
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE12 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD13 PD15 PD1
                           PD3 PD5 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_1
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
