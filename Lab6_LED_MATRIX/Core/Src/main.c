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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLK_GPIO GPIOD
#define CLK_PIN GPIO_PIN_13

#define LATCH_GPIO GPIOD
#define LATCH_PIN GPIO_PIN_14

#define DATA_GPIO GPIOD
#define DATA_PIN GPIO_PIN_15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//int rowsSelect[8] = {0x7F,0xBF,0xDF,0xEF,0xF7,0xFB,0xFD,0xFE};
//int colsData[][8] = {
//		{0x00,0x66,0x99,0x81,0x81,0x42,0x24,0x18},
//		{0x3C,0x7E,0xE2,0xC0,0xC0,0xE2,0x7E,0x3C},
//		{ 0x7E,0xFF,0x99,0x18,0x18,0x18,0x18,0x18 },
//		{ 0xC3,0xC3,0xC3,0xC3,0xC3,0xC3,0x66,0x3C }
//
//};

int rowsData[][8] = {
		{ 0xC7, 0x83, 0x81, 0xC0, 0xC0, 0x81, 0x83, 0xC7 }, // Heart
		{ 0xBE, 0x80, 0x80, 0xB6, 0xA2, 0xBE, 0x9C, 0xFF }, //  (E) 37
		{ 0xBE, 0x80, 0x80, 0xBE, 0xFE, 0xFC, 0xF8, 0xFF }, //  (L) 44
		{ 0x80, 0x80, 0xCF, 0xE7, 0xF3, 0x80, 0x80, 0xFF }, //  (N) 46
};
int colsSelect[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CLK()
{
	HAL_GPIO_WritePin(CLK_GPIO, CLK_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CLK_GPIO, CLK_PIN, GPIO_PIN_RESET);
}

void LATCH()
{
	HAL_GPIO_WritePin(LATCH_GPIO, LATCH_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LATCH_GPIO, LATCH_PIN, GPIO_PIN_RESET);
}

void DATA(int value)
{
	if(1==value)
	{
		HAL_GPIO_WritePin(DATA_GPIO, DATA_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(DATA_GPIO, DATA_PIN, GPIO_PIN_RESET);
	}
	CLK();
}

void pushHEX(int hexValue)
{
	for (int i = 0; i < 8; i++) {
		if (hexValue & 0x01) {
			DATA(1);
		} else {
			DATA(0);
		}
		hexValue >>= 1;
	}
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//stand still in one place
//		for (int i = 0; i <= 7; i++) {
//			pushHEX(colsSelect[i]);
//			pushHEX(colsSelect[i]);
//			pushHEX(colsSelect[i]);
//			pushHEX(rowsData[0][i]);
//			LATCH();
//			HAL_Delay(1);
//		}

	  // {\0} {\0}  {L}
	      // nhan L
	      for(int step =0; step<8;step++)
	      {
	        for(int i = 0;i<= step;i++)
	        {
	          pushHEX(colsSelect[7-step+i]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(rowsData[2][i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        HAL_Delay(1);
	      }
	      //==============

	      //==============
	      // {\0} {L}  {E}
	      int count = 0;
	      //nhan E
	      for(int step1 =0; step1<8;step1++)
	      {
	        for(int j = 0;j<= count;j++)
	        {
	          pushHEX(colsSelect[7-step1+j]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(rowsData[1][j]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day L
	        for(int i = 0; i < (7-step1); i++)
	        {
	          pushHEX(colsSelect[6-i-step1]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(rowsData[2][7-i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Nhan L
	        for(int a = 0;a<= count;a++)
	        {
	          pushHEX(0x00);
	          pushHEX(colsSelect[7-step1+a]);
	          pushHEX(0x00);
	          pushHEX(rowsData[2][a]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        count++;
	        HAL_Delay(1);
	      }
	      //==============

	      //==============
	      // {L}2  {E}1  {N}3
	      count = 0;
	      // Nhan N
	      for(int step2 =0; step2<8;step2++)
	      {
	        for(int j = 0;j<= count;j++)
	        {
	          pushHEX(colsSelect[7-step2+j]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(rowsData[3][j]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day E
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(rowsData[1][7-i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        //Nhan E
	        for(int a = 0;a<= count;a++)
	        {
	          pushHEX(0x00);
	          pushHEX(colsSelect[7-step2+a]);
	          pushHEX(0x00);
	          pushHEX(rowsData[1][a]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day L
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(0x00);
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(0x00);
	          pushHEX(rowsData[2][7-i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Nhan L
	        for(int a = 0;a<= count;a++)
	        {
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(colsSelect[7-step2+a]);
	          pushHEX(rowsData[2][a]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        count++;
	        HAL_Delay(1);
	      }
	      //==============
	      //==============
	      // 2  {E}1  {N}3  {0}
	      count = 0;

	      for(int step2 =0; step2<8;step2++)
	      {
	        //Nhan Rong
	        for(int j = 0;j<= count;j++)
	        {
	          pushHEX(colsSelect[7-step2+j]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(0xFF);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day N
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(rowsData[3][7-i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Nhan N
	        for(int a = 0;a<= count;a++)
	        {
	          pushHEX(0x00);
	          pushHEX(colsSelect[7-step2+a]);
	          pushHEX(0x00);
	          pushHEX(rowsData[3][a]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day E
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(0x00);
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(0x00);
	          pushHEX(rowsData[1][7-i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Nhan E
	        for(int a = 0;a<= count;a++)
	        {
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(colsSelect[7-step2+a]);
	          pushHEX(rowsData[1][a]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day L
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(rowsData[2][7-i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        count++;
	        HAL_Delay(1);
	      }
	      //==============
	      //==============
	      // 1  {N}3  {0}  {0}
	      count = 0;
	      for(int step2 =0; step2<8;step2++)
	      {
	        //Nhan Rong
	        for(int j = 0;j<= count;j++)
	        {
	          pushHEX(colsSelect[7-step2+j]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(0xFF);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day Rong
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(0xFF);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Nhan Rong
	        for(int a = 0;a<= count;a++)
	        {
	          pushHEX(0x00);
	          pushHEX(colsSelect[7-step2+a]);
	          pushHEX(0x00);
	          pushHEX(0xFF);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day N
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(0x00);
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(0x00);
	          pushHEX(rowsData[3][7-i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Nhan N
	        for(int a = 0;a<= count;a++)
	        {
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(colsSelect[7-step2+a]);
	          pushHEX(rowsData[3][a]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day E
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(rowsData[1][7-i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        count++;
	        HAL_Delay(1);
	      }
	      //==============
	      //==============
	      // 3  {0}  {0} {0}
	      count = 0;
	      for(int step2 =0; step2<8;step2++)
	      {
	        //Nhan Rong
	        for(int j = 0;j<= count;j++)
	        {
	          pushHEX(colsSelect[7-step2+j]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(0xFF);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day Rong
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(0xFF);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Nhan Rong
	        for(int a = 0;a<= count;a++)
	        {
	          pushHEX(0x00);
	          pushHEX(colsSelect[7-step2+a]);
	          pushHEX(0x00);
	          pushHEX(0xFF);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day Rong
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(0x00);
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(0x00);
	          pushHEX(0xFF);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // nhan Rong
	        for(int a = 0;a<= count;a++)
	        {
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(colsSelect[7-step2+a]);
	          pushHEX(0xFF);
	          LATCH();
	          HAL_Delay(1);
	        }
	        // Day N
	        for(int i = 0; i < (7-step2); i++)
	        {
	          pushHEX(0x00);
	          pushHEX(0x00);
	          pushHEX(colsSelect[6-i-step2]);
	          pushHEX(rowsData[3][7-i]);
	          LATCH();
	          HAL_Delay(1);
	        }
	        count++;
	        HAL_Delay(1);
	      }
	      //==============

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
