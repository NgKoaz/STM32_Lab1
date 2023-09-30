/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int b[10] = {1, 79, 18, 6, 76, 36, 32, 15, 0, 4};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void display7SEG(int num);
void TrafficLightManagement(void);
void Light_Vertical(int status);
void Light_Horizontal(int status);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  TrafficLightManagement();

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HLED_Red_Pin|HLED_Yellow_Pin|HLED_Green_Pin|VLED_Red_Pin
                          |VLED_Yellow_Pin|VLED_Green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEQ7_A_Pin|SEQ7_B_Pin|SEQ7_C_Pin|SEQ7_D1_Pin
                          |SEQ7_E1_Pin|SEQ7_F1_Pin|SEQ7_G1_Pin|SEQ7_D_Pin
                          |SEQ7_E_Pin|SEQ7_G_Pin|SEQ7_A1_Pin|SEQ7_B1_Pin
                          |SEQ7_C1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HLED_Red_Pin HLED_Yellow_Pin HLED_Green_Pin VLED_Red_Pin
                           VLED_Yellow_Pin VLED_Green_Pin */
  GPIO_InitStruct.Pin = HLED_Red_Pin|HLED_Yellow_Pin|HLED_Green_Pin|VLED_Red_Pin
                          |VLED_Yellow_Pin|VLED_Green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEQ7_A_Pin SEQ7_B_Pin SEQ7_C_Pin SEQ7_D_Pin
                           SEQ7_E_Pin SEQ7_G_Pin */
  GPIO_InitStruct.Pin = SEQ7_A_Pin|SEQ7_B_Pin|SEQ7_C_Pin|SEQ7_D_Pin
                          |SEQ7_E_Pin|SEQ7_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SEQ7_D1_Pin SEQ7_E1_Pin SEQ7_F1_Pin SEQ7_G1_Pin
                           SEQ7_A1_Pin SEQ7_B1_Pin SEQ7_C1_Pin */
  GPIO_InitStruct.Pin = SEQ7_D1_Pin|SEQ7_E1_Pin|SEQ7_F1_Pin|SEQ7_G1_Pin
                          |SEQ7_A1_Pin|SEQ7_B1_Pin|SEQ7_C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SEQ7_F_Pin */
  GPIO_InitStruct.Pin = SEQ7_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SEQ7_F_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void display7SEG(int num)
{
	if (num > 9 || num < 0) return;
	int sel = b[num];

	HAL_GPIO_WritePin(SEQ7_G_GPIO_Port, SEQ7_G_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_F_GPIO_Port, SEQ7_F_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_E_GPIO_Port, SEQ7_E_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_D_GPIO_Port, SEQ7_D_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_C_GPIO_Port, SEQ7_C_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_B_GPIO_Port, SEQ7_B_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_A_GPIO_Port, SEQ7_A_Pin, sel % 2);
}

void display7SEG_1(int num)
{
	if (num > 9 || num < 0) return;
	int sel = b[num];

	HAL_GPIO_WritePin(SEQ7_G1_GPIO_Port, SEQ7_G1_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_F1_GPIO_Port, SEQ7_F1_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_E1_GPIO_Port, SEQ7_E1_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_D1_GPIO_Port, SEQ7_D1_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_C1_GPIO_Port, SEQ7_C1_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_B1_GPIO_Port, SEQ7_B1_Pin, sel % 2);
	sel /= 2;
	HAL_GPIO_WritePin(SEQ7_A1_GPIO_Port, SEQ7_A1_Pin, sel % 2);
}

void TrafficLightManagement(void)
{
	int statusVertical = 0; // 0 - Green | 1 - Yellow | 2 - Red
	int statusHorizontal = 2; // 0 - Green | 1 - Yellow | 2 - Red
	int NumToDisplay_Vertical = 0;
	int NumToDisplay_Horizontal = 0;
	for (int i = 9; i >= 0; i--) {
		//Vertical
		switch (i) {
		case 9:
			statusVertical = 0;
			NumToDisplay_Vertical = 4;
			break;
		case 4:
			statusVertical = 2;
			NumToDisplay_Vertical = 2;
			break;
		case 1:
			statusVertical = 1;
			NumToDisplay_Vertical = 1;
			break;
		}

		//Horizontal
		switch (i) {
		case 9:
			statusHorizontal = 2;
			NumToDisplay_Horizontal = 2;
			break;
		case 6:
			statusHorizontal = 1;
			NumToDisplay_Horizontal = 1;
			break;
		case 4:
			statusHorizontal = 0;
			NumToDisplay_Horizontal = 4;
			break;
		}
		display7SEG(NumToDisplay_Vertical--);
		display7SEG_1(NumToDisplay_Horizontal--);
		Light_Vertical(statusVertical);
		Light_Horizontal(statusHorizontal);
		HAL_Delay(1000);
	}
}

void Light_Vertical(int status) {
	switch (status) {
	case 0:
		HAL_GPIO_WritePin(VLED_Red_GPIO_Port, VLED_Red_Pin, RESET);
		HAL_GPIO_WritePin(VLED_Yellow_GPIO_Port, VLED_Yellow_Pin, SET);
		HAL_GPIO_WritePin(VLED_Green_GPIO_Port, VLED_Green_Pin, SET);
		break;
	case 1:
		HAL_GPIO_WritePin(VLED_Red_GPIO_Port, VLED_Red_Pin, SET);
		HAL_GPIO_WritePin(VLED_Yellow_GPIO_Port, VLED_Yellow_Pin, RESET);
		HAL_GPIO_WritePin(VLED_Green_GPIO_Port, VLED_Green_Pin, SET);
		break;
	case 2:
		HAL_GPIO_WritePin(VLED_Red_GPIO_Port, VLED_Red_Pin, SET);
		HAL_GPIO_WritePin(VLED_Yellow_GPIO_Port, VLED_Yellow_Pin, SET);
		HAL_GPIO_WritePin(VLED_Green_GPIO_Port, VLED_Green_Pin, RESET);
		break;
	}
}

void Light_Horizontal(int status) {
	switch (status) {
	case 0:
		HAL_GPIO_WritePin(HLED_Red_GPIO_Port, HLED_Red_Pin, RESET);
		HAL_GPIO_WritePin(HLED_Yellow_GPIO_Port, HLED_Yellow_Pin, SET);
		HAL_GPIO_WritePin(HLED_Green_GPIO_Port, HLED_Green_Pin, SET);
		break;
	case 1:
		HAL_GPIO_WritePin(HLED_Red_GPIO_Port, HLED_Red_Pin, SET);
		HAL_GPIO_WritePin(HLED_Yellow_GPIO_Port, HLED_Yellow_Pin, RESET);
		HAL_GPIO_WritePin(HLED_Green_GPIO_Port, HLED_Green_Pin, SET);
		break;
	case 2:
		HAL_GPIO_WritePin(HLED_Red_GPIO_Port, HLED_Red_Pin, SET);
		HAL_GPIO_WritePin(HLED_Yellow_GPIO_Port, HLED_Yellow_Pin, SET);
		HAL_GPIO_WritePin(HLED_Green_GPIO_Port, HLED_Green_Pin, RESET);
		break;
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
