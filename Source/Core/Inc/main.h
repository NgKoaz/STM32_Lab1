/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define VLED_Red_Pin GPIO_PIN_5
#define VLED_Red_GPIO_Port GPIOA
#define VLED_Yellow_Pin GPIO_PIN_6
#define VLED_Yellow_GPIO_Port GPIOA
#define VLED_Green_Pin GPIO_PIN_7
#define VLED_Green_GPIO_Port GPIOA
#define SEQ7_A_Pin GPIO_PIN_0
#define SEQ7_A_GPIO_Port GPIOB
#define SEQ7_B_Pin GPIO_PIN_1
#define SEQ7_B_GPIO_Port GPIOB
#define SEQ7_C_Pin GPIO_PIN_2
#define SEQ7_C_GPIO_Port GPIOB
#define SEQ7_D1_Pin GPIO_PIN_10
#define SEQ7_D1_GPIO_Port GPIOB
#define SEQ7_E1_Pin GPIO_PIN_11
#define SEQ7_E1_GPIO_Port GPIOB
#define SEQ7_F1_Pin GPIO_PIN_12
#define SEQ7_F1_GPIO_Port GPIOB
#define SEQ7_G1_Pin GPIO_PIN_13
#define SEQ7_G1_GPIO_Port GPIOB
#define HLED_Red_Pin GPIO_PIN_8
#define HLED_Red_GPIO_Port GPIOA
#define HLED_Yellow_Pin GPIO_PIN_9
#define HLED_Yellow_GPIO_Port GPIOA
#define HLED_Green_Pin GPIO_PIN_10
#define HLED_Green_GPIO_Port GPIOA
#define SEQ7_D_Pin GPIO_PIN_3
#define SEQ7_D_GPIO_Port GPIOB
#define SEQ7_E_Pin GPIO_PIN_4
#define SEQ7_E_GPIO_Port GPIOB
#define SEQ7_F_Pin GPIO_PIN_5
#define SEQ7_F_GPIO_Port GPIOB
#define SEQ7_G_Pin GPIO_PIN_6
#define SEQ7_G_GPIO_Port GPIOB
#define SEQ7_A1_Pin GPIO_PIN_7
#define SEQ7_A1_GPIO_Port GPIOB
#define SEQ7_B1_Pin GPIO_PIN_8
#define SEQ7_B1_GPIO_Port GPIOB
#define SEQ7_C1_Pin GPIO_PIN_9
#define SEQ7_C1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
