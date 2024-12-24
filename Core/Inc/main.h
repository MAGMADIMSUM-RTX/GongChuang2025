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
#include "stm32f4xx_hal.h"

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
#define EN1_Pin GPIO_PIN_0
#define EN1_GPIO_Port GPIOB
#define EN2_Pin GPIO_PIN_1
#define EN2_GPIO_Port GPIOB
#define EN3_Pin GPIO_PIN_2
#define EN3_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_0
#define DIR1_GPIO_Port GPIOD
#define DIR2_Pin GPIO_PIN_1
#define DIR2_GPIO_Port GPIOD
#define DIR3_Pin GPIO_PIN_2
#define DIR3_GPIO_Port GPIOD
#define DIR4_Pin GPIO_PIN_3
#define DIR4_GPIO_Port GPIOD
#define DIR5_Pin GPIO_PIN_4
#define DIR5_GPIO_Port GPIOD
#define EN4_Pin GPIO_PIN_3
#define EN4_GPIO_Port GPIOB
#define EN5_Pin GPIO_PIN_4
#define EN5_GPIO_Port GPIOB
#define STEP1_Pin GPIO_PIN_5
#define STEP1_GPIO_Port GPIOB
#define STEP2_Pin GPIO_PIN_6
#define STEP2_GPIO_Port GPIOB
#define STEP3_Pin GPIO_PIN_7
#define STEP3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
