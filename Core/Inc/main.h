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
#define PIN1_Pin GPIO_PIN_0
#define PIN1_GPIO_Port GPIOA
#define PIN2_Pin GPIO_PIN_1
#define PIN2_GPIO_Port GPIOA
#define PIN3_Pin GPIO_PIN_2
#define PIN3_GPIO_Port GPIOA
#define PIN4_Pin GPIO_PIN_3
#define PIN4_GPIO_Port GPIOA
#define PIN5_Pin GPIO_PIN_4
#define PIN5_GPIO_Port GPIOA
#define PIN6_Pin GPIO_PIN_5
#define PIN6_GPIO_Port GPIOA
#define PIN7_Pin GPIO_PIN_6
#define PIN7_GPIO_Port GPIOA
#define PIN8_Pin GPIO_PIN_7
#define PIN8_GPIO_Port GPIOA
#define PIN9_Pin GPIO_PIN_0
#define PIN9_GPIO_Port GPIOB
#define PIN10_Pin GPIO_PIN_1
#define PIN10_GPIO_Port GPIOB
#define PIN11_Pin GPIO_PIN_2
#define PIN11_GPIO_Port GPIOB
#define PIN12_Pin GPIO_PIN_3
#define PIN12_GPIO_Port GPIOB
#define PIN13_Pin GPIO_PIN_4
#define PIN13_GPIO_Port GPIOB
#define PIN14_Pin GPIO_PIN_5
#define PIN14_GPIO_Port GPIOB
#define PIN15_Pin GPIO_PIN_6
#define PIN15_GPIO_Port GPIOB
#define PIN16_Pin GPIO_PIN_7
#define PIN16_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
