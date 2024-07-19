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
#include "stm32g0xx_hal.h"

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
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define POTI_Pin GPIO_PIN_1
#define POTI_GPIO_Port GPIOA
#define TASTE_L_Pin GPIO_PIN_4
#define TASTE_L_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_6
#define DIR_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_7
#define STEP_GPIO_Port GPIOA
#define MS3_Pin GPIO_PIN_8
#define MS3_GPIO_Port GPIOA
#define MS2_Pin GPIO_PIN_9
#define MS2_GPIO_Port GPIOA
#define TASTE_R_Pin GPIO_PIN_6
#define TASTE_R_GPIO_Port GPIOC
#define MS1_Pin GPIO_PIN_10
#define MS1_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_11
#define EN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SLP_Pin GPIO_PIN_6
#define SLP_GPIO_Port GPIOD
#define RST_Pin GPIO_PIN_3
#define RST_GPIO_Port GPIOB
#define MVQ_R_Pin GPIO_PIN_8
#define MVQ_R_GPIO_Port GPIOB
#define MVQ_L_Pin GPIO_PIN_9
#define MVQ_L_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ADCx hadc1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
