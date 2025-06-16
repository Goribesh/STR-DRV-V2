/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define DBG_RX_Pin GPIO_PIN_0
#define DBG_RX_GPIO_Port GPIOC
#define DBG_TX_Pin GPIO_PIN_1
#define DBG_TX_GPIO_Port GPIOC
#define STR_ADC_1_Pin GPIO_PIN_2
#define STR_ADC_1_GPIO_Port GPIOC
#define STR_ADC_2_Pin GPIO_PIN_3
#define STR_ADC_2_GPIO_Port GPIOC
#define STR_ADC_3_Pin GPIO_PIN_0
#define STR_ADC_3_GPIO_Port GPIOA
#define NTC_READ_Pin GPIO_PIN_1
#define NTC_READ_GPIO_Port GPIOA
#define PWR_LED_Pin GPIO_PIN_12
#define PWR_LED_GPIO_Port GPIOB
#define BUTTON_LED_Pin GPIO_PIN_13
#define BUTTON_LED_GPIO_Port GPIOB
#define BUTTON_1_Pin GPIO_PIN_15
#define BUTTON_1_GPIO_Port GPIOB
#define STR_CS_3_Pin GPIO_PIN_6
#define STR_CS_3_GPIO_Port GPIOC
#define STR_CS_2_Pin GPIO_PIN_7
#define STR_CS_2_GPIO_Port GPIOC
#define STR_CS_1_Pin GPIO_PIN_8
#define STR_CS_1_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PG_STR_3_Pin GPIO_PIN_15
#define PG_STR_3_GPIO_Port GPIOA
#define EN_STR_3_Pin GPIO_PIN_10
#define EN_STR_3_GPIO_Port GPIOC
#define PG_STR_2_Pin GPIO_PIN_12
#define PG_STR_2_GPIO_Port GPIOC
#define EN_STR_2_Pin GPIO_PIN_2
#define EN_STR_2_GPIO_Port GPIOD
#define EN_STR_1_Pin GPIO_PIN_4
#define EN_STR_1_GPIO_Port GPIOB
#define PG_STR_1_Pin GPIO_PIN_5
#define PG_STR_1_GPIO_Port GPIOB
#define CAN1_RS_Pin GPIO_PIN_7
#define CAN1_RS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
