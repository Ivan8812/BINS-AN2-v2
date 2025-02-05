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
#include "stm32h5xx_hal.h"

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
#define uart_gnss huart11
#define uart_mti huart2
#define uart_232 huart4
#define uart_485 huart5
#define uart_422 huart6
#define i2c_bmp hi2c1
#define i2c_temp hi2c2
#define RST_IMU_Pin GPIO_PIN_2
#define RST_IMU_GPIO_Port GPIOC
#define SYNC_IMU_Pin GPIO_PIN_3
#define SYNC_IMU_GPIO_Port GPIOC
#define RST_GNSS_Pin GPIO_PIN_4
#define RST_GNSS_GPIO_Port GPIOA
#define PPS_GNSS_Pin GPIO_PIN_5
#define PPS_GNSS_GPIO_Port GPIOA
#define ANT_PWR_EN_Pin GPIO_PIN_4
#define ANT_PWR_EN_GPIO_Port GPIOC
#define ANT_PWR_FAULT_Pin GPIO_PIN_5
#define ANT_PWR_FAULT_GPIO_Port GPIOC
#define BS_HEAT_Pin GPIO_PIN_0
#define BS_HEAT_GPIO_Port GPIOB
#define IMU_HEAT_Pin GPIO_PIN_8
#define IMU_HEAT_GPIO_Port GPIOA
#define LED_MCU_Pin GPIO_PIN_5
#define LED_MCU_GPIO_Port GPIOB
#define DRVR_PWR_FAULT_Pin GPIO_PIN_6
#define DRVR_PWR_FAULT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
