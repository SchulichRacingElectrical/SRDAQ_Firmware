/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define A10_Pin GPIO_PIN_0
#define A10_GPIO_Port GPIOC
#define A11_Pin GPIO_PIN_1
#define A11_GPIO_Port GPIOC
#define A12_Pin GPIO_PIN_2
#define A12_GPIO_Port GPIOC
#define A13_Pin GPIO_PIN_3
#define A13_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOA
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOA
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOA
#define A6_Pin GPIO_PIN_6
#define A6_GPIO_Port GPIOA
#define A7_Pin GPIO_PIN_7
#define A7_GPIO_Port GPIOA
#define A14_Pin GPIO_PIN_4
#define A14_GPIO_Port GPIOC
#define A15_Pin GPIO_PIN_5
#define A15_GPIO_Port GPIOC
#define A8_Pin GPIO_PIN_0
#define A8_GPIO_Port GPIOB
#define A9_Pin GPIO_PIN_1
#define A9_GPIO_Port GPIOB
#define PWM_2_Pin GPIO_PIN_2
#define PWM_2_GPIO_Port GPIOB
#define PWM_1_Pin GPIO_PIN_10
#define PWM_1_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_2
#define SD_CS_GPIO_Port GPIOG
#define PWM_3_Pin GPIO_PIN_6
#define PWM_3_GPIO_Port GPIOC
#define PWM_4_Pin GPIO_PIN_7
#define PWM_4_GPIO_Port GPIOC
#define GPS_Ready_Pin GPIO_PIN_10
#define GPS_Ready_GPIO_Port GPIOG
#define GPS_nReset_Pin GPIO_PIN_11
#define GPS_nReset_GPIO_Port GPIOG
#define SD_SCK_Pin GPIO_PIN_3
#define SD_SCK_GPIO_Port GPIOB
#define SD_MISO_Pin GPIO_PIN_4
#define SD_MISO_GPIO_Port GPIOB
#define SD_MOSI_Pin GPIO_PIN_5
#define SD_MOSI_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB
#define LOG_Switch_Pin GPIO_PIN_0
#define LOG_Switch_GPIO_Port GPIOE
#define SD_Detect_Pin GPIO_PIN_1
#define SD_Detect_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
