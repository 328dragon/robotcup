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
#define servos_6_Pin GPIO_PIN_5
#define servos_6_GPIO_Port GPIOE
#define servos_7_Pin GPIO_PIN_6
#define servos_7_GPIO_Port GPIOE
#define servos_5_Pin GPIO_PIN_2
#define servos_5_GPIO_Port GPIOA
#define servos_4_Pin GPIO_PIN_3
#define servos_4_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define servos_1_Pin GPIO_PIN_0
#define servos_1_GPIO_Port GPIOB
#define servos_2_Pin GPIO_PIN_1
#define servos_2_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOE
#define LCD_RS_Pin GPIO_PIN_12
#define LCD_RS_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_14
#define TFT_RST_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_10
#define TFT_CS_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOD
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOD
#define INT1_GRYO_Pin GPIO_PIN_2
#define INT1_GRYO_GPIO_Port GPIOD
#define INT1_GRYO_EXTI_IRQn EXTI2_IRQn
#define INT1_ACCEL_Pin GPIO_PIN_3
#define INT1_ACCEL_GPIO_Port GPIOD
#define INT1_ACCEL_EXTI_IRQn EXTI3_IRQn
#define CS1_GYRO_Pin GPIO_PIN_4
#define CS1_GYRO_GPIO_Port GPIOD
#define CS1_ACCEL_Pin GPIO_PIN_7
#define CS1_ACCEL_GPIO_Port GPIOD
#define servos_3_Pin GPIO_PIN_4
#define servos_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
