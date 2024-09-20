/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PIN_MODO_Pin GPIO_PIN_13
#define PIN_MODO_GPIO_Port GPIOC
#define Reservado_Pin GPIO_PIN_0
#define Reservado_GPIO_Port GPIOH
#define ReservadoH1_Pin GPIO_PIN_1
#define ReservadoH1_GPIO_Port GPIOH
#define ADC_STR_Pin GPIO_PIN_1
#define ADC_STR_GPIO_Port GPIOA
#define ADC_CAM_Pin GPIO_PIN_2
#define ADC_CAM_GPIO_Port GPIOA
#define ENCODER_A_Pin GPIO_PIN_5
#define ENCODER_A_GPIO_Port GPIOA
#define AUX_Pin GPIO_PIN_6
#define AUX_GPIO_Port GPIOA
#define THR_AUTO_Pin GPIO_PIN_9
#define THR_AUTO_GPIO_Port GPIOA
#define STR_AUTO_Pin GPIO_PIN_10
#define STR_AUTO_GPIO_Port GPIOA
#define ReservadoA11_Pin GPIO_PIN_11
#define ReservadoA11_GPIO_Port GPIOA
#define ReservadoA12_Pin GPIO_PIN_12
#define ReservadoA12_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_3
#define ENCODER_B_GPIO_Port GPIOB
#define UART_IMU_Pin GPIO_PIN_6
#define UART_IMU_GPIO_Port GPIOB
#define UART_IMUB7_Pin GPIO_PIN_7
#define UART_IMUB7_GPIO_Port GPIOB
#define CAM_OUT_Pin GPIO_PIN_8
#define CAM_OUT_GPIO_Port GPIOB
#define MOSFET_IMU_Pin GPIO_PIN_9
#define MOSFET_IMU_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
