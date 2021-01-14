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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void SystemClock_Config(void);
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
#define FLAG_2_Pin GPIO_PIN_13
#define FLAG_2_GPIO_Port GPIOC
#define BUSY_2_Pin GPIO_PIN_14
#define BUSY_2_GPIO_Port GPIOC
#define STAT_Pin GPIO_PIN_15
#define STAT_GPIO_Port GPIOC
#define CS_2_Pin GPIO_PIN_0
#define CS_2_GPIO_Port GPIOH
#define BUSY_1_Pin GPIO_PIN_1
#define BUSY_1_GPIO_Port GPIOH
#define _5V_MON_Pin GPIO_PIN_0
#define _5V_MON_GPIO_Port GPIOA
#define FLAG_1_Pin GPIO_PIN_1
#define FLAG_1_GPIO_Port GPIOA
#define CS_1_Pin GPIO_PIN_4
#define CS_1_GPIO_Port GPIOA
#define PWM_MON_Pin GPIO_PIN_5
#define PWM_MON_GPIO_Port GPIOA
#define I_MON_Pin GPIO_PIN_6
#define I_MON_GPIO_Port GPIOA
#define EXT_1_Pin GPIO_PIN_7
#define EXT_1_GPIO_Port GPIOA
#define EXT_2_Pin GPIO_PIN_0
#define EXT_2_GPIO_Port GPIOB
#define FLAG_6_Pin GPIO_PIN_1
#define FLAG_6_GPIO_Port GPIOB
#define BUSY_6_Pin GPIO_PIN_2
#define BUSY_6_GPIO_Port GPIOB
#define CS_6_Pin GPIO_PIN_12
#define CS_6_GPIO_Port GPIOB
#define FLAG_5_Pin GPIO_PIN_8
#define FLAG_5_GPIO_Port GPIOA
#define BUSY_5_Pin GPIO_PIN_9
#define BUSY_5_GPIO_Port GPIOA
#define CS_5_Pin GPIO_PIN_10
#define CS_5_GPIO_Port GPIOA
#define RGB_STAT_Pin GPIO_PIN_4
#define RGB_STAT_GPIO_Port GPIOB
#define FLAG_4_Pin GPIO_PIN_5
#define FLAG_4_GPIO_Port GPIOB
#define BUSY_4_Pin GPIO_PIN_6
#define BUSY_4_GPIO_Port GPIOB
#define CS_4_Pin GPIO_PIN_7
#define CS_4_GPIO_Port GPIOB
#define BUSY_3_Pin GPIO_PIN_3
#define BUSY_3_GPIO_Port GPIOH
#define FLAG_3_Pin GPIO_PIN_8
#define FLAG_3_GPIO_Port GPIOB
#define CS_3_Pin GPIO_PIN_9
#define CS_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
