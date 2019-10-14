/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum StatesEnum { off = 0, braking, braking_timeout};

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
bno055_calibration_data_t eeprom_read_bno_calibration_data();
void eeprom_write_bno_calibration_data(bno055_calibration_data_t data);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VBAT_SCALE_Pin GPIO_PIN_1
#define VBAT_SCALE_GPIO_Port GPIOA
#define WS2812_DIN_Pin GPIO_PIN_2
#define WS2812_DIN_GPIO_Port GPIOA
#define WS2812_PWR_ON_Pin GPIO_PIN_3
#define WS2812_PWR_ON_GPIO_Port GPIOA
#define TSC_G2_SAMPLING_Pin GPIO_PIN_4
#define TSC_G2_SAMPLING_GPIO_Port GPIOA
#define TSC_G2_IN_Pin GPIO_PIN_5
#define TSC_G2_IN_GPIO_Port GPIOA
#define LED_DOWN_Pin GPIO_PIN_6
#define LED_DOWN_GPIO_Port GPIOA
#define SX_RESET_Pin GPIO_PIN_1
#define SX_RESET_GPIO_Port GPIOB
#define SX_DIO0_EXTI8_Pin GPIO_PIN_8
#define SX_DIO0_EXTI8_GPIO_Port GPIOA
#define SX_DIO0_EXTI9_Pin GPIO_PIN_9
#define SX_DIO0_EXTI9_GPIO_Port GPIOA
#define SX_NSS_Pin GPIO_PIN_10
#define SX_NSS_GPIO_Port GPIOA
#define LDR_IN_Pin GPIO_PIN_15
#define LDR_IN_GPIO_Port GPIOA
#define LDR_PWR_Pin GPIO_PIN_4
#define LDR_PWR_GPIO_Port GPIOB
#define BNO_INT_EXTI5_Pin GPIO_PIN_5
#define BNO_INT_EXTI5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
