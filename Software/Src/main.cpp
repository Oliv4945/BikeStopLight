/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "touchsensing.h"
#include "tsc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "WS2812B.h"
#include "bno055.h"
#include "bno055_stm32.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint32_t eeprom_base_address = 0x08080000;
const uint8_t linacc_filter_weight = 10;
volatile StatesEnum state = off;
volatile bool bno_int_triggered = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	tsl_user_status_t tsl_status;
	uint8_t button_state = 0;
	float linacc_avg = 0;
	float linacc_long_term = 0;
	float linacc_decision = 0;
	volatile uint16_t ldr_value;
	uint8_t byte;

	// WS2812
	s_color led_color_green = { 0x0F, 0x00, 0x00 };
	s_color led_color_red = { 0x00, 0x0F, 0x00 };
	s_color led_color_red_brake = { 0x0F, 0x0F, 0x00 };
	s_color led_color_blue = { 0x00, 0x00, 0x0F};
	s_color led_color_off = { 0x00, 0x00, 0x00 };
	WS2812B leds(15);

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_LPTIM1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TSC_Init();
  MX_TOUCHSENSING_Init();
  /* USER CODE BEGIN 2 */
	// ADC Calibration
	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
	}

	// Power WS2812
	HAL_GPIO_WritePin(WS2812_PWR_ON_GPIO_Port, WS2812_PWR_ON_Pin, GPIO_PIN_SET);
	// Switch ON red LED
	HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_SET);
	// POWER LDR
	HAL_GPIO_WritePin(LDR_PWR_GPIO_Port, LDR_PWR_Pin, GPIO_PIN_SET);

	// Init BNO055
	bno055_assignI2C(&hi2c1);
	bno055_setup();
    bno055_setCalibrationData(eeprom_read_bno_calibration_data());
    bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
    // Get from EEPROM
	bno055_calibration_state_t bno_calib_state = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
	while ((bno_calib_state.sys != 3) || (bno_calib_state.mag != 3) ||  (bno_calib_state.gyro != 3) || (bno_calib_state.accel != 3)) {
		bno_calib_state = bno055_getCalibrationState();
		HAL_Delay(1000);
	}
	HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
	bno055_calibration_data_t bno_calib_data = bno055_getCalibrationData();
	eeprom_write_bno_calibration_data(bno_calib_data);

	// Enable only accelerometer interrupt and redirect it to interrupt pin. Cleat INT flags
	// TODO - Why it can not be set before calibration procedure ?
	bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
	bno055_setInterruptMask(0xC0);
	bno055_getInterruptStatus();
	bno055_setInterruptEnable(0xC0);
	// bno055_setInterruptAccelThresholds(0.1, 0, 1);
	// bno055_setInterruptNoOrSlowMotion(0, 1);
	bno055_setInterruptAccelSettings(0x1C);
	// TODO - Why calibration has to be written again ?
    bno055_setCalibrationData(bno_calib_data);
	bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);

	volatile uint8_t bno_int_statu = 0;
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	while(1) {
		if (bno_int_triggered == true) {
			bno_int_triggered = false;
			bno_int_statu = bno055_getInterruptStatus();
		}
		if (bno_int_statu > 0) {
			if ((bno_int_statu >> 7) == 1) {
				HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(WS2812_PWR_ON_GPIO_Port, WS2812_PWR_ON_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LDR_PWR_GPIO_Port, LDR_PWR_Pin, GPIO_PIN_RESET);
				bno055_setPowerMode(BNO055_POWER_MODE_SUSPEND);
				HAL_ADCEx_DisableVREFINT();
				HAL_ADC_DeInit(&hadc);
				HAL_ADC_MspDeInit(&hadc);
				HAL_I2C_MspDeInit(&hi2c1);
				HAL_SPI_MspDeInit(&hspi1);
				HAL_TSC_MspDeInit(&htsc);
				HAL_TIM_Base_MspDeInit(&htim2);
				GPIO_InitStruct.Pin = 0xFFFFU;
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				GPIO_InitStruct.Pin = 0xDBU;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				__HAL_RCC_GPIOA_CLK_DISABLE();
				__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
				HAL_SuspendTick();
				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
				HAL_ResumeTick();
			} else {
				HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_SET);
			}
			bno_int_statu += 1;
		}
		HAL_Delay(500);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* Execute STMTouch Driver state machine */
		/*
		 * tsl_status = tsl_user_Exec();
		 if (tsl_status != TSL_USER_STATUS_BUSY) {
		 switch (TSL_tkey_GetStateId()) {
		 case TSL_STATEID_DETECT:
		 leds.setColorAll(led_color_green);
		 HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_SET);
		 break;
		 case TSL_STATEID_PROX:
		 leds.setColorAll(led_color_blue);
		 HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
		 break;
		 default:
		 leds.setColorAll(led_color_red);
		 }
		 leds.update();
		 }
		 */
		volatile uint8_t bno_int_status = 0;
		bno_int_status = bno055_getInterruptStatus();
		if (bno_int_status > 0) {
			bno_int_status += 1;
		}


		ldr_value = get_ldr_value();

		uint8_t bno_error = bno055_getSystemError();

		bno055_vector_t linacc = bno055_getVectorLinearAccel();
		linacc_avg = ((linacc_avg * linacc_filter_weight) + linacc.z)
				/ (linacc_filter_weight + 1);
		linacc_long_term = linacc_long_term * 0.99 + linacc_avg * 0.01;
		linacc_decision = linacc_avg - linacc_long_term;

		// TODO: Should not update every loop to decrease battery consumption
		if ((ldr_value < 150) && (state == off)) {
			leds.setColorAll(led_color_red);
			leds.update();
		} else if ((ldr_value >= 150) && (state == off)) {
			leds.setColorAll(led_color_off);
			leds.update();
		}
		  if ((linacc_decision > 0.2) && (state == off)) {
			  // Start timer and switch on the LEDs
			  leds.setColorAll(led_color_off);
			  uint8_t leds_on = ((uint8_t)(linacc_avg * 8)+2) % 16;
			  leds.setColorRange(0, leds_on, led_color_red);
			  leds.update();
			  state = braking;
			  HAL_LPTIM_Counter_Start_IT(&hlptim1, 65000);
		  }
		  if ((linacc_decision > 0.3) && (state == braking)) {
			  // Reset timer breaking is still detected
			  __HAL_TIM_SET_COUNTER(&hlptim1, 0);
		  }

		  if (state == braking_timeout) {
			  // LED switching off id done regarding LDR
			  state = off;
		  }
		  HAL_Delay(10);


	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim) {
	state = braking_timeout;
}

bno055_calibration_data_t eeprom_read_bno_calibration_data() {
	bno055_calibration_data_t data;
	for (uint8_t i = 0; i < 5; i++) {
		memcpy(((uint32_t*) &data) + i, ((uint32_t *) eeprom_base_address) + i, 4);
	}
	memcpy(((uint32_t*) &data) + 5, ((uint32_t *) eeprom_base_address) + 5, 2);
	return data;
}

void eeprom_write_bno_calibration_data(bno055_calibration_data_t data) {
	uint32_t buffer_32; // TODO - Required ? https://community.st.com/s/question/0D50X00009XkhJbSAJ/hard-fault-writing-flash last post
	uint16_t buffer_16; // TODO - Required ? Can be replaced by buffer_32 depending of endianness
	if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK) {
		Error_Handler();
	}
	for (uint8_t i = 0; i < 5; i++) {
		memcpy(&buffer_32, ((uint32_t *) &data) + i, 4);
		if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, eeprom_base_address+ i*4, buffer_32) != HAL_OK) {
			Error_Handler();
		}
	}
	memcpy(&buffer_16, (uint32_t *) &data + 5, 2);
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, eeprom_base_address + 20, buffer_16);
	HAL_FLASHEx_DATAEEPROM_Lock();
}

uint16_t get_ldr_value(void) {
	uint16_t value;

	ADC_ChannelConfTypeDef sConfig;

	sConfig.Channel = ADC_CHANNEL_8;  
	  
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ADC_Start(&hadc) != HAL_OK) {
		Error_Handler();
	}

	// TODO: More accurate timeout
	if (HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK) {
		Error_Handler();
	}
	while ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) != HAL_ADC_STATE_REG_EOC) {}
	value = HAL_ADC_GetValue(&hadc);

	
	if (HAL_ADC_Stop(&hadc) != HAL_OK) {
		Error_Handler();
	}

	return value;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BNO_INT_EXTI5_Pin) {
		bno_int_triggered = true;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
