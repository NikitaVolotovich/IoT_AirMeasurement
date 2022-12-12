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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  <string.h>
#include  <stdlib.h>
#include  <stdint.h>
#include  <stdio.h>

#include "bme680.h"
#include "bme680_utils.h"
#include "bme680_defs.h"
#include "registry.h"

#include "stm32_seq.h"

#include <oled.h>
#include "fonts.h"

#include "bme280.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern struct Registry_t reg;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void   MX_USART1_UART_Init(void);

/* USER CODE BEGIN EFP */
void send_float_by_uart(const char *format, float value);
void send_int_by_uart(const char *format, int value);
void send_char_by_uart(const char *format, char value);
void send_str_by_uart(const char *format);
uint8_t get_to_notify(void);
void notified(void);
uint8_t send_value_to_screen(uint8_t counter);

void WS2812b_set_LED_color (int LEDnum, int Red, int Green, int Blue);
void WS2812b_set_brightness (int brightness);
void WS2812b_send (void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
#define SWITCH_3V_Pin GPIO_PIN_2
#define SWITCH_3V_GPIO_Port GPIOC
#define SWITCH_5V_Pin GPIO_PIN_3
#define SWITCH_5V_GPIO_Port GPIOC
#define CO_Pin GPIO_PIN_7
#define CO_GPIO_Port GPIOA
#define NH3_Pin GPIO_PIN_8
#define NH3_GPIO_Port GPIOA
#define NO2_Pin GPIO_PIN_9
#define NO2_GPIO_Port GPIOA
#define BATTERY_LVL_Pin GPIO_PIN_4
#define BATTERY_LVL_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOB
#define RGB_LED_Pin GPIO_PIN_10
#define RGB_LED_GPIO_Port GPIOA
#define B2_Pin GPIO_PIN_0
#define B2_GPIO_Port GPIOD
#define B3_Pin GPIO_PIN_1
#define B3_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_5
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define ADC_CALC_VOLTAGE(msb,vref,adc)						((((float)adc*vref)/msb))


#define ADC_CALC_TEMPERATURE(__VREFANALOG_VOLTAGE__,\
                                  __TEMPSENSOR_ADC_DATA__,\
                                  __ADC_RESOLUTION__)                              \
  (float)((((( ((int32_t)((__LL_ADC_CONVERT_DATA_RESOLUTION((__TEMPSENSOR_ADC_DATA__),     \
                                                    (__ADC_RESOLUTION__),          \
                                                    LL_ADC_RESOLUTION_12B)         \
                   * (__VREFANALOG_VOLTAGE__))                                     \
                  / TEMPSENSOR_CAL_VREFANALOG)                                     \
        - (int32_t) *TEMPSENSOR_CAL1_ADDR)                                         \
     ) * (int32_t)(TEMPSENSOR_CAL2_TEMP*1000 - TEMPSENSOR_CAL1_TEMP*1000)                    \
    ) / (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) \
   ) + TEMPSENSOR_CAL1_TEMP*1000) / 1000.0f                                                        \
  )

#define MiCS_CO_PROPORTION_COEF		3.103
#define MiCS_NO2_PROPORTION_COEF	15.515
#define MiCS_NH3_PROPORTION_COEF	10.34

#define BATTERY_LOW					1.5
#define BATTERY_HIGH				2.1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
