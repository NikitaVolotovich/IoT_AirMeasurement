/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
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
#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* STATUS_SVC */
  CUSTOM_STM_MODE,
  CUSTOM_STM_BATTERY_V,
  CUSTOM_STM_MCU_TEMP,
  /* BME_SVC */
  CUSTOM_STM_BME_IAQ,
  CUSTOM_STM_BME_TEMP,
  CUSTOM_STM_BME_HUM,
  CUSTOM_STM_BME_PRESS,
  /* MiCS_SVC */
  CUSTOM_STM_CO,
  CUSTOM_STM_NH3,
  CUSTOM_STM_NO2,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* MODE */
  CUSTOM_STM_MODE_WRITE_EVT,
  CUSTOM_STM_MODE_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_MODE_NOTIFY_DISABLED_EVT,
  /* BATTERY_V */
  CUSTOM_STM_BATTERY_V_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_BATTERY_V_NOTIFY_DISABLED_EVT,
  /* MCU_TEMP */
  CUSTOM_STM_MCU_TEMP_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_MCU_TEMP_NOTIFY_DISABLED_EVT,
  /* BME_IAQ */
  CUSTOM_STM_BME_IAQ_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_BME_IAQ_NOTIFY_DISABLED_EVT,
  /* BME_TEMP */
  CUSTOM_STM_BME_TEMP_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_BME_TEMP_NOTIFY_DISABLED_EVT,
  /* BME_HUM */
  CUSTOM_STM_BME_HUM_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_BME_HUM_NOTIFY_DISABLED_EVT,
  /* BME_PRESS */
  CUSTOM_STM_BME_PRESS_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_BME_PRESS_NOTIFY_DISABLED_EVT,
  /* CO */
  CUSTOM_STM_CO_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_CO_NOTIFY_DISABLED_EVT,
  /* NH3 */
  CUSTOM_STM_NH3_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_NH3_NOTIFY_DISABLED_EVT,
  /* NO2 */
  CUSTOM_STM_NO2_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_NO2_NOTIFY_DISABLED_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t   Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint8_t SizeMode;
extern uint8_t SizeBattery_V;
extern uint8_t SizeMcu_Temp;
extern uint8_t SizeBme_Iaq;
extern uint8_t SizeBme_Temp;
extern uint8_t SizeBme_Hum;
extern uint8_t SizeBme_Press;
extern uint8_t SizeCo;
extern uint8_t SizeNh3;
extern uint8_t SizeNo2;

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc(void);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
