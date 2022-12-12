/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* STATUS_SVC */
  uint8_t               Mode_Notification_Status;
  uint8_t               Battery_v_Notification_Status;
  uint8_t               Mcu_temp_Notification_Status;
  /* BME_SVC */
  uint8_t               Bme_iaq_Notification_Status;
  uint8_t               Bme_temp_Notification_Status;
  uint8_t               Bme_hum_Notification_Status;
  uint8_t               Bme_press_Notification_Status;
  /* MiCS_SVC */
  uint8_t               Co_Notification_Status;
  uint8_t               Nh3_Notification_Status;
  uint8_t               No2_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

//static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */
uint8_t notify_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* STATUS_SVC */
static void Custom_Mode_Update_Char(void);
//static void Custom_Mode_Send_Notification(void);
static void Custom_Battery_v_Update_Char(void);
//static void Custom_Battery_v_Send_Notification(void);
static void Custom_Mcu_temp_Update_Char(void);
//static void Custom_Mcu_temp_Send_Notification(void);
/* BME_SVC */
static void Custom_Bme_iaq_Update_Char(void);
//static void Custom_Bme_iaq_Send_Notification(void);
static void Custom_Bme_temp_Update_Char(void);
//static void Custom_Bme_temp_Send_Notification(void);
static void Custom_Bme_hum_Update_Char(void);
//static void Custom_Bme_hum_Send_Notification(void);
static void Custom_Bme_press_Update_Char(void);
//static void Custom_Bme_press_Send_Notification(void);
/* MiCS_SVC */
static void Custom_Co_Update_Char(void);
//static void Custom_Co_Send_Notification(void);
static void Custom_Nh3_Update_Char(void);
//static void Custom_Nh3_Send_Notification(void);
static void Custom_No2_Update_Char(void);
//static void Custom_No2_Send_Notification(void);

/* USER CODE BEGIN PFP */
void myTask(void){
	if(get_to_notify()) {
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		switch(notify_counter){
			case 0: Custom_Mode_Update_Char(); break;
			case 1: Custom_Battery_v_Update_Char(); break;
			case 2: Custom_Mcu_temp_Update_Char(); break;

			case 4: Custom_Bme_iaq_Update_Char(); break;
			case 5: Custom_Bme_temp_Update_Char(); break;
			case 6: Custom_Bme_hum_Update_Char(); break;
			case 7: Custom_Bme_press_Update_Char(); break;

			case 8: Custom_Co_Update_Char(); break;
			case 9: Custom_Nh3_Update_Char(); break;
			case 10: Custom_No2_Update_Char(); break;
		}
		notified();
		notify_counter++;
	}
	if(notify_counter == 10)
		notify_counter = 0;

	UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK, CFG_SCH_PRIO_0);
}

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* STATUS_SVC */
    case CUSTOM_STM_MODE_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MODE_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_MODE_WRITE_EVT */
      break;

    case CUSTOM_STM_MODE_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MODE_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_MODE_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MODE_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MODE_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_MODE_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_BATTERY_V_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATTERY_V_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_BATTERY_V_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_BATTERY_V_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATTERY_V_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_BATTERY_V_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_MCU_TEMP_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MCU_TEMP_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_MCU_TEMP_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MCU_TEMP_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MCU_TEMP_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_MCU_TEMP_NOTIFY_DISABLED_EVT */
      break;

    /* BME_SVC */
    case CUSTOM_STM_BME_IAQ_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BME_IAQ_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_BME_IAQ_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_BME_IAQ_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BME_IAQ_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_BME_IAQ_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_BME_TEMP_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BME_TEMP_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_BME_TEMP_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_BME_TEMP_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BME_TEMP_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_BME_TEMP_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_BME_HUM_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BME_HUM_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_BME_HUM_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_BME_HUM_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BME_HUM_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_BME_HUM_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_BME_PRESS_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BME_PRESS_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_BME_PRESS_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_BME_PRESS_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BME_PRESS_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_BME_PRESS_NOTIFY_DISABLED_EVT */
      break;

    /* MiCS_SVC */
    case CUSTOM_STM_CO_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CO_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_CO_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_CO_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CO_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_CO_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NH3_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NH3_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_NH3_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_NH3_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NH3_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_NH3_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NO2_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NO2_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_NO2_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_NO2_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NO2_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_NO2_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* STATUS_SVC */
void Custom_Mode_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Mode_UC_1*/
  updateflag = 1;
  UpdateCharData[0] = reg.mode;
  /* USER CODE END Mode_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MODE, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Mode_UC_Last*/
  updateflag = 0;
  /* USER CODE END Mode_UC_Last*/
  return;
}

//void Custom_Mode_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN Mode_NS_1*/
//
//  /* USER CODE END Mode_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_MODE, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN Mode_NS_Last*/
//
//  /* USER CODE END Mode_NS_Last*/
//
//  return;
//}

void Custom_Battery_v_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Battery_v_UC_1*/
  updateflag = 1;
  UpdateCharData[0] = reg.battery_lvl;
  /* USER CODE END Battery_v_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_V, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Battery_v_UC_Last*/
  updateflag = 0;
  /* USER CODE END Battery_v_UC_Last*/
  return;
}

//void Custom_Battery_v_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN Battery_v_NS_1*/
//
//  /* USER CODE END Battery_v_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_V, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN Battery_v_NS_Last*/
//
//  /* USER CODE END Battery_v_NS_Last*/
//
//  return;
//}

void Custom_Mcu_temp_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Mcu_temp_UC_1*/
  updateflag = 1;
  UpdateCharData[0] = reg.mcu_temp;
  /* USER CODE END Mcu_temp_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MCU_TEMP, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Mcu_temp_UC_Last*/
  updateflag = 0;
  /* USER CODE END Mcu_temp_UC_Last*/
  return;
}

//void Custom_Mcu_temp_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN Mcu_temp_NS_1*/
//
//  /* USER CODE END Mcu_temp_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_MCU_TEMP, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN Mcu_temp_NS_Last*/
//
//  /* USER CODE END Mcu_temp_NS_Last*/
//
//  return;
//}

/* BME_SVC */
void Custom_Bme_iaq_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Bme_iaq_UC_1*/
  UpdateCharData[0] = reg.bme680_iaq;
  updateflag = 1;
  /* USER CODE END Bme_iaq_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BME_IAQ, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Bme_iaq_UC_Last*/
  updateflag = 0;
  /* USER CODE END Bme_iaq_UC_Last*/
  return;
}

//void Custom_Bme_iaq_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN Bme_iaq_NS_1*/
//
//  /* USER CODE END Bme_iaq_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_BME_IAQ, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN Bme_iaq_NS_Last*/
//
//  /* USER CODE END Bme_iaq_NS_Last*/
//
//  return;
//}

void Custom_Bme_temp_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Bme_temp_UC_1*/
  updateflag = 1;
  UpdateCharData[0] = reg.bme680_temperature;
  /* USER CODE END Bme_temp_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BME_TEMP, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Bme_temp_UC_Last*/

  /* USER CODE END Bme_temp_UC_Last*/
  return;
}

//void Custom_Bme_temp_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN Bme_temp_NS_1*/
//
//  /* USER CODE END Bme_temp_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_BME_TEMP, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN Bme_temp_NS_Last*/
//
//  /* USER CODE END Bme_temp_NS_Last*/
//
//  return;
//}

void Custom_Bme_hum_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Bme_hum_UC_1*/
  updateflag = 1;
  UpdateCharData[0] = reg.bme680_humidity;
  /* USER CODE END Bme_hum_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BME_HUM, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Bme_hum_UC_Last*/

  /* USER CODE END Bme_hum_UC_Last*/
  return;
}

//void Custom_Bme_hum_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN Bme_hum_NS_1*/
//
//  /* USER CODE END Bme_hum_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_BME_HUM, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN Bme_hum_NS_Last*/
//
//  /* USER CODE END Bme_hum_NS_Last*/
//
//  return;
//}

void Custom_Bme_press_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Bme_press_UC_1*/
  updateflag = 1;
  UpdateCharData[0] = reg.bme680_pressure;
  /* USER CODE END Bme_press_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BME_PRESS, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Bme_press_UC_Last*/

  /* USER CODE END Bme_press_UC_Last*/
  return;
}

//void Custom_Bme_press_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN Bme_press_NS_1*/
//
//  /* USER CODE END Bme_press_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_BME_PRESS, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN Bme_press_NS_Last*/
//
//  /* USER CODE END Bme_press_NS_Last*/
//
//  return;
//}

/* MiCS_SVC */
void Custom_Co_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Co_UC_1*/
  updateflag = 1;
  UpdateCharData[0] = reg.MiCS_CO;
  /* USER CODE END Co_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CO, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Co_UC_Last*/

  /* USER CODE END Co_UC_Last*/
  return;
}

//void Custom_Co_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN Co_NS_1*/
//
//  /* USER CODE END Co_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_CO, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN Co_NS_Last*/
//
//  /* USER CODE END Co_NS_Last*/
//
//  return;
//}

void Custom_Nh3_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Nh3_UC_1*/
  updateflag = 1;
  UpdateCharData[0] = reg.MiCS_NH3;
  /* USER CODE END Nh3_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_NH3, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Nh3_UC_Last*/

  /* USER CODE END Nh3_UC_Last*/
  return;
}

//void Custom_Nh3_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN Nh3_NS_1*/
//
//  /* USER CODE END Nh3_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_NH3, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN Nh3_NS_Last*/
//
//  /* USER CODE END Nh3_NS_Last*/
//
//  return;
//}

void Custom_No2_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN No2_UC_1*/
  updateflag = 1;
  UpdateCharData[0] = reg.MiCS_NO2;
  /* USER CODE END No2_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_NO2, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN No2_UC_Last*/

  /* USER CODE END No2_UC_Last*/
  return;
}

//void Custom_No2_Send_Notification(void) /* Property Notification */
//{
//  uint8_t updateflag = 0;
//
//  /* USER CODE BEGIN No2_NS_1*/
//
//  /* USER CODE END No2_NS_1*/
//
//  if (updateflag != 0)
//  {
//    Custom_STM_App_Update_Char(CUSTOM_STM_NO2, (uint8_t *)NotifyCharData);
//  }
//
//  /* USER CODE BEGIN No2_NS_Last*/
//
//  /* USER CODE END No2_NS_Last*/
//
//  return;
//}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
