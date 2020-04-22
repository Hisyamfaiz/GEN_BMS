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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define DIP1 HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin)
#define DIP2 HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin)

#define BUZZ_Toggle HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin)
#define ON 1
#define OFF 0

#define STATE_STANDBY 0
#define STATE_CHARGE 1
#define STATE_DISCHARGE 2
#define STATE_FULL_CHARGE_DISCHARGE 3

void BMS_ON_InitBeep(void);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t 		flag_trip_overtemperature,
					flag_trip_undertemperature,
					flag_trip_SOCOverDischarge,
					flag_trip_SOCOverCharge,			//di tiada kan..!
					flag_trip_undervoltage,
					flag_trip_overvoltage,
					flag_trip_overcurrentdischarge,
					flag_trip_overcurrentcharge,
					flag_trip_shortcircuit,
					flag_trip_systemfailure,
					flag_trip_unbalance;
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
#define DIP1_Pin GPIO_PIN_0
#define DIP1_GPIO_Port GPIOC
#define DIP2_Pin GPIO_PIN_1
#define DIP2_GPIO_Port GPIOC
#define BUZZ_Pin GPIO_PIN_2
#define BUZZ_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOC
#define CS_SPI1_Pin GPIO_PIN_0
#define CS_SPI1_GPIO_Port GPIOB
#define MEM_WP_Pin GPIO_PIN_12
#define MEM_WP_GPIO_Port GPIOB
#define BAT_CUT_N_Pin GPIO_PIN_9
#define BAT_CUT_N_GPIO_Port GPIOC
#define BAT_CUT_P_Pin GPIO_PIN_8
#define BAT_CUT_P_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
