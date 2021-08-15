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
#include "user.h"
#include "ADC.h"  
#include "stdio.h" 
#include "string.h"
#include "RC_pwm.h"  
  
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
;
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
#define CAN_SHDN_Pin GPIO_PIN_14
#define CAN_SHDN_GPIO_Port GPIOC
#define MOTOR_POWER_ON_Pin GPIO_PIN_0
#define MOTOR_POWER_ON_GPIO_Port GPIOC
#define EN_5V_OUT_Pin GPIO_PIN_1
#define EN_5V_OUT_GPIO_Port GPIOC
#define LED_Red_Pin GPIO_PIN_2
#define LED_Red_GPIO_Port GPIOC
#define POWER_ON_Pin GPIO_PIN_3
#define POWER_ON_GPIO_Port GPIOC
#define keyled_ON_Pin GPIO_PIN_4
#define keyled_ON_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_0
#define PWM3_GPIO_Port GPIOB
#define DLSW_4_Pin GPIO_PIN_12
#define DLSW_4_GPIO_Port GPIOB
#define DLSW_3_Pin GPIO_PIN_13
#define DLSW_3_GPIO_Port GPIOB
#define DLSW_2_Pin GPIO_PIN_14
#define DLSW_2_GPIO_Port GPIOB
#define DLSW_1_Pin GPIO_PIN_15
#define DLSW_1_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOC
#define RS485_TX_EN_Pin GPIO_PIN_8
#define RS485_TX_EN_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

#define LED_Red_ON() HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET)
#define LED_Red_OFF() HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET)
#define LED_Red_Toggle() HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin)

#define RS_T_EN() HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_SET)
#define RS_R_EN() HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_RESET)

#define DLSW_1_Pin_read() HAL_GPIO_ReadPin(DLSW_1_GPIO_Port, DLSW_1_Pin)
#define DLSW_2_Pin_read() HAL_GPIO_ReadPin(DLSW_2_GPIO_Port, DLSW_2_Pin)
#define DLSW_3_Pin_read() HAL_GPIO_ReadPin(DLSW_3_GPIO_Port, DLSW_3_Pin)
#define DLSW_4_Pin_read() HAL_GPIO_ReadPin(DLSW_4_GPIO_Port, DLSW_4_Pin)

#define RS485_Mode 0x00U
#define CAN_Mode 0x01U

#define Speed_Control_Mode 0x00U
#define Angle_Control_Mode 0x01U


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
