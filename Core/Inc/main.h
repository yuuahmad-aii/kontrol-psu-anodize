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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */

  // Pindahkan definisi struktur ke sini agar bisa diakses oleh flash_storage.h
  typedef struct
  {
    uint32_t remaining_seconds; // Waktu tersisa dalam detik
    uint16_t current_mA;        // Arus dalam miliampere
    bool is_running;            // Status: true jika berjalan
    bool is_finished;           // Status: true jika timer selesai
  } TimerInfo_t;

  /* USER CODE END ET */

  /* Exported constants --------------------------------------------------------*/
  /* USER CODE BEGIN EC */
#define NUM_TIMERS 8 // Jumlah total timer
  /* USER CODE END EC */

  /* Exported macro ------------------------------------------------------------*/
  /* USER CODE BEGIN EM */

  /* USER CODE END EM */

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BTN_ENC_Pin GPIO_PIN_4
#define BTN_ENC_GPIO_Port GPIOA
#define MAX7219_CS_Pin GPIO_PIN_10
#define MAX7219_CS_GPIO_Port GPIOB
#define MODBUS_SEL_Pin GPIO_PIN_7
#define MODBUS_SEL_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MAX7219_CLK_Pin GPIO_PIN_3
#define MAX7219_CLK_GPIO_Port GPIOB
#define MAX7219_MISO_Pin GPIO_PIN_4
#define MAX7219_MISO_GPIO_Port GPIOB
#define MAX7219_MOSI_Pin GPIO_PIN_5
#define MAX7219_MOSI_GPIO_Port GPIOB
#define HC595_LATCH_Pin GPIO_PIN_6
#define HC595_LATCH_GPIO_Port GPIOB

  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
