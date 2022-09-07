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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PI_RxD_Pin GPIO_PIN_6
#define PI_RxD_GPIO_Port GPIOB
#define PI_TxD_Pin GPIO_PIN_7
#define PI_TxD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define RING_BUF_SIZE 128

typedef struct {
  uint16_t w_pos;
  uint16_t r_pos;
  uint8_t data[RING_BUF_SIZE];
} t_ring;

void ring_put_byte(t_ring *ring, uint8_t data);
uint8_t* ring_get_byte(t_ring *ring);
void ring_put_data(t_ring *ring, uint8_t *data, uint16_t size);
uint16_t ring_get_data(t_ring *ring, uint8_t *data, uint16_t size);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
