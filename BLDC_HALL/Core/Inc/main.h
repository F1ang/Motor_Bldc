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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
#define VBUS_Pin GPIO_PIN_3
#define VBUS_GPIO_Port GPIOA
#define BEMF_A_Pin GPIO_PIN_4
#define BEMF_A_GPIO_Port GPIOA
#define BEMF_B_Pin GPIO_PIN_5
#define BEMF_B_GPIO_Port GPIOA
#define BEMF_C_Pin GPIO_PIN_6
#define BEMF_C_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef int int32_T;
typedef double real64_T;

typedef enum { FALSE = 0, TRUE = !FALSE } bool;

#define U8_MAX  ((u8)255)
#define S8_MAX  ((s8)127)
#define S8_MIN  ((s8) - 128)
#define U16_MAX ((u16)65535u)
#define S16_MAX ((s16)32767)
#define S16_MIN ((s16) - 32768)
#define U32_MAX ((u32)4294967295uL)
#define S32_MAX ((s32)2147483647)
#define S32_MIN ((s32) - 2147483648)

#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define pi     3.14159265358979f

#define UNHALL_MODE
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
