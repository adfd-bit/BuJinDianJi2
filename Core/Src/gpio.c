/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
/**
  * @brief 使能所用 GPIO 端口时钟。
  *        工程未使用普通 GPIO 功能，串口引脚 PA10(雷达 RX)、PA2(上报 TX)
  *        分别在 usart.c 的 HAL_UART_MspInit 中按复用/输入模式配置。
  */
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();  /* 外部晶振 HSE 用的 PD0/PD1 端口时钟 */
  __HAL_RCC_GPIOA_CLK_ENABLE();  /* 两个串口引脚所在端口 */

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
