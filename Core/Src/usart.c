/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
/*
 * 串口驱动：雷达接收（USART1）与报告发送（USART2）
 *
 * USART1（PA10，仅接收）：以 230400 8N1 接雷达 TX。
 *   中断里逐字节调用 HAL_UART_Receive_IT 收 1 字节 → 写入 uart1_rx_buf 环形缓冲；
 *   主循环的 radar_uart_poll() 负责出错后的恢复（中止接收、清标志、重启接收）。
 *   中断只做拷贝和重启接收，校验、分圈、分组全部在主循环完成（见 n10_radar.c）。
 *
 * USART2（PA2，仅发送）：把 JSON 报告经 DMA1_Channel7 发给电脑。发送完成后由
 *   USART2 中断回调通知 radar_telemetry。
 *
 * 注意：本文件由 CubeMX 生成，USER CODE 区间外的注释在重新生成代码时可能丢失。
 */
#include "common.h"
#include "n10_radar.h"
#include "radar_telemetry.h"

/* 雷达数据接收环形缓冲：中断写入，主循环读取 */
uart_ring_buf_t uart1_rx_buf;
static uint8_t radar_rx_byte;              /* 单字节接收存放处，每次中断复用 */
static volatile uint8_t rx_recovery_pending; /* 置位表示接收已停、需要在主循环恢复 */
static volatile uint32_t rx_error_count;     /* UART 错误/接收启动失败累计次数 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USART1 初始化：接雷达 TX（PA10），230400 8N1，只开接收。
 * 波特率与雷达规格一致，勿随意改动；仅 RX 可避免误驱动未接线的引脚。 */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* USART2 初始化：障碍物报告经 PA2 发往 USB-UART 适配器，230400 8N1，只开发送 */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA10     ------> USART1_RX   （接收雷达数据，配置为浮空输入）
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 中断：优先级最高(0)，保证 230400 波特率下不漏字节 */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PA2：USART2_TX，复用推挽输出 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 发送通道使用 DMA1_Channel7：内存→外设，不循环，一次一条报告 */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);
    /* TC is handled by USART2 after the DMA transfer has completed. */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

/* USER CODE BEGIN 1 */

/* 请求接收恢复：中断上下文调用，只置标志并关掉接收相关中断，
 * 真正的中止+重启放在主循环 radar_uart_poll() 里做（中断中不宜做耗时操作）。
 * 关中断后不会再进接收中断，从而保证不会再有字节写进环形缓冲。 */
static void request_rx_recovery(void)
{
    rx_recovery_pending = 1U;
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);
}

/* 启动雷达接收：清空环形缓冲与错误计数，清除溢出标志后开启单字节中断接收。
 * 全程关中断，避免初始化过程中被接收中断打断导致状态不一致 */
void radar_uart_start(void)
{
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    ring_buf_init(&uart1_rx_buf);
    rx_error_count = 0U;
    rx_recovery_pending = 0U;
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    if (HAL_UART_Receive_IT(&huart1, &radar_rx_byte, 1U) != HAL_OK) {
        rx_error_count++;
        request_rx_recovery();
    }
    __set_PRIMASK(interrupt_state);
}

/* 主循环任务：处理 USART1 接收错误后的恢复。
 * 流程：中止当前接收 → 清溢出标志 → 丢弃缓冲残留字节 → 重启中断接收；
 * 任一步失败就重新置位恢复请求，下轮再试。
 * 注意只中止"接收"，不影响 gState 和 USART2 正在进行的发送 DMA */
void radar_uart_poll(void)
{
    bool reset_scan = false;
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (rx_recovery_pending != 0U) {
        reset_scan = true;
        rx_recovery_pending = 0U;
        /* 只中止接收；gState 与正在进行的发送 DMA 都不受影响 */
        if (HAL_UART_AbortReceive(&huart1) == HAL_OK) {
            __HAL_UART_CLEAR_OREFLAG(&huart1);
            uart1_rx_buf.tail = uart1_rx_buf.head;  /* 丢弃已收到但不可信的字节 */
            if (HAL_UART_Receive_IT(&huart1, &radar_rx_byte, 1U) != HAL_OK) {
                rx_error_count++;
                request_rx_recovery();
            }
        } else {
            rx_error_count++;
            request_rx_recovery();
        }
    }
    __set_PRIMASK(interrupt_state);
    /* 在中断外通知解析模块：数据流已断，扫描需要重新同步 */
    if (reset_scan) {
        n10p_radar_notify_rx_error();
    }
}

/* 累计 UART 错误/接收启动失败次数，随 JSON 报告上报，便于电脑端诊断线路 */
uint32_t radar_uart_errors(void)
{
    return rx_error_count;
}

/* USART1 每收到 1 字节进入一次：把字节存入环形缓冲并立刻重新开启接收。
 * 校验、组帧等全部留给主循环，中断内只做最少的拷贝和重启 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }
    /* 重新开启接收会清 ErrorCode，因此先让 HAL 处理完错误字节，
     * 本字节丢弃，等 ErrorCallback 触发恢复流程 */
    if ((huart->ErrorCode & (HAL_UART_ERROR_PE | HAL_UART_ERROR_NE |
                              HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE)) != 0U) {
        return;
    }
    ring_buf_put(&uart1_rx_buf, radar_rx_byte);
    if (HAL_UART_Receive_IT(huart, &radar_rx_byte, 1U) != HAL_OK) {
        rx_error_count++;
        request_rx_recovery();
    }
}

/* UART 错误回调：
 * USART1 出现校验/噪声/帧/溢出错误 → 请求恢复（数据流断点，扫描需重新同步）；
 * USART2 出现 DMA 错误 → 通知上报模块收尾并释放缓冲 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1 &&
        (huart->ErrorCode & (HAL_UART_ERROR_PE | HAL_UART_ERROR_NE |
                             HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE)) != 0U) {
        rx_error_count++;
        request_rx_recovery();
    }
    if (huart->Instance == USART2 && (huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U) {
        radar_telemetry_tx_error();
    }
}

/* USART2 发送完成回调：DMA 数据已发完（TC），通知上报模块可以释放缓冲。
 * 上报模块只在这里置标志，实际状态切换在主循环完成 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        radar_telemetry_tx_complete();
    }
}

/* USER CODE END 1 */
