/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Radar obstacle monitor
 ******************************************************************************
 * 功能：雷达障碍物监视程序（STM32F103C8T6）。
 *   USART1 接收雷达串口数据 → 主循环解析出障碍物坐标 →
 *   USART2 每 100 ms 输出一行 JSON 报告给电脑端可视化程序。
 *
 * 整体结构：
 *   1. 外设初始化：SystemClock_Config / MX_GPIO_Init / MX_DMA_Init /
 *      MX_USART1_UART_Init（雷达接收，仅 RX）/ MX_USART2_UART_Init（上报，仅 TX）
 *   2. 应用初始化：环形缓冲、雷达解析模块、上报模块，最后启动 USART1 中断接收
 *   3. 主循环：三个任务轮询执行，均不阻塞，中断只负责收字节和记录发送完成
 *
 * 注意：本文件大部分代码由 CubeMX 生成，只有 USER CODE 区间内的内容在重新
 * 生成时会被保留；在 CubeMX 中重新生成代码前请留意这一点。
 ******************************************************************************
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 * This software is licensed under terms in the LICENSE file in the root
 * directory of this software component. If no LICENSE file comes with this
 * software, it is provided AS-IS.
 ******************************************************************************
 */
/* USER CODE END Header */
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "common.h"
#include "n10_radar.h"
#include "radar_telemetry.h"
/* USER CODE END Includes */

void SystemClock_Config(void);

/**
 * @brief  程序入口：初始化时钟与外设，然后进入雷达数据处理主循环。
 */
int main(void)
{
  HAL_Init();            /* HAL 库初始化（含 SysTick 1 ms 时基、中断优先级分组） */
  SystemClock_Config();  /* 配置系统时钟为 72 MHz */
  MX_GPIO_Init();        /* 使能 GPIO 时钟 */
  MX_DMA_Init();         /* 使能 DMA1 时钟并开 DMA1_Channel7 中断（USART2 发送用） */
  MX_USART1_UART_Init(); /* USART1：接雷达 TX，230400 8N1，仅接收 */
  MX_USART2_UART_Init(); /* USART2：接 USB-UART，230400 8N1，仅发送报告 */

  /* USER CODE BEGIN 2 */
  /* 初始化顺序有依赖：先备好接收缓冲，再复位解析状态和上报状态，最后才开中断收数据 */
  ring_buf_init(&uart1_rx_buf); /* 清空 USART1 接收环形缓冲（中断写入、主循环读取） */
  n10p_radar_init();            /* 复位雷达帧解析、分圈与障碍物统计状态 */
  radar_telemetry_init();       /* 复位 JSON 上报缓冲池与计数 */
  radar_uart_start();           /* 启动 USART1 单字节中断接收，开始接收雷达数据 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  /* 主循环：三个函数各做一件事，都不等待外设，因此不会互相阻塞 */
  while (1)
  {
    radar_uart_poll();         /* 处理 USART1 接收错误：中止并重新启动接收（见 usart.c） */
    n10p_radar_process();      /* 解析环形缓冲中的字节：组帧 → 校验 → 分圈 → 分组 → 生成快照 */
    radar_telemetry_process(); /* 每 100 ms 把最新快照打包成 JSON，经 USART2 DMA 发出 */
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief  配置系统时钟：外部 8 MHz 晶振（HSE）经 PLL 9 倍频得到 72 MHz 系统时钟。
 *         AHB=72 MHz，APB2=72 MHz（USART1 挂 APB2），APB1=36 MHz（USART2 挂 APB1）。
 *         两个串口的 230400 波特率都依赖该时钟配置。
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 打开 HSE 并以其作为 PLL 输入，PLL ×9 → 72 MHz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 切换时钟源为 PLL，并设置各总线分频 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  致命错误处理：初始化失败（通常是时钟或外设配置错误）时调用。
 *         关中断后死循环，便于调试器停在此处定位问题。
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  HAL 断言失败回调（仅当定义了 USE_FULL_ASSERT 时编译）；
 *         参数为出错的文件名和行号，可在调试时打断点查看。
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
