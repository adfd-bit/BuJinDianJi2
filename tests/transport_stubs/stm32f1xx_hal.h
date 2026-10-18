#ifndef TEST_TRANSPORT_HAL_H
#define TEST_TRANSPORT_HAL_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t SR, DR, CR1, CR3; } USART_TypeDef;
typedef struct DMA_HandleTypeDef {
    void *Instance;
    struct {
        uint32_t Direction, PeriphInc, MemInc, PeriphDataAlignment;
        uint32_t MemDataAlignment, Mode, Priority;
    } Init;
} DMA_HandleTypeDef;
typedef struct {
    USART_TypeDef *Instance;
    struct {
        uint32_t BaudRate, WordLength, StopBits, Parity, Mode;
        uint32_t HwFlowCtl, OverSampling;
    } Init;
    uint32_t ErrorCode;
    DMA_HandleTypeDef *hdmatx, *hdmarx;
} UART_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Speed, Pull; } GPIO_InitTypeDef;

extern USART_TypeDef test_uart_instance;
extern USART_TypeDef test_uart2_instance;
extern uint32_t test_primask;
#define USART1 (&test_uart_instance)
#define USART2 (&test_uart2_instance)
#define DMA1_Channel7 ((void *)7)
#define GPIOA ((void *)0)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_MODE_AF_PP 0U
#define GPIO_SPEED_FREQ_HIGH 0U
#define GPIO_MODE_INPUT 0U
#define GPIO_NOPULL 0U
#define UART_WORDLENGTH_8B 0U
#define UART_STOPBITS_1 0U
#define UART_PARITY_NONE 0U
#define UART_MODE_RX 1U
#define UART_MODE_TX 2U
#define UART_HWCONTROL_NONE 0U
#define UART_OVERSAMPLING_16 0U
#define DMA_MEMORY_TO_PERIPH 0U
#define DMA_PINC_DISABLE 0U
#define DMA_MINC_ENABLE 0U
#define DMA_PDATAALIGN_BYTE 0U
#define DMA_MDATAALIGN_BYTE 0U
#define DMA_NORMAL 0U
#define DMA_PRIORITY_LOW 0U
#define USART1_IRQn 0U
#define USART2_IRQn 1U
#define HAL_UART_ERROR_NONE 0U
#define HAL_UART_ERROR_PE 1U
#define HAL_UART_ERROR_NE 2U
#define HAL_UART_ERROR_FE 4U
#define HAL_UART_ERROR_ORE 8U
#define HAL_UART_ERROR_DMA 16U
#define UART_IT_RXNE 1U
#define UART_IT_ERR 2U
#define UART_IT_PE 4U
#define __HAL_UART_DISABLE_IT(handle, interrupt) ((void)(handle), (void)(interrupt))
#define __HAL_UART_CLEAR_OREFLAG(handle) ((handle)->Instance->SR = 0U)
#define __HAL_RCC_USART1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART1_CLK_DISABLE() ((void)0)
#define __HAL_RCC_USART2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART2_CLK_DISABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_LINKDMA(handle, member, dma) ((handle)->member = &(dma))
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1U; }
static inline void __set_PRIMASK(uint32_t state) { test_primask = state; }

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *handle);
void HAL_DMA_DeInit(DMA_HandleTypeDef *handle);
void HAL_GPIO_Init(void *gpio, GPIO_InitTypeDef *init);
void HAL_GPIO_DeInit(void *gpio, uint32_t pins);
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(uint32_t irq);
void HAL_NVIC_DisableIRQ(uint32_t irq);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *handle, const uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *handle);

#endif
