#include <assert.h>
#include <stdio.h>

#include "../Core/Src/usart.c"

USART_TypeDef test_uart_instance;
USART_TypeDef test_uart2_instance;
uint32_t test_primask;
static uint8_t *rx_destination;
static HAL_StatusTypeDef receive_result = HAL_OK;
static HAL_StatusTypeDef receive_abort_result = HAL_OK;
static unsigned int receives;
static unsigned int receive_aborts;
static unsigned int scan_resets;
static unsigned int tx_completions;
static unsigned int tx_errors;

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
void HAL_DMA_DeInit(DMA_HandleTypeDef *handle) { (void)handle; }
void HAL_GPIO_Init(void *gpio, GPIO_InitTypeDef *init) { (void)gpio; (void)init; }
void HAL_GPIO_DeInit(void *gpio, uint32_t pins) { (void)gpio; (void)pins; }
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority)
{ (void)irq; (void)priority; (void)subpriority; }
void HAL_NVIC_EnableIRQ(uint32_t irq) { (void)irq; }
void HAL_NVIC_DisableIRQ(uint32_t irq) { (void)irq; }
void Error_Handler(void) { assert(0); }
void n10p_radar_notify_rx_error(void) { scan_resets++; }
void radar_telemetry_tx_error(void) { tx_errors++; }
void radar_telemetry_tx_complete(void) { tx_completions++; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length)
{
    assert(handle == &huart1 && length == 1U);
    receives++;
    if (receive_result == HAL_OK) {
        rx_destination = data;
        handle->ErrorCode = HAL_UART_ERROR_NONE;
    }
    return receive_result;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{
    assert(handle == &huart1 && test_primask == 1U);
    receive_aborts++;
    return receive_abort_result;
}

int main(void)
{
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    HAL_UART_MspInit(&huart1);
    HAL_UART_MspInit(&huart2);
    assert(huart1.Init.BaudRate == 230400U);
    assert(huart1.Init.Mode == UART_MODE_RX);
    assert(huart1.hdmatx == NULL && huart1.hdmarx == NULL);
    assert(huart2.Init.Mode == UART_MODE_TX && huart2.Init.BaudRate == 230400U);
    assert(huart2.hdmatx == &hdma_usart2_tx && huart2.hdmarx == NULL);
    radar_uart_start();
    assert(receives == 1U && radar_uart_errors() == 0U && test_primask == 0U);
    *rx_destination = 0xA5;
    HAL_UART_RxCpltCallback(&huart1);
    assert(receives == 2U && ring_buf_get(&uart1_rx_buf) == 0xA5);

    /* ORE/FE/NE/PE must remain visible to HAL until ErrorCallback. */
    const uint32_t errors[] = {HAL_UART_ERROR_ORE, HAL_UART_ERROR_FE,
                               HAL_UART_ERROR_NE, HAL_UART_ERROR_PE};
    for (unsigned int index = 0U; index < sizeof(errors) / sizeof(errors[0]); index++) {
        huart1.ErrorCode = errors[index];
        unsigned int old_receives = receives;
        HAL_UART_RxCpltCallback(&huart1);
        assert(receives == old_receives && huart1.ErrorCode == errors[index]);
        assert(ring_buf_empty(&uart1_rx_buf));
        ring_buf_put(&uart1_rx_buf, 0x99);
        HAL_UART_ErrorCallback(&huart1);
        radar_uart_poll();
        assert(receives == old_receives + 1U && ring_buf_empty(&uart1_rx_buf));
        assert(huart2.hdmatx == &hdma_usart2_tx);
    }
    assert(radar_uart_errors() == 4U && receive_aborts == 4U && scan_resets == 4U);
    assert(tx_completions == 0U && tx_errors == 0U);

    /* Re-arm failure retries in the main loop and resets the scan. */
    receive_result = HAL_BUSY;
    *rx_destination = 0x5A;
    HAL_UART_RxCpltCallback(&huart1);
    assert(radar_uart_errors() == 5U);
    receive_result = HAL_OK;
    radar_uart_poll();
    assert(scan_resets == 5U && ring_buf_empty(&uart1_rx_buf));

    receive_result = HAL_ERROR;
    radar_uart_start();
    assert(radar_uart_errors() == 1U);
    receive_result = HAL_OK;
    receive_abort_result = HAL_ERROR;
    radar_uart_poll();
    assert(rx_recovery_pending != 0U && radar_uart_errors() == 2U);
    receive_abort_result = HAL_OK;
    radar_uart_poll();
    assert(rx_recovery_pending == 0U && test_primask == 0U);

    /* TX callbacks only notify telemetry and must not abort the receiver. */
    unsigned int old_aborts = receive_aborts;
    huart2.ErrorCode = HAL_UART_ERROR_DMA;
    HAL_UART_ErrorCallback(&huart2);
    HAL_UART_TxCpltCallback(&huart2);
    huart1.ErrorCode = HAL_UART_ERROR_DMA;
    HAL_UART_ErrorCallback(&huart1);
    HAL_UART_TxCpltCallback(&huart1);
    radar_uart_poll();
    assert(tx_errors == 1U && tx_completions == 1U && old_aborts == receive_aborts);
    puts("UART transport tests passed");
    return 0;
}
