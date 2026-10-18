#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "n10_radar.h"
#include "usart.h"
#include "../Core/Src/radar_telemetry.c"

USART_TypeDef test_uart_instance;
USART_TypeDef test_uart2_instance;
uint32_t test_primask;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
uart_ring_buf_t uart1_rx_buf;
static n10p_obstacle_snapshot_t test_snapshot;
static uint32_t test_tick;
static HAL_StatusTypeDef transmit_result = HAL_OK;
static HAL_StatusTypeDef abort_result = HAL_OK;
static const uint8_t *dma_data;
static uint16_t dma_length;
static unsigned int transmit_calls;
static unsigned int abort_calls;

uint32_t HAL_GetTick(void) { return test_tick; }
uint32_t radar_uart_errors(void) { return UINT32_MAX; }
uint32_t n10p_radar_valid_frames(void) { return UINT32_MAX; }
uint32_t n10p_radar_bad_frames(void) { return UINT32_MAX; }
void n10p_get_obstacle_snapshot(n10p_obstacle_snapshot_t *out) { *out = test_snapshot; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *handle, const uint8_t *data, uint16_t length)
{
    assert(handle == &huart2);
    transmit_calls++;
    if (transmit_result == HAL_OK) {
        dma_data = data;
        dma_length = length;
        assert(length > 0U && length < TX_BUFFER_SIZE);
        assert(data[length - 1U] == '\n' && data[length] == '\0');
    }
    return transmit_result;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *handle)
{
    assert(handle == &huart2);
    abort_calls++;
    return abort_result;
}

int main(void)
{
    test_snapshot.age_ms = UINT32_MAX;
    uart1_rx_buf.dropped = UINT32_MAX;
    radar_telemetry_init();
    radar_telemetry_process();
    assert(transmit_calls == 1U);
    assert(strstr((const char *)dma_data, "\"valid\":false") != NULL);
    assert(strstr((const char *)dma_data, "\"age_ms\":null") != NULL);
    assert(strstr((const char *)dma_data, "\"front_m\":null,\"blocked\":null,\"obstacles\":[]") != NULL);
    puts((const char *)dma_data);

    char immutable[TX_BUFFER_SIZE];
    memcpy(immutable, dma_data, dma_length + 1U);
    const uint8_t *first_dma = dma_data;
    test_snapshot.valid = true;
    test_snapshot.front_known = true;
    test_snapshot.front_mm = 499U;
    test_snapshot.blocked = true;
    test_snapshot.count = N10P_MAX_OBSTACLES;
    test_snapshot.age_ms = 1U;
    for (uint8_t index = 0U; index < test_snapshot.count; index++) {
        test_snapshot.obstacles[index].x_mm = INT32_MIN;
        test_snapshot.obstacles[index].y_mm = INT32_MAX;
    }
    for (uint32_t index = 1U; index <= 20U; index++) {
        test_tick = index * REPORT_INTERVAL_MS;
        test_snapshot.scan_seq = index;
        radar_telemetry_process();
        assert(transmit_calls == 1U);
        assert(memcmp(immutable, first_dma, strlen(immutable) + 1U) == 0);
    }
    radar_telemetry_tx_complete();
    assert(tx_buffers[(uint8_t)in_flight_index].state == TX_IN_FLIGHT);
    radar_telemetry_process();
    assert(transmit_calls == 2U);
    assert(dma_data != first_dma);
    assert(strstr((const char *)dma_data, "\"scan_seq\":20") != NULL);
    assert(strstr((const char *)dma_data, "\"x_m\":-2147483.648,\"y_m\":2147483.647") != NULL);
    puts((const char *)dma_data);

    /* Abort failure must retain DMA ownership until a successful abort. */
    radar_telemetry_tx_error();
    abort_result = HAL_ERROR;
    radar_telemetry_process();
    assert(abort_calls == 1U && in_flight_index >= 0);
    abort_result = HAL_OK;
    radar_telemetry_process();
    assert(abort_calls == 2U && in_flight_index < 0);

    /* HAL_BUSY retries later and consolidates pending snapshots. */
    test_tick += REPORT_INTERVAL_MS;
    transmit_result = HAL_BUSY;
    radar_telemetry_process();
    assert(in_flight_index < 0);
    test_snapshot.scan_seq = 21U;
    test_tick += REPORT_INTERVAL_MS;
    radar_telemetry_process();
    transmit_result = HAL_OK;
    radar_telemetry_process();
    assert(in_flight_index >= 0);
    assert(strstr((const char *)dma_data, "\"scan_seq\":21") != NULL);

    /* No truncated report can become READY or enter DMA. */
    tx_buffer_t tiny_check = {.length = TX_BUFFER_SIZE - 2U};
    assert(!append_json(&tiny_check, "abc"));
    assert(tiny_check.length == TX_BUFFER_SIZE - 2U);
    test_snapshot.count = N10P_MAX_OBSTACLES + 1U;
    tx_buffer_t invalid_check = {0};
    assert(!format_report(&invalid_check));
    puts("telemetry tests passed");
    return 0;
}
