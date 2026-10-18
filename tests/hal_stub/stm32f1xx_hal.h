#ifndef TEST_STM32F1XX_HAL_H
#define TEST_STM32F1XX_HAL_H

#include <stdint.h>
typedef struct { void *Instance; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);

#endif
