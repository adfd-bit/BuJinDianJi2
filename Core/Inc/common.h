#ifndef APP_COMMON_H
#define APP_COMMON_H

/*
 * common.h —— 公共定义：USART1 接收环形缓冲
 *
 * 雷达数据在 USART1 接收中断里逐字节写入，主循环读取解析（单生产者单消费者）。
 * 读写指针 head/tail 均为 volatile，容量取 2 的幂，取模运算可被优化。
 */

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

/*
 * 串口接收缓存由中断写入、主循环读取。容量必须大于一次主循环可能积压的
 * 雷达数据量；满时丢弃新字节并累计 dropped，避免覆盖尚未解析的数据。
 */
#define UART_RX_BUF_SIZE 2048U

typedef struct {
    uint8_t data[UART_RX_BUF_SIZE];
    volatile uint16_t head;    /* 写指针：中断中前进，指向下一个写入位置 */
    volatile uint16_t tail;    /* 读指针：主循环中前进，指向下一个待读位置 */
    volatile uint32_t dropped; /* 累计丢弃字节数，>0 说明发生过溢出 */
} uart_ring_buf_t;

extern uart_ring_buf_t uart1_rx_buf;

/* 清空缓冲（head=tail 即空）。调用前应确保没有中断正在写入。 */
static inline void ring_buf_init(uart_ring_buf_t *buf)
{
    buf->head = 0U;
    buf->tail = 0U;
    buf->dropped = 0U;
}

/* 缓冲区是否为空；主循环取数据前必须先判断 */
static inline bool ring_buf_empty(const uart_ring_buf_t *buf)
{
    return buf->head == buf->tail;
}

/* 中断中调用：写入一个字节；缓冲满时丢弃该字节并累加 dropped，不阻塞、不覆盖旧数据 */
static inline void ring_buf_put(uart_ring_buf_t *buf, uint8_t byte)
{
    uint16_t next = (uint16_t)((buf->head + 1U) % UART_RX_BUF_SIZE);
    if (next == buf->tail) {
        buf->dropped++;
        return;
    }
    buf->data[buf->head] = byte;
    buf->head = next;
}

/* 主循环中调用：取出一个字节并前进读指针 */
static inline uint8_t ring_buf_get(uart_ring_buf_t *buf)
{
    /* 调用者必须先用 ring_buf_empty() 确认缓冲区非空。 */
    uint8_t byte = buf->data[buf->tail];
    buf->tail = (uint16_t)((buf->tail + 1U) % UART_RX_BUF_SIZE);
    return byte;
}

/* 当前缓冲区内待处理的字节数 */
static inline uint16_t ring_buf_count(const uart_ring_buf_t *buf)
{
    return (uint16_t)((buf->head - buf->tail + UART_RX_BUF_SIZE) % UART_RX_BUF_SIZE);
}

#endif
