/*
 * radar_telemetry.c —— 障碍物结果上报（USART2 + DMA）
 *
 * 功能：每 100 ms 把 n10_radar 给出的障碍物快照格式化成一行 ASCII JSON，
 *       经 USART2(PA2) 用 DMA 发给电脑端可视化程序。
 *
 * 缓冲管理：两个 1024 字节缓冲，各有 FREE / READY / IN_FLIGHT 三态。
 *   主循环负责填数据、挑缓冲、启动 DMA；
 *   中断回调只置"完成/出错"标志，真正的状态切换回到主循环做，
 *   这样 DMA 正在读取的缓冲不会被改写，也不需要关中断保护长临界区。
 *   如遇发送拥塞，用最新快照覆盖尚未发出的旧快照（只保留最新数据）。
 *
 * 报告格式（每行一条，LF 结尾，详见 docs/RADAR_PORTING_GUIDE.md）：
 *   {"v":1,"seq":..,"scan_seq":..,"valid":..,"age_ms":..,"front_m":..,
 *    "blocked":..,"obstacles":[{"x_m":..,"y_m":..}],"rx_frames":..,
 *    "bad_frames":..,"rx_dropped":..,"uart_errors":..}
 */

#include "radar_telemetry.h"
#include "n10_radar.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>

#define REPORT_INTERVAL_MS 100U /* 上报周期：每 100 ms 一条 JSON */
#define TX_BUFFER_SIZE 1024U    /* 单个发送缓冲大小，需容纳最长一条报告 */
#define TX_BUFFER_COUNT 2U      /* 缓冲个数：一个在发、另一个可填 */

/* 发送缓冲状态机：空闲 → 已就绪(待发) → 发送中(DMA 占用) → 空闲 */
typedef enum {
    TX_FREE,
    TX_READY,
    TX_IN_FLIGHT
} tx_buffer_state_t;

/* 一个发送缓冲：JSON 文本 + 实际长度 + 当前状态 */
typedef struct {
    char data[TX_BUFFER_SIZE];
    uint16_t length;
    tx_buffer_state_t state;
} tx_buffer_t;

static tx_buffer_t tx_buffers[TX_BUFFER_COUNT];
static int8_t in_flight_index;            /* 正在 DMA 发送的缓冲下标，-1 表示无 */
static volatile uint8_t tx_complete_pending; /* 中断置位：上次发送已完成 */
static volatile uint8_t tx_error_pending;    /* 中断置位：上次发送出错 */
static uint32_t last_report_ms;           /* 上一条报告的生成时刻 */
static uint32_t report_sequence;          /* 报告序号，每条 +1 */

/* 按 printf 风格把文本追加到缓冲末尾。
 * 容量不足时返回 false 并保持原长度，由调用方整条丢弃——
 * 宁可少发一条，也不发出被截断的 JSON */
static bool append_json(tx_buffer_t *buffer, const char *format, ...)
{
    size_t remaining = sizeof(buffer->data) - buffer->length;
    va_list args;
    va_start(args, format);
    int written = vsnprintf(&buffer->data[buffer->length], remaining, format, args);
    va_end(args);
    if (written < 0 || (size_t)written >= remaining) {
        return false;
    }
    buffer->length = (uint16_t)(buffer->length + (uint16_t)written);
    return true;
}

/* 毫米转米并格式化成 "整数.三位小数"，全部用整数运算，不依赖浮点 printf */
static void unsigned_meters(char *text, size_t capacity, uint32_t millimeters)
{
    (void)snprintf(text, capacity, "%lu.%03lu",
                   (unsigned long)(millimeters / 1000U),
                   (unsigned long)(millimeters % 1000U));
}

/* 带符号版本：坐标可能为负（目标在车体左侧/后方）。
 * 先把绝对值转成无符号数再拼符号，避免 INT32_MIN 取反溢出 */
static void signed_meters(char *text, size_t capacity, int32_t millimeters)
{
    uint32_t magnitude = millimeters < 0 ?
                         (uint32_t)(-(int64_t)millimeters) : (uint32_t)millimeters;
    (void)snprintf(text, capacity, "%s%lu.%03lu", millimeters < 0 ? "-" : "",
                   (unsigned long)(magnitude / 1000U),
                   (unsigned long)(magnitude % 1000U));
}

/* 把当前障碍物快照格式化成一条完整 JSON 报告，成功返回 true。
 * 扫描无效时输出 valid:false、空障碍物表、前方信息为 null；
 * 末尾附带各类接收统计，便于电脑端区分"串口不通""雷达无数据""解析出错"。 */
static bool format_report(tx_buffer_t *buffer)
{
    n10p_obstacle_snapshot_t snapshot;
    char age[16];
    char front[20];
    char x[20];
    char y[20];

    n10p_get_obstacle_snapshot(&snapshot);
    buffer->length = 0U;  /* 复用缓冲前先复位长度 */
    /* age_ms 为 UINT32_MAX 表示从未完成过完整扫描，输出 null 而不是巨大数字 */
    if (snapshot.age_ms == UINT32_MAX) {
        (void)snprintf(age, sizeof(age), "null");
    } else {
        (void)snprintf(age, sizeof(age), "%lu", (unsigned long)snapshot.age_ms);
    }
    /* 前方距离只有"扫描有效且测到过前方点"时才有意义，否则为未知 */
    bool front_known = snapshot.valid && snapshot.front_known;
    if (front_known) {
        unsigned_meters(front, sizeof(front), snapshot.front_mm);
    } else {
        (void)snprintf(front, sizeof(front), "null");
    }
    /* 报告头：协议版本、报告/扫描序号、有效性、年龄、前方距离与阻塞标志 */
    if (!append_json(buffer,
            "{\"v\":1,\"seq\":%lu,\"scan_seq\":%lu,\"valid\":%s,"
            "\"age_ms\":%s,\"front_m\":%s,\"blocked\":%s,\"obstacles\":[",
            (unsigned long)++report_sequence, (unsigned long)snapshot.scan_seq,
            snapshot.valid ? "true" : "false", age, front,
            front_known ? (snapshot.blocked ? "true" : "false") : "null")) {
        return false;
    }
    /* 扫描无效时不输出任何残留目标；count 是内部状态，异常值直接判为格式化失败 */
    uint8_t count = snapshot.valid ? snapshot.count : 0U;
    if (count > N10P_MAX_OBSTACLES) {
        return false;
    }
    /* 逐个输出目标坐标（米，三位小数），首项前不加逗号 */
    for (uint8_t index = 0U; index < count; index++) {
        signed_meters(x, sizeof(x), snapshot.obstacles[index].x_mm);
        signed_meters(y, sizeof(y), snapshot.obstacles[index].y_mm);
        if (!append_json(buffer, "%s{\"x_m\":%s,\"y_m\":%s}",
                         index == 0U ? "" : ",", x, y)) {
            return false;
        }
    }
    /* 收尾：数据统计 + 换行结束这条记录 */
    return append_json(buffer,
        "],\"rx_frames\":%lu,\"bad_frames\":%lu,\"rx_dropped\":%lu,\"uart_errors\":%lu}\n",
        (unsigned long)n10p_radar_valid_frames(),
        (unsigned long)n10p_radar_bad_frames(),
        (unsigned long)uart1_rx_buf.dropped, (unsigned long)radar_uart_errors());
}

/* 初始化上报模块：全部缓冲置空闲，清标志与计数。
 * last_report_ms 回拨一个周期，使主循环首次进入就立即产生第一条报告 */
void radar_telemetry_init(void)
{
    for (uint8_t index = 0U; index < TX_BUFFER_COUNT; index++) {
        tx_buffers[index].state = TX_FREE;
        tx_buffers[index].length = 0U;
    }
    in_flight_index = -1;
    tx_complete_pending = 0U;
    tx_error_pending = 0U;
    last_report_ms = HAL_GetTick() - REPORT_INTERVAL_MS;
    report_sequence = 0U;
}

/* USART2 发送完成中断回调（DMA 已发完）：只置标志，状态切换留给主循环，避免中断里改缓冲状态 */
void radar_telemetry_tx_complete(void)
{
    tx_complete_pending = 1U;
}

/* USART2 发送出错中断回调：只置标志，收尾由主循环处理 */
void radar_telemetry_tx_error(void)
{
    tx_error_pending = 1U;
}

/* 在主循环中处理中断留下的完成/出错标志：
 * 短暂关中断取走标志并清零（避免与中断竞争），
 * 出错时先中止 DMA 发送，再把对应缓冲归还为空闲状态以便复用 */
static void finish_transmission(void)
{
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    bool completed = tx_complete_pending != 0U;
    bool failed = tx_error_pending != 0U;
    tx_complete_pending = 0U;
    tx_error_pending = 0U;
    __set_PRIMASK(interrupt_state);

    /* 发送失败：先中止 DMA，失败则保留错误标志下轮重试 */
    if (failed && HAL_UART_AbortTransmit(&huart2) != HAL_OK) {
        tx_error_pending = 1U;
        return;
    }
    /* 正常完成或已中止：释放发送中的缓冲 */
    if ((completed || failed) && in_flight_index >= 0) {
        tx_buffers[(uint8_t)in_flight_index].state = TX_FREE;
        in_flight_index = -1;
    }
}

/* 选取一个缓冲生成最新报告：
 * 优先使用已在排队的 READY 缓冲（用最新快照覆盖尚未发出的旧数据），
 * 否则使用空闲缓冲；两个缓冲都在发送中则本次跳过，等下一周期。
 * 格式化失败（长度不够）时缓冲退回空闲，不发送不完整数据 */
static void queue_latest_report(void)
{
    int8_t candidate = -1;
    for (uint8_t index = 0U; index < TX_BUFFER_COUNT; index++) {
        if (tx_buffers[index].state == TX_READY) {
            candidate = (int8_t)index;
            break;
        }
        if (tx_buffers[index].state == TX_FREE) {
            candidate = (int8_t)index;
        }
    }
    if (candidate < 0) {
        return;
    }
    tx_buffer_t *buffer = &tx_buffers[(uint8_t)candidate];
    buffer->state = format_report(buffer) ? TX_READY : TX_FREE;
}

/* 若有 READY 缓冲且当前没有正在发送的，就交给 DMA 发出去。
 * 启动失败时把缓冲退回 READY，下个周期重试 */
static void begin_transmission(void)
{
    if (in_flight_index >= 0 || tx_error_pending != 0U) {
        return;
    }
    for (uint8_t index = 0U; index < TX_BUFFER_COUNT; index++) {
        tx_buffer_t *buffer = &tx_buffers[index];
        if (buffer->state != TX_READY) {
            continue;
        }
        buffer->state = TX_IN_FLIGHT;
        in_flight_index = (int8_t)index;
        HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(&huart2,
            (uint8_t *)buffer->data, buffer->length);
        if (result != HAL_OK) {
            in_flight_index = -1;
            buffer->state = TX_READY;
        }
        break;
    }
}

/* 主循环任务，三步走：
 * ① 处理上一次 DMA 发送的结果（完成/出错）
 * ② 距上次报告满 100 ms 就生成一条最新快照报告
 * ③ 有就绪报告且空闲就启动 DMA 发送 */
void radar_telemetry_process(void)
{
    finish_transmission();
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_report_ms) >= REPORT_INTERVAL_MS) {
        last_report_ms = now;
        queue_latest_report();
    }
    begin_transmission();
}
