#ifndef RADAR_TELEMETRY_H
#define RADAR_TELEMETRY_H

/*
 * radar_telemetry.h —— 障碍物结果上报模块接口（USART2 + DMA）
 *
 * 【暂时可以先跳过】本模块只负责把 n10p_get_obstacle_snapshot() 交出来的
 * 结果打包成 JSON 从 USART2 发走，不参与任何雷达数据处理。
 * 雷达如何收数据、算角度、分圈、聚类，全部在 n10_radar.c。
 *
 * 每 100 ms 输出一行 JSON 报告，实现与格式说明见 radar_telemetry.c。
 */

/* 初始化缓冲池与计数（在 main 中调用一次） */
void radar_telemetry_init(void);
/* 主循环任务：处理发送收尾 → 按周期生成报告 → 启动 DMA 发送 */
void radar_telemetry_process(void);

/* 以下两个只由 UART 中断回调调用：仅置标志，
 * 缓冲状态切换等实际处理都在主循环 radar_telemetry_process() 中完成 */
void radar_telemetry_tx_complete(void);
void radar_telemetry_tx_error(void);

#endif
