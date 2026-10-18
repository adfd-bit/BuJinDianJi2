#ifndef N10_RADAR_H
#define N10_RADAR_H

/*
 * n10_radar.h —— 雷达数据接收解析模块的对外接口
 *
 * 职责：把 USART1 收到的雷达原始字节流解析成"障碍物快照"
 * （障碍物坐标、前方距离、阻塞标志），供 radar_telemetry 打包上报。
 * 具体实现、算法与数据流说明见 n10_radar.c。
 */

#include "common.h"

/* ---- 协议常量 ---- */
#define N10P_FRAME_LEN 108U        /* 旧 N10 帧长：108 字节 */
#define N10P_POINTS_PER_FRAME 32U  /* 旧 N10 每帧点数 */
#define LD06_FRAME_LEN 47U         /* LDROBOT 帧长：47 字节（实物使用） */
#define LD06_POINTS_PER_FRAME 12U  /* LDROBOT 每帧点数 */
#define STL06_STATUS_FRAME_LEN 6U  /* 实物每约 500 ms 夹带的状态包长度 */
#define STL06NBJ_BAUD_RATE 230400U /* 雷达串口波特率 */

/* ---- 识别参数 ---- */
#define N10P_MAX_OBSTACLES 8U      /* 单次快照最多输出的目标数（按距离取最近的） */
#define N10P_SCAN_TIMEOUT_MS 500U  /* 超过该时间没有新完整扫描则结果失效 */

/* ---- 雷达安装参数（相对车体中心，单位 mm / 度） ---- */
#define N10P_MOUNT_OFFSET_RIGHT_MM 106   /* 右偏量 */
#define N10P_MOUNT_OFFSET_FORWARD_MM 106 /* 前偏量 */
#define N10P_MOUNT_HEADING_DEG 0.0f      /* 雷达 0° 方向相对车头的安装角 */

/* 单个障碍物（分组中心）在小车坐标系下的位置，单位 mm：X 向右、Y 向前 */
typedef struct {
    int32_t x_mm;
    int32_t y_mm;
} n10p_obstacle_t;

/* 一次完整扫描的结果快照；整份拷贝给调用方，调用方无需关心内部状态 */
typedef struct {
    bool valid;           /* 快照是否有效（有完整扫描且未超时） */
    uint32_t scan_seq;    /* 完整扫描序号，每发布一圈 +1 */
    uint32_t age_ms;      /* 距最近一圈发布的毫秒数；从未完成扫描时为 UINT32_MAX */
    bool front_known;     /* 前方 ±30° 是否测到过有效点 */
    uint32_t front_mm;    /* 前方 ±30° 最近距离(mm)；未知时为 UINT32_MAX */
    bool blocked;         /* 前方 0.5 m 内是否有障碍物（仅在 front_known 为真时有意义） */
    uint8_t count;        /* obstacles 中有效项数 */
    n10p_obstacle_t obstacles[N10P_MAX_OBSTACLES]; /* 按到车体中心距离由近到远排列 */
} n10p_obstacle_snapshot_t;

/* 初始化解析模块：清统计、清快照、复位解析状态（在 main 中调用一次） */
void n10p_radar_init(void);
/* 主循环任务：解析环形缓冲中的字节并更新快照 */
void n10p_radar_process(void);
/* 主循环在 UART 接收出错后调用，使扫描重新同步 */
void n10p_radar_notify_rx_error(void);
/* 取当前快照（含年龄），内部会先处理超时失效 */
void n10p_get_obstacle_snapshot(n10p_obstacle_snapshot_t *out);
/* 当前是否有有效的完整扫描 */
bool n10p_radar_is_valid(void);
/* 旧接口名，等同于 n10p_radar_is_valid() */
bool n10p_scan_is_ready(void);
/* 累计校验通过的帧数 / 坏帧数，用于诊断 */
uint32_t n10p_radar_valid_frames(void);
uint32_t n10p_radar_bad_frames(void);

#endif
