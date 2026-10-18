/*
 * n10_radar.c —— 雷达数据接收解析与障碍物识别
 *
 * 数据流：
 *   USART1 中断逐字节收 → uart1_rx_buf 环形缓冲（见 usart.c）
 *   n10p_radar_process() 在主循环中把字节流拼成完整帧，再按下列步骤处理：
 *     ① 帧同步：识别帧头（0x54=LDROBOT，0xA5=旧 N10），拼满一帧
 *     ② 校验：LD06 用 CRC8，旧 N10 用累加和低八位
 *     ③ 逐点解析：由起止角插值算出每点角度，角度回绕跨 0° 即为一圈结束
 *     ④ 分圈：跨 0° 时 publish_scan() 发布上一圈，重新累积新一圈
 *     ⑤ 坐标换算：补偿雷达安装偏移，得到小车坐标系下的点（mm）
 *     ⑥ 安全判断：前方 ±30° 扇区内取最近距离，<0.5 m 置 blocked
 *     ⑦ 分组聚类：用 collect_point()/finish_groups() 把相邻点聚成目标
 *     ⑧ 结果存入 snapshot，由 n10p_get_obstacle_snapshot() 供上报模块读取
 *
 * 支持两种协议（自动识别帧头，可混线）：
 *   - LDROBOT 兼容格式（实物 STL-06NBJ 使用）：0x54 0x2C 帧头，47 字节，12 点，CRC8
 *   - 旧 N10 格式：0xA5 0x5A 0x6C 帧头，108 字节，32 点，累加和校验
 *
 * 所有函数都在主循环上下文执行，不在中断中调用，因此无需加锁；
 * 与中断的唯一交接点是环形缓冲（写入侧）和 UART 错误标志（见 usart.c）。
 */

#include "n10_radar.h"
#include <math.h>

/* 测距有效范围(mm)：超出范围的读数按噪声/无效处理 */
#define VALID_MIN_MM 50U
#define VALID_MAX_MM 6000U
/* 同属一个目标的相邻点允许的最大角度间隔(度) */
#define MAX_POINT_GAP_DEG 3U
/* 同属一个目标的相邻点允许的最大间距平方(mm²)，22500 = 150 mm² */
#define MAX_POINT_DISTANCE_SQ 22500.0f
/* 角度转弧度系数（π/180） */
#define DEG_TO_RAD 0.017453292519943295f

/* LD06/LD19 帧校验用 CRC8 查表（多项式 0x4D，初值 0，不反射）。
 * LDROBOT SDK lipkg.cpp, Copyright (c) 2021 SHENZHEN LDROBOT CO., LTD.
 * MIT licensed: https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk */
static const uint8_t ld06_crc_table[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
};

/* 换算到小车坐标系后的一个测距点：x 向右、y 向前，单位为 mm 浮点数；
 * angle 保留雷达原始角度(度)，用于判断相邻点是否连续 */
typedef struct {
    float x;
    float y;
    float angle;
} scan_point_t;

/* 一次"连续目标"分组的累加器：记录首/末点（判断连续性）以及坐标和与采样数
 * （最后用平均值作为目标位置），避免为分组另开数组 */
typedef struct {
    scan_point_t first;
    scan_point_t last;
    float sum_x;
    float sum_y;
    uint32_t samples;
} point_group_t;

/* ---- 运行统计（通过 n10p_radar_valid_frames/bad_frames 对外提供） ---- */
static uint32_t valid_frames;     /* 校验并解析成功的帧数 */
static uint32_t bad_frames;       /* 校验失败或帧头错乱的帧数 */
static uint32_t last_frame_tick;  /* 最近一帧有效数据的时刻(ms)，用于接收超时判断 */
static uint32_t last_scan_tick;   /* 最近一圈发布的时刻(ms)，用于扫描超时判断 */
static uint32_t observed_dropped; /* 已知的环形缓冲丢字节计数，用于检测中途溢出 */

/* ---- 组帧状态 ---- */
static uint8_t frame_bytes[N10P_FRAME_LEN]; /* 正在拼装的帧，按最长帧(108 字节)分配 */
static uint8_t frame_index;                 /* 已收字节数，0 表示正在等待帧头 */
static uint8_t frame_length;                /* 当前帧的预期总长度，随帧头类型变化 */

/* ---- 分组(聚类)状态 ---- */
static point_group_t first_group;   /* 跨 0° 处暂存的第一组，等圈末判断能否与末组合并 */
static point_group_t current_group; /* 正在累积的分组 */
static bool current_exists;         /* current_group 是否有效 */
static bool first_saved;            /* first_group 是否已暂存 */

/* ---- 正在构建的扫描结果（本圈累积） ---- */
static n10p_obstacle_snapshot_t building_snapshot; /* 已识别的目标，按距离由近到远排列 */
static float previous_angle;      /* 上一个点的角度，用于判断角度递增与跨 0° */
static bool previous_angle_valid; /* previous_angle 是否可用（上电首点无效） */
static bool scan_started;         /* 是否已跨过 0°、进入有效扫描圈 */
static bool receive_active;       /* 是否正在接收数据，用于区分"未接雷达"和"数据出错" */
static bool has_completed_scan;   /* 是否发布过至少一圈完整扫描 */
static bool building_front_known; /* 本圈前方 ±30° 是否测到过有效点 */
static bool building_blocked;     /* 本圈前方 0.5 m 内是否存在障碍物 */
static float building_front_sq;   /* 本圈前方最近距离的平方(mm²)，用平方避免开方 */

/* ---- 对外发布的快照（一圈结束时整体更新） ---- */
static n10p_obstacle_snapshot_t snapshot;

/* 按大端(高字节在前)读取 16 位无符号数，旧 N10 帧使用 */
static uint16_t read_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* 按小端(低字节在前)读取 16 位无符号数，LDROBOT 帧使用 */
static uint16_t read_le_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/* 查表计算 CRC8，length 为参与校验的字节数（不含末尾校验字节本身） */
static uint8_t ld06_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    while (length-- != 0U) crc = ld06_crc_table[crc ^ *data++];
    return crc;
}

/* 把角度归一化到 [0,360)：插值结果最多超出一圈，减一次即可 */
static float normalize_degrees(float angle)
{
    if (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

/* 浮点毫米转整数毫米，远离零点方向取整（等价四舍五入），保证正负对称 */
static int32_t round_mm(float value)
{
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

/* 清空"本圈累积"状态，准备重新开始一圈：
 * 前方距离用最大值 UINT32_MAX 作哨兵，取最小值时任何有效点都会覆盖它 */
static void clear_building_scan(void)
{
    memset(&building_snapshot, 0, sizeof(building_snapshot));
    current_exists = first_saved = false;
    building_front_known = building_blocked = false;
    building_front_sq = (float)UINT32_MAX;
}

/* 使当前扫描彻底失效并复位全部解析状态（数据流不可信时调用）。
 * discard_received=true 表示环形缓冲里已收到但未解析的字节也已经不可信
 * （UART 错误、丢字节），直接丢掉，等下一个帧头重新同步；
 * false 表示仍相信缓冲内容，只是当前这圈作废。 */
static void invalidate_scan(bool discard_received)
{
    clear_building_scan();
    previous_angle_valid = scan_started = receive_active = false;
    frame_index = 0U;
    frame_length = 0U;
    snapshot.valid = snapshot.front_known = snapshot.blocked = false;
    snapshot.front_mm = UINT32_MAX;
    snapshot.count = 0U;
    memset(snapshot.obstacles, 0, sizeof(snapshot.obstacles));
    if (discard_received) uart1_rx_buf.tail = uart1_rx_buf.head;
}

/* 超时检查：超过 N10P_SCAN_TIMEOUT_MS 没有新扫描或新帧，说明雷达断连，
 * 立即让结果失效，避免把陈旧数据当成实时障碍物（失效本身会清标志，只会触发一次） */
static void expire_scan(uint32_t now)
{
    /* Invalidation clears these flags, so an expired timestamp resets once. */
    if ((snapshot.valid && (uint32_t)(now - last_scan_tick) >= N10P_SCAN_TIMEOUT_MS) ||
        (receive_active && (uint32_t)(now - last_frame_tick) >= N10P_SCAN_TIMEOUT_MS)) {
        invalidate_scan(true);
    }
}

/* 把"雷达角度 + 距离"换算成小车坐标系下的点（单位 mm）：
 *   x = 右偏安装补偿 + r·sin(θ+安装角)
 *   y = 前偏安装补偿 + r·cos(θ+安装角)
 * 角度定义 0° 指向车头、90° 指向右侧，安装角由 N10P_MOUNT_HEADING_DEG 给出 */
static scan_point_t point_for_angle(float angle, uint16_t distance)
{
    float radians = (angle + N10P_MOUNT_HEADING_DEG) * DEG_TO_RAD;
    scan_point_t point;
    point.x = (float)distance * sinf(radians) + N10P_MOUNT_OFFSET_RIGHT_MM;
    point.y = (float)distance * cosf(radians) + N10P_MOUNT_OFFSET_FORWARD_MM;
    point.angle = angle;
    return point;
}

/* 判断相邻两点是否属于同一目标：角度间隔不超过 3° 且实际间距不超过 150 mm。
 * across_zero=true 时用于圈末与圈首的比较（跨 0° 的接缝），角度差需补 360°
 * 再计算，否则会被误判成间隔很大的两个目标 */
static bool points_connected(const scan_point_t *a, const scan_point_t *b, bool across_zero)
{
    float gap = across_zero ? 360.0f + b->angle - a->angle : b->angle - a->angle;
    float dx = b->x - a->x;
    float dy = b->y - a->y;
    return gap >= 0.0f && gap <= MAX_POINT_GAP_DEG && dx * dx + dy * dy <= MAX_POINT_DISTANCE_SQ;
}

/* 以给定点作为首点，开始一个新分组 */
static void group_start(point_group_t *group, scan_point_t point)
{
    group->first = group->last = point;
    group->sum_x = point.x;
    group->sum_y = point.y;
    group->samples = 1U;
}

/* 把点追加到分组末尾，同时累加坐标和与采样数（供最后求中心位置） */
static void group_append(point_group_t *group, scan_point_t point)
{
    group->last = point;
    group->sum_x += point.x;
    group->sum_y += point.y;
    group->samples++;
}

/* 目标到车体中心距离的平方：排序/比较只用平方，避免开方开销 */
static uint32_t obstacle_distance_sq(const n10p_obstacle_t *obstacle)
{
    return (uint32_t)(obstacle->x_mm * obstacle->x_mm + obstacle->y_mm * obstacle->y_mm);
}

/* 一个分组结束时调用，把分组转成一个目标存入结果列表：
 * - 少于 3 个采样点的组按噪声丢弃
 * - 用组内坐标平均值作为目标位置（可见表面中心，不是物体几何中心）
 * - 按到车体中心的距离做插入排序（近的在前），只保留最近的 8 组 */
static void add_group(const point_group_t *group)
{
    if (group->samples < 3U) return;
    n10p_obstacle_t obstacle;
    obstacle.x_mm = round_mm(group->sum_x / (float)group->samples);
    obstacle.y_mm = round_mm(group->sum_y / (float)group->samples);
    uint32_t distance_sq = obstacle_distance_sq(&obstacle);
    /* 找到第一个比新目标更远的槽位，即为插入位置 */
    uint8_t position = 0U;
    while (position < building_snapshot.count &&
           obstacle_distance_sq(&building_snapshot.obstacles[position]) <= distance_sq) position++;
    /* 比现有 8 个目标都远，直接丢弃（列表只保留最近的） */
    if (position >= N10P_MAX_OBSTACLES) return;
    if (building_snapshot.count < N10P_MAX_OBSTACLES) building_snapshot.count++;
    /* 后移腾位后插入，保持按距离升序 */
    for (uint8_t i = building_snapshot.count - 1U; i > position; --i) {
        building_snapshot.obstacles[i] = building_snapshot.obstacles[i - 1U];
    }
    building_snapshot.obstacles[position] = obstacle;
}

/* 逐点输入的分组状态机：与当前组末点相连则并入，否则结束当前组并另起一组。
 * 由于圈末与圈首可能在 0° 处属于同一目标，第一个分组先暂存不处理，
 * 等 finish_groups() 在圈末判断接缝能否合并 */
static void collect_point(scan_point_t point)
{
    if (!current_exists) {
        group_start(&current_group, point);
        current_exists = true;
    } else if (points_connected(&current_group.last, &point, false)) {
        group_append(&current_group, point);
    } else {
        /* 第一个分组先暂存，等圈末检查跨 0° 接缝后再决定是否单独输出 */
        if (!first_saved) {
            first_group = current_group;
            first_saved = true;
        } else {
            add_group(&current_group);
        }
        group_start(&current_group, point);
    }
}

/* 一圈数据结束时收尾分组，处理跨 0° 的接缝：
 * - 只有一个分组：直接输出
 * - 首组与末组在 0° 处相连：合并成一个目标再输出
 * - 不相连：两个目标分别输出 */
static void finish_groups(void)
{
    if (current_exists) {
        if (!first_saved) {
            add_group(&current_group);
        } else if (points_connected(&current_group.last, &first_group.first, true)) {
            current_group.sum_x += first_group.sum_x;
            current_group.sum_y += first_group.sum_y;
            current_group.samples += first_group.samples;
            add_group(&current_group);
        } else {
            add_group(&first_group);
            add_group(&current_group);
        }
    }
}

/* 发布一圈完整扫描：把本圈累积结果整体拷入 snapshot，
 * 前方距离按平方根换算回毫米（本圈没测到前方点时置 UINT32_MAX 表示未知），
 * 清空累积状态准备下一圈，扫描序号自增并记录发布时刻供超时判断 */
static void publish_scan(uint32_t now)
{
    finish_groups();
    uint32_t next_seq = snapshot.scan_seq + 1U;
    snapshot = building_snapshot;
    snapshot.front_known = building_front_known;
    snapshot.blocked = building_blocked;
    snapshot.front_mm = building_front_known ? (uint32_t)round_mm(sqrtf(building_front_sq)) : UINT32_MAX;
    clear_building_scan();
    snapshot.scan_seq = next_seq;
    snapshot.valid = true;
    has_completed_scan = true;
    last_scan_tick = now;
}

/* 校验并解析一帧完整数据（frame_bytes/frame_length 已由 n10p_radar_process 填好），
 * 成功返回 true。依次完成：协议识别与校验 → 提取起止角 → 逐点插值角度并跨 0° 分圈
 * → 距离有效性过滤 → 坐标换算 + 前方 ±30° 安全判断 → 交给分组状态机。
 * 距离无效的点同样会推进角度，保证分圈基准不丢。 */
static bool parse_frame(uint32_t now)
{
    /* 帧头 0x54 = LDROBOT 格式，0xA5 = 旧 N10 格式 */
    bool ld06 = frame_bytes[0] == 0x54U;
    if (ld06 && frame_bytes[1] == 0x1FU) {
        /* 实物每约 500 ms 夹带的 6 字节状态包（证据：tests/artifacts/hardware-rx-09.bin）。
         * 只接受已确认的全零状态内容，其他含义未知；
         * 状态包不计入测距帧，也不刷新点云时间戳（即不影响扫描有效期）。 */
        return frame_bytes[2] == 0U && frame_bytes[3] == 0U && frame_bytes[4] == 0U &&
               ld06_crc8(frame_bytes, STL06_STATUS_FRAME_LEN - 1U) ==
               frame_bytes[STL06_STATUS_FRAME_LEN - 1U];
    }
    if (ld06) {
        /* LDROBOT：前 46 字节 CRC8 与最后一个字节比较 */
        if (ld06_crc8(frame_bytes, LD06_FRAME_LEN - 1U) != frame_bytes[LD06_FRAME_LEN - 1U]) return false;
    } else {
        /* 旧 N10：前 107 字节累加和的低八位（不是 CRC8） */
        uint16_t sum = 0U;
        for (uint16_t i = 0U; i < N10P_FRAME_LEN - 1U; ++i) sum += frame_bytes[i];
        if ((uint8_t)sum != frame_bytes[N10P_FRAME_LEN - 1U]) return false;
    }
    /* 提取本帧的起始角与结束角（单位 0.01°） */
    uint16_t start_raw = ld06 ? read_le_u16(&frame_bytes[4]) : read_be_u16(&frame_bytes[5]);
    uint16_t end_raw = ld06 ? read_le_u16(&frame_bytes[42]) : read_be_u16(&frame_bytes[105]);
    if (start_raw > 36000U || end_raw > 36000U) return false;
    /* 36000 表示 0°，统一折算成 0 避免算出负跨度 */
    float start = start_raw == 36000U ? 0.0f : (float)start_raw * 0.01f;
    float end = end_raw == 36000U ? 0.0f : (float)end_raw * 0.01f;
    float span = end - start;
    if (span < 0.0f) span += 360.0f;
    /* 一帧只覆盖一小段圆弧，跨度 ≥180° 说明端点错乱或数据损坏 */
    if (span >= 180.0f) return false;

    uint8_t point_count = ld06 ? LD06_POINTS_PER_FRAME : N10P_POINTS_PER_FRAME;
    float intervals = ld06 ? (float)(LD06_POINTS_PER_FRAME - 1U) : (float)N10P_POINTS_PER_FRAME;
    for (uint8_t i = 0U; i < point_count; ++i) {
        /* 在起止角之间均匀插值出每个点的角度。
         * LDROBOT 的结束角包含在点序列内，用 i/11；旧 N10 的结束角是边界，用 i/32。 */
        float angle = normalize_degrees(start + span * (float)i / intervals);
        /* 角度回绕检测：前一点 ~360°、当前点 ~0° 时视为跨 0°（一圈结束） */
        bool wrapped = previous_angle_valid && previous_angle - angle > 180.0f;
        /* 角度非回绕地倒退，或单步跳变 ≥180°，说明丢了数据：本圈结果立即作废 */
        if (previous_angle_valid && ((angle < previous_angle && !wrapped) ||
                                     angle - previous_angle >= 180.0f)) {
            invalidate_scan(false);
        }
        if (wrapped) {
            /* 跨 0° 即一圈结束：先发布上一圈，再开始累积新一圈。
             * 同一帧内部也可能跨 0°，所以判断放在逐点循环里而不是帧尾。 */
            if (scan_started) publish_scan(now);
            else {
                /* 上电后第一次跨 0° 只用来确定圈的起点，不发布结果 */
                clear_building_scan();
                scan_started = true;
            }
        }
        previous_angle = angle;
        previous_angle_valid = true;
        /* 距离字段位置：LDROBOT 从第 6 字节起，旧 N10 从第 7 字节起，每点 3 字节 */
        uint8_t offset = (uint8_t)((ld06 ? 6U : 7U) + i * 3U);
        /* LDROBOT 为小端 16 位；旧 N10 为 15 位（高字节最高位是无效标志，须屏蔽） */
        uint16_t distance = ld06 ? read_le_u16(&frame_bytes[offset]) :
                           (uint16_t)(((uint16_t)(frame_bytes[offset] & 0x7FU) << 8) |
                                      frame_bytes[offset + 1U]);
        /* 距离超范围的点不参与分组，但上面的角度推进已经生效，分圈不受影响 */
        if (!scan_started || distance < VALID_MIN_MM || distance > VALID_MAX_MM) continue;
        /* 分组与前方判断都使用换算后的原始点，不做量化取舍 */
        scan_point_t point = point_for_angle(angle, distance);
        /* 前方 ±30° 扇区判断：只统计车头方向(y>0)且 |x| ≤ y·tan30° 的点，
         * tan30°≈0.57735026919 */
        if (point.y > 0.0f && fabsf(point.x) <= point.y * 0.57735026919f) {
            float distance_sq = point.x * point.x + point.y * point.y;
            building_front_known = true;
            if (distance_sq < building_front_sq) building_front_sq = distance_sq;
            /* 250000 mm² = 500 mm：前方 0.5 m 内有障碍物 → 置阻塞标志 */
            if (distance_sq < 250000.0f) building_blocked = true;
        }
        collect_point(point);
    }
    /* 本帧有效：更新统计与"正在接收"时间戳，供超时判断使用 */
    valid_frames++;
    last_frame_tick = now;
    receive_active = true;
    return true;
}

/* 初始化：清空统计和快照，复位帧解析/分圈/分组状态。
 * 记录当前丢字节计数作为基准，之后才能检测出新增的溢出。
 * 上电后第一次跨 0° 只确定起点，因此需要两圈才能产生第一个有效结果。 */
void n10p_radar_init(void)
{
    valid_frames = bad_frames = last_frame_tick = last_scan_tick = 0U;
    has_completed_scan = false;
    memset(&snapshot, 0, sizeof(snapshot));
    invalidate_scan(false);
    observed_dropped = uart1_rx_buf.dropped;
}

/* UART 接收出现错误（溢出/帧错/噪声等）后由主循环调用：
 * 数据流已不可信，丢弃缓冲区残留字节并让扫描重新同步 */
void n10p_radar_notify_rx_error(void)
{
    invalidate_scan(true);
    observed_dropped = uart1_rx_buf.dropped;
}

/* 主循环任务：把 USART1 环形缓冲中的字节推进组帧状态机。
 * 一次调用会尽量处理完缓冲中所有字节，因此单次耗时可观，但不会阻塞等待新数据。 */
void n10p_radar_process(void)
{
    expire_scan(HAL_GetTick());   /* 先处理扫描/接收超时 */
    while (!ring_buf_empty(&uart1_rx_buf)) {
        uint32_t dropped = uart1_rx_buf.dropped;
        if (dropped != observed_dropped) {
            /* 环形缓冲溢出丢过字节，帧边界已经丢失：丢弃数据并重新同步 */
            observed_dropped = dropped;
            invalidate_scan(true);
            continue;
        }
        uint8_t byte = ring_buf_get(&uart1_rx_buf);
        if (frame_index == 0U) {
            /* 等待帧头：0xA5=旧 N10(108 字节)，0x54=LDROBOT(47 字节，含状态包) */
            if (byte == 0xA5U || byte == 0x54U) {
                frame_length = byte == 0x54U ? LD06_FRAME_LEN : N10P_FRAME_LEN;
                frame_bytes[frame_index++] = byte;
            }
            else if (receive_active) {
                /* 正常收数据时出现杂散字节，说明码流错乱，但不必丢弃缓冲 */
                bad_frames++;
                invalidate_scan(false);
            }
            continue;
        }
        /* 逐字节核对帧头：0x54 后应跟 0x2C(测距帧) 或 0x1F(状态包)；
         * 0xA5 后应跟 0x5A，再跟 0x6C */
        if ((frame_index == 1U &&
             (frame_bytes[0] == 0x54U ? (byte != 0x2CU && byte != 0x1FU) : byte != 0x5AU)) ||
            (frame_index == 2U && frame_bytes[0] == 0xA5U && byte != 0x6CU)) {
            if (receive_active) {
                bad_frames++;
                invalidate_scan(false);
            }
            /* 帧头不匹配：当前字节本身可能就是新帧头，据此重新开始组帧 */
            frame_index = (byte == 0xA5U || byte == 0x54U) ? 1U : 0U;
            if (frame_index != 0U) {
                frame_length = byte == 0x54U ? LD06_FRAME_LEN : N10P_FRAME_LEN;
                frame_bytes[0] = byte;
            }
            continue;
        }
        /* 0x54 0x1F 为 6 字节状态包，长度与测距帧不同，需在此修正 */
        if (frame_index == 1U && frame_bytes[0] == 0x54U && byte == 0x1FU) {
            frame_length = STL06_STATUS_FRAME_LEN;
        }
        frame_bytes[frame_index++] = byte;
        if (frame_index == frame_length) {
            /* 一帧收齐：校验并解析；失败则作废本圈结果并重新同步 */
            if (!parse_frame(HAL_GetTick())) {
                bad_frames++;
                invalidate_scan(false);
            }
            frame_index = 0U;
        }
    }
    /* 上面的解析耗时期间中断可能又丢了字节，退出前再确认一次 */
    if (uart1_rx_buf.dropped != observed_dropped) {
        observed_dropped = uart1_rx_buf.dropped;
        invalidate_scan(true);
    }
}

/* 取当前障碍物快照（含年龄）供上报模块使用：
 * 先做一次超时检查；未发布过完整扫描时 age_ms 用 UINT32_MAX 表示"未知" */
void n10p_get_obstacle_snapshot(n10p_obstacle_snapshot_t *out)
{
    if (out == NULL) return;
    uint32_t now = HAL_GetTick();
    expire_scan(now);
    snapshot.age_ms = has_completed_scan ? (uint32_t)(now - last_scan_tick) : UINT32_MAX;
    *out = snapshot;
}

/* 当前是否存在有效的完整扫描（最近一圈在 500 ms 内发布）；内部先做超时检查 */
bool n10p_radar_is_valid(void)
{
    expire_scan(HAL_GetTick());
    return snapshot.valid;
}

/* 兼容旧接口名，语义与 n10p_radar_is_valid() 相同 */
bool n10p_scan_is_ready(void) { return n10p_radar_is_valid(); }
/* 累计校验通过的帧数 / 累计坏帧数，用于电脑端排查数据质量 */
uint32_t n10p_radar_valid_frames(void) { return valid_frames; }
uint32_t n10p_radar_bad_frames(void) { return bad_frames; }
