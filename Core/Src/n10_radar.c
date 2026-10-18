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
 * ==================== 阅读指引：雷达处理就看本文件 ====================
 * 本文件是雷达数据处理的核心，和"发送 JSON"无关。建议按数据流只看：
 *   ① n10p_radar_process()           入口：字节流 → 拼出完整帧
 *   ② parse_frame()                  核心：校验、逐点算角度、跨 0° 分圈
 *   ③ point_for_angle()              角度 + 距离 → 小车平面坐标
 *   ④ collect_point() / add_group()  把相邻点聚类成障碍物
 *   ⑤ publish_scan()                 一圈结束：保存本圈结果快照
 *   ⑥ n10p_get_obstacle_snapshot()   交出结果（本文件到此为止）
 * JSON 打包与发送在 Core/Src/radar_telemetry.c，可以先跳过不看。
 * ======================================================================
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
/* 同属一个目标的相邻点允许的最大间距平方(mm²)，22500 = 150 mm 的平方 */
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

/* ============================================================
 * 一、小工具：读 16 位数、CRC8、角度归一化、毫米取整
 * ============================================================ */

/* 按大端(高字节在前)读取 16 位无符号数，旧 N10 帧使用 */
static uint16_t read_be_u16(const uint8_t *p)
{
    /* p[0] 是高字节，左移 8 位后与低字节 p[1] 拼成 16 位 */
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* 按小端(低字节在前)读取 16 位无符号数，LDROBOT 帧使用 */
static uint16_t read_le_u16(const uint8_t *p)
{
    /* p[1] 是高字节，左移 8 位后与低字节 p[0] 拼成 16 位 */
    return (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/* 查表计算 CRC8，length 为参与校验的字节数（不含末尾校验字节本身） */
static uint8_t ld06_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U; /* CRC 初值为 0（LDROBOT 规定） */
    /* 每处理一个字节：用"当前 CRC 异或数据字节"查表得到新 CRC，直到处理完 length 个字节 */
    while (length-- != 0U) crc = ld06_crc_table[crc ^ *data++];
    /* 返回最终算出的 8 位校验值 */
    return crc;
}

/* 把角度归一化到 [0,360)：插值结果最多超出一圈，减一次即可 */
static float normalize_degrees(float angle)
{
    /* 等于或超过 360° 时减一圈，把角度拉回 [0,360) */
    if (angle >= 360.0f) angle -= 360.0f;
    /* 返回归一化后的角度 */
    return angle;
}

/* 浮点毫米转整数毫米，远离零点方向取整（等价四舍五入），保证正负对称 */
static int32_t round_mm(float value)
{
    /* 正数加 0.5、负数减 0.5 再截断成整数，实现四舍五入且正负对称 */
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

/* ============================================================
 * 二、扫描状态管理：开始新一圈 / 出错或超时作废
 * ============================================================ */

/* 清空"本圈累积"状态，准备重新开始一圈：
 * 前方距离用最大值 UINT32_MAX 作哨兵，取最小值时任何有效点都会覆盖它 */
static void clear_building_scan(void)
{
    /* 把本圈累积的快照整体清零（障碍物列表、数量等） */
    memset(&building_snapshot, 0, sizeof(building_snapshot));
    /* 当前分组和"暂存首组"都标记为不存在 */
    current_exists = first_saved = false;
    /* 本圈前方还没测到有效点，也还没判定被挡 */
    building_front_known = building_blocked = false;
    /* 最近距离初值设为最大值，任何真实点都会比它小并覆盖它 */
    building_front_sq = (float)UINT32_MAX;
}

/* 使当前扫描彻底失效并复位全部解析状态（数据流不可信时调用）。
 * discard_received=true 表示环形缓冲里已收到但未解析的字节也已经不可信
 * （UART 错误、丢字节），直接丢掉，等下一个帧头重新同步；
 * false 表示仍相信缓冲内容，只是当前这圈作废。 */
static void invalidate_scan(bool discard_received)
{
    /* 先清空本圈正在累积的障碍物和前方判断 */
    clear_building_scan();
    /* 角度基准、分圈状态、接收状态全部复位，下一帧重新开始 */
    previous_angle_valid = scan_started = receive_active = false;
    /* 放弃当前正在拼的半帧，回到"等待帧头"状态 */
    frame_index = 0U;
    /* 清掉当前帧的预期长度 */
    frame_length = 0U;
    /* 对外快照立即标记无效，并清掉前方距离/阻塞/数量 */
    snapshot.valid = snapshot.front_known = snapshot.blocked = false;
    /* 前方距离置为"未知"哨兵值 */
    snapshot.front_mm = UINT32_MAX;
    /* 障碍物数量清零 */
    snapshot.count = 0U;
    /* 清空旧障碍物坐标，避免上层读到过期数据 */
    memset(snapshot.obstacles, 0, sizeof(snapshot.obstacles));
    /* 若连缓冲区里的字节也不可信，直接丢到"空"（tail 追上 head） */
    if (discard_received) uart1_rx_buf.tail = uart1_rx_buf.head;
}

/* 超时检查：超过 N10P_SCAN_TIMEOUT_MS 没有新扫描或新帧，说明雷达断连，
 * 立即让结果失效，避免把陈旧数据当成实时障碍物（失效本身会清标志，只会触发一次） */
static void expire_scan(uint32_t now)
{
    /* 失效处理会清掉下面两个标志，因此已过期的时刻只会触发一次作废 */
    /* 条件一：快照有效但距上一圈发布已 ≥500 ms；条件二：正在接收但距上一帧有效数据已 ≥500 ms */
    if ((snapshot.valid && (uint32_t)(now - last_scan_tick) >= N10P_SCAN_TIMEOUT_MS) ||
        (receive_active && (uint32_t)(now - last_frame_tick) >= N10P_SCAN_TIMEOUT_MS)) { /* 条件二：接收中但已 500 ms 没有有效帧 */
        /* 超时说明雷达断开或数据停止：丢弃缓冲并让结果立即失效 */
        invalidate_scan(true);
    }
}

/* ============================================================
 * 三、坐标换算与聚类：把一个个测距点变成"障碍物"
 * ============================================================ */

/* 把"雷达角度 + 距离"换算成小车坐标系下的点（单位 mm）：
 *   x = 右偏安装补偿 + r·sin(θ+安装角)
 *   y = 前偏安装补偿 + r·cos(θ+安装角)
 * 角度定义 0° 指向车头、90° 指向右侧，安装角由 N10P_MOUNT_HEADING_DEG 给出 */
static scan_point_t point_for_angle(float angle, uint16_t distance)
{
    /* 角度转弧度：雷达原始角 + 安装角，再乘 π/180 */
    float radians = (angle + N10P_MOUNT_HEADING_DEG) * DEG_TO_RAD;
    /* 待返回的平面点（x 向右、y 向前） */
    scan_point_t point;
    /* x（向右）= 距离 × sin(角度) + 雷达右偏安装量 */
    point.x = (float)distance * sinf(radians) + N10P_MOUNT_OFFSET_RIGHT_MM;
    /* y（向前）= 距离 × cos(角度) + 雷达前偏安装量 */
    point.y = (float)distance * cosf(radians) + N10P_MOUNT_OFFSET_FORWARD_MM;
    /* 保留原始角度，供聚类时判断相邻点是否连续 */
    point.angle = angle;
    /* 返回换算后的点 */
    return point;
}

/* 判断相邻两点是否属于同一目标：角度间隔不超过 3° 且实际间距不超过 150 mm。
 * across_zero=true 时用于圈末与圈首的比较（跨 0° 的接缝），角度差需补 360°
 * 再计算，否则会被误判成间隔很大的两个目标 */
static bool points_connected(const scan_point_t *a, const scan_point_t *b, bool across_zero)
{
    /* 跨 0° 时把 b 的角度加 360°，避免 359°→1° 被算成 -358° */
    float gap = across_zero ? 360.0f + b->angle - a->angle : b->angle - a->angle;
    /* 两点在 x 方向的间距 */
    float dx = b->x - a->x;
    /* 两点在 y 方向的间距 */
    float dy = b->y - a->y;
    /* 角度差在 0~3° 且间距平方 ≤150² mm²，才认为是同一个目标上的点 */
    return gap >= 0.0f && gap <= MAX_POINT_GAP_DEG && dx * dx + dy * dy <= MAX_POINT_DISTANCE_SQ;
}

/* 以给定点作为首点，开始一个新分组 */
static void group_start(point_group_t *group, scan_point_t point)
{
    /* 首点和末点都设为该点 */
    group->first = group->last = point;
    /* x 坐标和从该点开始累加 */
    group->sum_x = point.x;
    /* y 坐标和从该点开始累加 */
    group->sum_y = point.y;
    /* 当前组已有 1 个采样点 */
    group->samples = 1U;
}

/* 把点追加到分组末尾，同时累加坐标和与采样数（供最后求中心位置） */
static void group_append(point_group_t *group, scan_point_t point)
{
    /* 更新组末点，下一次连续性判断用它和新点比较 */
    group->last = point;
    /* 累加 x 坐标，供最后求平均值 */
    group->sum_x += point.x;
    /* 累加 y 坐标，供最后求平均值 */
    group->sum_y += point.y;
    /* 采样点计数加一 */
    group->samples++;
}

/* 目标到车体中心距离的平方：排序/比较只用平方，避免开方开销 */
static uint32_t obstacle_distance_sq(const n10p_obstacle_t *obstacle)
{
    /* 距离平方 = x² + y²，只用于比较远近，不需要开方 */
    return (uint32_t)(obstacle->x_mm * obstacle->x_mm + obstacle->y_mm * obstacle->y_mm);
}

/* 一个分组结束时调用，把分组转成一个目标存入结果列表：
 * - 少于 3 个采样点的组按噪声丢弃
 * - 用组内坐标平均值作为目标位置（可见表面中心，不是物体几何中心）
 * - 按到车体中心的距离做插入排序（近的在前），只保留最近的 8 组 */
static void add_group(const point_group_t *group)
{
    /* 少于 3 个采样点的组当作噪点，直接丢弃 */
    if (group->samples < 3U) return;
    /* 准备写入结果列表的一个目标 */
    n10p_obstacle_t obstacle;
    /* x 取组内所有点的平均值，并四舍五入为整数毫米 */
    obstacle.x_mm = round_mm(group->sum_x / (float)group->samples);
    /* y 取组内所有点的平均值，并四舍五入为整数毫米 */
    obstacle.y_mm = round_mm(group->sum_y / (float)group->samples);
    /* 计算新目标到车体中心距离的平方，用于排序 */
    uint32_t distance_sq = obstacle_distance_sq(&obstacle);
    /* 新目标应插入的位置，从最前面开始找 */
    uint8_t position = 0U;
    /* 跳过所有不比新目标远的目标，找到第一个更远的槽位 */
    while (position < building_snapshot.count &&
           obstacle_distance_sq(&building_snapshot.obstacles[position]) <= distance_sq) position++; /* 当前目标不比新目标远，插入位置后移 */
    /* 已经排到第 8 名之外：列表只保留最近 8 个，直接丢弃 */
    if (position >= N10P_MAX_OBSTACLES) return;
    /* 列表还没满时，有效目标数加一 */
    if (building_snapshot.count < N10P_MAX_OBSTACLES) building_snapshot.count++;
    /* 从后往前把元素后移一位，给新目标腾出 position 槽位 */
    for (uint8_t i = building_snapshot.count - 1U; i > position; --i) {
        /* 前一个元素覆盖后一个元素的位置 */
        building_snapshot.obstacles[i] = building_snapshot.obstacles[i - 1U];
    }
    /* 把新目标放进腾好的位置，列表仍按由近到远排列 */
    building_snapshot.obstacles[position] = obstacle;
}

/* 逐点输入的分组状态机：与当前组末点相连则并入，否则结束当前组并另起一组。
 * 由于圈末与圈首可能在 0° 处属于同一目标，第一个分组先暂存不处理，
 * 等 finish_groups() 在圈末判断接缝能否合并 */
static void collect_point(scan_point_t point)
{
    /* 当前还没有任何分组：以这个点开第一组 */
    if (!current_exists) {
        /* 用该点初始化当前分组 */
        group_start(&current_group, point);
        /* 标记当前组有效 */
        current_exists = true;
    /* 当前组存在，且新点与组末点相连：并入当前组 */
    } else if (points_connected(&current_group.last, &point, false)) {
        /* 把新点追加到当前组末尾 */
        group_append(&current_group, point);
    /* 新点与当前组断开：结束当前组，并以新点另起一组 */
    } else {
        /* 第一个分组先暂存，等圈末检查它能否和最后一组合并 */
        if (!first_saved) {
            /* 暂存跨 0° 的首组 */
            first_group = current_group;
            /* 标记首组已暂存 */
            first_saved = true;
        } else { /* 首组之后的其他分组：立即结算 */
            /* 非首组：直接结算并放进结果列表 */
            add_group(&current_group);
        }
        /* 以新点开始下一组 */
        group_start(&current_group, point);
    }
}

/* 一圈数据结束时收尾分组，处理跨 0° 的接缝：
 * - 只有一个分组：直接输出
 * - 首组与末组在 0° 处相连：合并成一个目标再输出
 * - 不相连：两个目标分别输出 */
static void finish_groups(void)
{
    /* 只有存在未结算的分组时才需要收尾 */
    if (current_exists) {
        /* 整圈只有一个组：直接结算 */
        if (!first_saved) {
            /* 把当前组（也是唯一一组）结算进结果列表 */
            add_group(&current_group);
        /* 首组已暂存，且末组与首组在 0° 处相连：合并成一个目标 */
        } else if (points_connected(&current_group.last, &first_group.first, true)) {
            /* 合并首组的 x 坐标和 */
            current_group.sum_x += first_group.sum_x;
            /* 合并首组的 y 坐标和 */
            current_group.sum_y += first_group.sum_y;
            /* 合并首组的采样点数 */
            current_group.samples += first_group.samples;
            /* 合并后的整组一起结算 */
            add_group(&current_group);
        /* 首尾不相连：它们是两个独立目标，分别结算 */
        } else {
            /* 先结算暂存的首组 */
            add_group(&first_group);
            /* 再结算末组 */
            add_group(&current_group);
        }
    }
}

/* 发布一圈完整扫描：把本圈累积结果整体拷入 snapshot，
 * 前方距离按平方根换算回毫米（本圈没测到前方点时置 UINT32_MAX 表示未知），
 * 清空累积状态准备下一圈，扫描序号自增并记录发布时刻供超时判断 */
static void publish_scan(uint32_t now)
{
    /* 先把还没结算的分组处理完 */
    finish_groups();
    /* 扫描序号在旧快照基础上加一，保证每圈递增 */
    uint32_t next_seq = snapshot.scan_seq + 1U;
    /* 把本圈累积的障碍物列表整体拷入对外快照 */
    snapshot = building_snapshot;
    /* 写入本圈前方是否测到过有效点 */
    snapshot.front_known = building_front_known;
    /* 写入本圈前方 0.5 m 内是否被挡 */
    snapshot.blocked = building_blocked;
    /* 前方最近距离由平方开方还原成毫米；没测到前方点时置 UINT32_MAX 表示未知 */
    snapshot.front_mm = building_front_known ? (uint32_t)round_mm(sqrtf(building_front_sq)) : UINT32_MAX;
    /* 清空累积状态，准备接收下一圈 */
    clear_building_scan();
    /* 写入本圈序号 */
    snapshot.scan_seq = next_seq;
    /* 快照正式有效 */
    snapshot.valid = true;
    /* 记录至少完成过一圈，供 age_ms 判断 */
    has_completed_scan = true;
    /* 记录发布时间，供 500 ms 超时判断 */
    last_scan_tick = now;
}

/* ============================================================
 * 四、最核心：解析一帧数据（校验 → 算角度 → 分圈 → 判断前方）
 * ============================================================ */

/* 校验并解析一帧完整数据（frame_bytes/frame_length 已由 n10p_radar_process 填好），
 * 成功返回 true。依次完成：协议识别与校验 → 提取起止角 → 逐点插值角度并跨 0° 分圈
 * → 距离有效性过滤 → 坐标换算 + 前方 ±30° 安全判断 → 交给分组状态机。
 * 距离无效的点同样会推进角度，保证分圈基准不丢。 */
static bool parse_frame(uint32_t now)
{
    /* 帧头 0x54 = LDROBOT 格式，0xA5 = 旧 N10 格式 */
    bool ld06 = frame_bytes[0] == 0x54U;
    /* 0x54 0x1F 是 6 字节状态包，不是测距帧，单独处理 */
    if (ld06 && frame_bytes[1] == 0x1FU) {
        /* 实物每约 500 ms 夹带的 6 字节状态包（证据：tests/artifacts/hardware-rx-09.bin）。
         * 只接受已确认的全零状态内容，其他含义未知；
         * 状态包不计入测距帧，也不刷新点云时间戳（即不影响扫描有效期）。 */
        /* 三个状态字节必须全为 0，且 CRC8 与帧尾校验字节一致，才认为状态包有效 */
        return frame_bytes[2] == 0U && /* 状态字节 1 必须为 0 */
               frame_bytes[3] == 0U && /* 状态字节 2 必须为 0 */
               frame_bytes[4] == 0U && /* 状态字节 3 必须为 0 */
               ld06_crc8(frame_bytes, STL06_STATUS_FRAME_LEN - 1U) == /* 计算前 5 字节的 CRC8 */
               frame_bytes[STL06_STATUS_FRAME_LEN - 1U];               /* 与第 6 字节（校验字节）比较 */
    }
    /* LDROBOT 测距帧：用 CRC8 校验 */
    if (ld06) {
        /* 前 46 字节的 CRC8 必须等于最后一个字节，否则帧损坏 */
        if (ld06_crc8(frame_bytes, LD06_FRAME_LEN - 1U) != frame_bytes[LD06_FRAME_LEN - 1U]) return false;
    } else { /* 不是 LDROBOT，按旧 N10 协议校验 */
        /* 旧 N10：前 107 字节做累加和，不是 CRC8 */
        uint16_t sum = 0U;
        /* 逐字节累加 */
        for (uint16_t i = 0U; i < N10P_FRAME_LEN - 1U; ++i) sum += frame_bytes[i];
        /* 累加和的低 8 位必须等于最后一字节，否则帧损坏 */
        if ((uint8_t)sum != frame_bytes[N10P_FRAME_LEN - 1U]) return false;
    }
    /* 提取本帧的起始角与结束角（单位 0.01°） */
    /* 起始角：LDROBOT 在偏移 4（小端），旧 N10 在偏移 5（大端） */
    uint16_t start_raw = ld06 ? read_le_u16(&frame_bytes[4]) : read_be_u16(&frame_bytes[5]);
    /* 结束角：LDROBOT 在偏移 42（小端），旧 N10 在偏移 105（大端） */
    uint16_t end_raw = ld06 ? read_le_u16(&frame_bytes[42]) : read_be_u16(&frame_bytes[105]);
    /* 角度字段超过 36000（即 360.00°）说明数据异常，整帧丢弃 */
    if (start_raw > 36000U || end_raw > 36000U) return false;
    /* 把 0.01° 单位换算成度；36000 实际表示 0°，统一折算成 0 避免算出负跨度 */
    float start = start_raw == 36000U ? 0.0f : (float)start_raw * 0.01f;
    /* 结束角同样换算，36000 折算成 0 */
    float end = end_raw == 36000U ? 0.0f : (float)end_raw * 0.01f;
    /* 这一帧覆盖的角度跨度 */
    float span = end - start;
    /* 结束角小于起始角说明跨过了 360°，补一圈 */
    if (span < 0.0f) span += 360.0f;
    /* 单帧只覆盖一小段圆弧，跨度 ≥180° 说明端点错乱或数据损坏 */
    if (span >= 180.0f) return false;

    /* 每帧点数：LDROBOT 12 点，旧 N10 32 点 */
    uint8_t point_count = ld06 ? LD06_POINTS_PER_FRAME : N10P_POINTS_PER_FRAME;
    /* 插值分母：LDROBOT 用 11（首尾角都算点），旧 N10 用 32（结束角是边界） */
    float intervals = ld06 ? (float)(LD06_POINTS_PER_FRAME - 1U) : (float)N10P_POINTS_PER_FRAME;
    /* 逐点解析这一帧中的所有测距点 */
    for (uint8_t i = 0U; i < point_count; ++i) {
        /* 在起止角之间均匀插值出第 i 个点的角度，并归一化到 [0,360)。
         * LDROBOT 的结束角包含在点序列内，用 i/11；旧 N10 的结束角是边界，用 i/32。 */
        float angle = normalize_degrees(start + span * (float)i / intervals);
        /* 角度回绕检测：前一点接近 360°、当前点接近 0° 时视为跨 0°（一圈结束） */
        bool wrapped = previous_angle_valid && previous_angle - angle > 180.0f;
        /* 角度非回绕地倒退，或单步跳变 ≥180°，说明中间丢了数据：本圈结果立即作废 */
        if (previous_angle_valid && ((angle < previous_angle && !wrapped) || /* 非回绕地倒退 */
                                     angle - previous_angle >= 180.0f)) {    /* 或单步跳变 ≥180° */
            /* 当前圈作废，但保留缓冲中尚未解析的字节 */
            invalidate_scan(false);
        }
        /* 检测到跨 0°：说明刚转完一整圈 */
        if (wrapped) {
            /* 跨 0° 即一圈结束：先发布上一圈，再开始累积新一圈。
             * 同一帧内部也可能跨 0°，所以判断放在逐点循环里而不是帧尾。 */
            /* 已经建立过起点：发布上一圈完整结果 */
            if (scan_started) publish_scan(now);
            else { /* 还没建立扫描起点：这是上电后的第一圈 */
                /* 上电后第一次跨 0° 只用来确定圈的起点，不发布结果 */
                /* 清空可能收到的半圈残片 */
                clear_building_scan();
                /* 标记从下一帧起进入有效扫描圈 */
                scan_started = true;
            }
        }
        /* 更新角度基准，供下一个点判断连续性 */
        previous_angle = angle;
        /* 标记角度基准已可用 */
        previous_angle_valid = true;
        /* 该点距离字段在帧中的偏移：LDROBOT 从第 6 字节起，旧 N10 从第 7 字节起，每点 3 字节 */
        uint8_t offset = (uint8_t)((ld06 ? 6U : 7U) + i * 3U);
        /* 读距离：LDROBOT 为小端 16 位；旧 N10 为 15 位（高字节最高位是无效标志，须屏蔽） */
        uint16_t distance = ld06 ? read_le_u16(&frame_bytes[offset]) : /* LDROBOT：小端 16 位 */
                           (uint16_t)(((uint16_t)(frame_bytes[offset] & 0x7FU) << 8) | /* 旧 N10：高字节屏蔽最高位 */
                                      frame_bytes[offset + 1U]);                       /* 再拼上低字节 */
        /* 距离超范围的点不参与分组，但上面的角度推进已经生效，分圈不受影响 */
        if (!scan_started || distance < VALID_MIN_MM || distance > VALID_MAX_MM) continue;
        /* 分组与前方判断都使用换算后的原始点，不做量化取舍 */
        scan_point_t point = point_for_angle(angle, distance);
        /* 前方 ±30° 扇区判断：只统计车头方向(y>0)且 |x| ≤ y·tan30° 的点，
         * tan30°≈0.57735026919 */
        if (point.y > 0.0f && fabsf(point.x) <= point.y * 0.57735026919f) {
            /* 该点到车体中心距离的平方 */
            float distance_sq = point.x * point.x + point.y * point.y;
            /* 本圈前方至少测到过一个有效点 */
            building_front_known = true;
            /* 如果比之前记录的更近，就更新前方最近距离（用平方比较，避免开方） */
            if (distance_sq < building_front_sq) building_front_sq = distance_sq;
            /* 250000 mm² = 500 mm：前方 0.5 m 内有障碍物 → 置阻塞标志 */
            if (distance_sq < 250000.0f) building_blocked = true;
        }
        /* 把这个点送进聚类状态机 */
        collect_point(point);
    }
    /* 本帧校验和解析都成功：有效帧计数加一 */
    valid_frames++;
    /* 记录最近有效帧时刻，用于 500 ms 接收超时判断 */
    last_frame_tick = now;
    /* 标记正在接收数据，区别于"根本没接雷达" */
    receive_active = true;
    /* 告诉调用方这一帧有效 */
    return true;
}

/* 初始化：清空统计和快照，复位帧解析/分圈/分组状态。
 * 记录当前丢字节计数作为基准，之后才能检测出新增的溢出。
 * 上电后第一次跨 0° 只确定起点，因此需要两圈才能产生第一个有效结果。 */
void n10p_radar_init(void)
{
    /* 所有统计计数清零：有效帧、坏帧、最近帧时刻、最近发布时刻 */
    valid_frames = bad_frames = last_frame_tick = last_scan_tick = 0U;
    /* 还没完成过任何一圈完整扫描 */
    has_completed_scan = false;
    /* 对外快照整体清零 */
    memset(&snapshot, 0, sizeof(snapshot));
    /* 复位帧解析、分圈、分组和接收状态（保留缓冲内容） */
    invalidate_scan(false);
    /* 记录当前丢字节计数作为基准，之后新增的溢出才能被发现 */
    observed_dropped = uart1_rx_buf.dropped;
}

/* UART 接收出现错误（溢出/帧错/噪声等）后由主循环调用：
 * 数据流已不可信，丢弃缓冲区残留字节并让扫描重新同步 */
void n10p_radar_notify_rx_error(void)
{
    /* 数据流已不可信：作废当前圈，并丢弃缓冲区残留字节 */
    invalidate_scan(true);
    /* 同步丢字节基准，避免把这次错误重复判为新的溢出 */
    observed_dropped = uart1_rx_buf.dropped;
}

/* ============================================================
 * 五、组帧入口：把串口字节流拼成一帧一帧
 * ============================================================ */

/* 主循环任务：把 USART1 环形缓冲中的字节推进组帧状态机。
 * 一次调用会尽量处理完缓冲中所有字节，因此单次耗时可观，但不会阻塞等待新数据。 */
void n10p_radar_process(void)
{
    /* 先检查 500 ms 超时：雷达断连时及时让结果失效 */
    expire_scan(HAL_GetTick());
    /* 只要环形缓冲里还有字节就继续处理 */
    while (!ring_buf_empty(&uart1_rx_buf)) {
        /* 取当前累计丢字节数 */
        uint32_t dropped = uart1_rx_buf.dropped;
        /* 丢字节数变化 → 中断侧发生过溢出，帧边界已经丢失 */
        if (dropped != observed_dropped) {
            /* 记下新的丢字节数，避免重复处理 */
            observed_dropped = dropped;
            /* 丢弃缓冲内容并复位解析状态，重新找帧头 */
            invalidate_scan(true);
            /* 直接进入下一轮 while，重新检查缓冲 */
            continue;
        }
        /* 从缓冲取出 1 个字节 */
        uint8_t byte = ring_buf_get(&uart1_rx_buf);
        /* frame_index==0 表示还没开始一帧，正在等帧头 */
        if (frame_index == 0U) {
            /* 0xA5 是旧 N10 帧头，0x54 是 LDROBOT 帧头 */
            if (byte == 0xA5U || byte == 0x54U) {
                /* 按帧头类型确定整帧长度 */
                frame_length = byte == 0x54U ? LD06_FRAME_LEN : N10P_FRAME_LEN;
                /* 把帧头字节存入帧缓冲，索引后移 */
                frame_bytes[frame_index++] = byte;
            }
            /* 不是帧头：如果此时正在收数据，说明码流中混入了杂散字节 */
            else if (receive_active) {
                /* 坏帧计数加一 */
                bad_frames++;
                /* 当前圈作废，但不丢弃缓冲，继续找下一个帧头 */
                invalidate_scan(false);
            }
            /* 这一字节处理完，取下一字节 */
            continue;
        }
        /* 逐字节核对帧头：0x54 后应跟 0x2C(测距帧) 或 0x1F(状态包)；
         * 0xA5 后应跟 0x5A，再下一字节应为 0x6C */
        if ((frame_index == 1U && /* 正在核对帧头的第 2 个字节 */
             (frame_bytes[0] == 0x54U ? (byte != 0x2CU && byte != 0x1FU) : byte != 0x5AU)) || /* 第 2 字节不匹配 */
            (frame_index == 2U && frame_bytes[0] == 0xA5U && byte != 0x6CU)) { /* 旧 N10 第 3 字节不匹配 */
            /* 正在收数据时帧头错乱：坏帧计数加一 */
            if (receive_active) {
                /* 累计坏帧数 */
                bad_frames++;
                /* 当前圈结果作废（缓冲里的字节仍保留） */
                invalidate_scan(false);
            }
            /* 当前字节本身可能就是新帧头，据此重新开始组帧 */
            frame_index = (byte == 0xA5U || byte == 0x54U) ? 1U : 0U;
            /* 如果当前字节确实是新帧头 */
            if (frame_index != 0U) {
                /* 按新帧头类型设置帧长度 */
                frame_length = byte == 0x54U ? LD06_FRAME_LEN : N10P_FRAME_LEN;
                /* 把新帧头存为帧的第 0 字节 */
                frame_bytes[0] = byte;
            }
            /* 这一字节处理完，取下一字节 */
            continue;
        }
        /* 0x54 0x1F 是 6 字节状态包，长度与测距帧不同，在这里修正 */
        if (frame_index == 1U && frame_bytes[0] == 0x54U && byte == 0x1FU) {
            /* 状态包总长为 6 字节 */
            frame_length = STL06_STATUS_FRAME_LEN;
        }
        /* 帧头校验通过：把当前字节存入帧缓冲，索引后移 */
        frame_bytes[frame_index++] = byte;
        /* 判断是否已经收满一帧的预期长度 */
        if (frame_index == frame_length) {
            /* 一帧收齐：校验并解析；失败则作废本圈结果并重新同步 */
            if (!parse_frame(HAL_GetTick())) {
                /* 校验或解析失败：坏帧计数加一 */
                bad_frames++;
                /* 当前圈作废，但继续使用缓冲中的后续字节 */
                invalidate_scan(false);
            }
            /* 无论成功失败，都回到"等待下一个帧头"状态 */
            frame_index = 0U;
        }
    }
    /* 上面的解析耗时期间中断可能又丢了字节，退出前再确认一次 */
    if (uart1_rx_buf.dropped != observed_dropped) {
        /* 同步新的丢字节计数，避免重复报告 */
        observed_dropped = uart1_rx_buf.dropped;
        /* 数据流已经断过，当前圈作废并丢弃残留字节 */
        invalidate_scan(true);
    }
}

/* ============================================================
 * 六、对外接口：把处理结果交给上报模块
 * 注意：JSON 怎么拼、怎么发，不在这里，见 radar_telemetry.c
 * ============================================================ */

/* 取当前障碍物快照（含年龄）供上报模块使用：
 * 先做一次超时检查；未发布过完整扫描时 age_ms 用 UINT32_MAX 表示"未知" */
void n10p_get_obstacle_snapshot(n10p_obstacle_snapshot_t *out)
{
    /* 调用方传入空指针时直接返回，防止解引用崩溃 */
    if (out == NULL) return;
    /* 读取当前毫秒时基 */
    uint32_t now = HAL_GetTick();
    /* 先做超时检查，可能把快照标记为无效 */
    expire_scan(now);
    /* 计算快照年龄：完成过扫描用 now-发布时间，否则用 UINT32_MAX 表示未知 */
    snapshot.age_ms = has_completed_scan ? (uint32_t)(now - last_scan_tick) : UINT32_MAX;
    /* 把内部快照整份拷贝给调用方 */
    *out = snapshot;
}

/* 当前是否存在有效的完整扫描（最近一圈在 500 ms 内发布）；内部先做超时检查 */
bool n10p_radar_is_valid(void)
{
    /* 先检查是否超时（超时会清除 valid） */
    expire_scan(HAL_GetTick());
    /* 返回快照当前是否有效 */
    return snapshot.valid;
}

/* 兼容旧接口名，语义与 n10p_radar_is_valid() 相同 */
bool n10p_scan_is_ready(void) { return n10p_radar_is_valid(); } /* 直接转发到新接口 */
/* 累计校验通过的帧数 / 累计坏帧数，用于电脑端排查数据质量 */
uint32_t n10p_radar_valid_frames(void) { return valid_frames; } /* 返回有效帧累计数 */
uint32_t n10p_radar_bad_frames(void) { return bad_frames; }     /* 返回坏帧累计数 */
