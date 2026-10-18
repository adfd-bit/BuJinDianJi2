# 雷达接口与移植

## 原始雷达输入

STL-06NBJ 接 USART1_RX / PA10，`230400 8N1`。实物捕获确认使用 LDROBOT 格式：帧头 `54 2C`、47 字节、12 点，小端字段，前 46 字节按 CRC8 多项式 `0x4D`、初值 0、不反射计算，与最后一字节比较。字段依次为帧头、版本/长度、转速、起始角、12 个距离/强度点、结束角、时间戳、CRC。转速单位为度/秒，角度单位为 0.01°，点角度包含结束端点，使用 `i/11` 插值。

固件自动识别实物格式，并兼容旧 `A5 5A 6C` 帧头的 108 字节、32 点格式。旧格式保留原有 `i/32` 插值和前 107 字节累加低八位校验；旧格式的校验不是 CRC8。

实物还每隔约 500 ms 夹带 `54 1F 00 00 00 41` 六字节状态包，证据为 `tests/artifacts/hardware-rx-09.bin`。最后一字节同样满足 CRC8。解析器仅接受这一已确认的零状态内容，不把它计入测距帧，也不刷新扫描有效期；未知状态内容、未知帧头和校验错误仍使扫描失效并重新同步。该状态字段的完整厂商语义尚未确认。

接收中断只写入 2048 字节环形缓冲并重新启动单字节接收。主循环完成校验、逐点回绕分圈、分组和坐标换算；距离无效的点也参与扫描角度推进。坏帧、UART 错误和丢字节使扫描重新同步。

## 固件调用

外设初始化必须先完成 GPIO、DMA、USART1 和 USART2，然后执行：

```c
ring_buf_init(&uart1_rx_buf);
n10p_radar_init();
radar_telemetry_init();
radar_uart_start();

while (1) {
    radar_uart_poll();
    n10p_radar_process();
    radar_telemetry_process();
}
```

`n10p_get_obstacle_snapshot(n10p_obstacle_snapshot_t *out)` 获取固定容量结果，包括 `valid`、`scan_seq`、`age_ms`、`front_known`、`front_mm`、`blocked`、`count` 和八个 `x_mm/y_mm` 目标槽。内部毫米坐标为有符号整数。未产生过完整扫描时 `age_ms=UINT32_MAX`；无前方测距时 `front_known=false`，调用方不得使用前方距离判定畅通。

`n10p_radar_notify_rx_error()` 由主循环在接收错误后调用；`n10p_radar_valid_frames()` 与 `n10p_radar_bad_frames()` 提供输入统计。完整扫描年龄达到 500 ms 时结果失效，过期转换只重置一次，以允许重新积累完整圈。

安装参数位于 `n10_radar.h`：右偏 106 mm、前偏 106 mm、安装角 0°。当前角度定义为 0° 向前、90° 向右；位置换算为 `x=106+r*sin(theta)`、`y=106+r*cos(theta)`，这里 r、x、y 单位为毫米。

## 电脑输出协议 v1

USART2_TX / PA2 经 DMA1 Channel7 普通模式输出，`230400 8N1`。USART1 仅接收雷达，USART2 仅发送电脑报告。每 100 ms 一条 ASCII JSON，以 LF 结束；不混入原始二进制或调试文本。每条消息是一份完整快照。

```json
{"v":1,"seq":12,"scan_seq":7,"valid":true,"age_ms":20,"front_m":0.420,"blocked":true,"obstacles":[{"x_m":0.106,"y_m":0.406}],"rx_frames":140,"bad_frames":0,"rx_dropped":0,"uart_errors":0}
```

| 字段 | 定义 |
| --- | --- |
| `v` | 协议版本，固定整数 1 |
| `seq` | 输出报告序号，无符号 32 位，复位后重置 |
| `scan_seq` | 已完成扫描序号，无符号 32 位 |
| `valid` | 当前完整扫描是否有效 |
| `age_ms` | 最近完整扫描年龄；从未完成扫描时为 `null` |
| `front_m` | 前方 ±30° 的最近有效点到小车中心距离，米；未知为 `null` |
| `blocked` | 前方距离小于 0.5 m；未知为 `null` |
| `obstacles` | 最多八项，每项为小车坐标 `x_m/y_m`，米，保留三位小数 |
| `rx_frames` | 校验通过的输入帧数 |
| `bad_frames` | 输入坏帧数 |
| `rx_dropped` | 环形缓冲丢弃的字节数 |
| `uart_errors` | UART 错误或接收启动失败次数 |

扫描失效时 `obstacles=[]`、`front_m=null`、`blocked=null`。新完整扫描可以有效但没有满足分组条件的目标；没有前方有效测距不等同于道路畅通。计数使用无符号 32 位，允许自然回绕。

发送使用两个固定 1024 字节缓冲，分别维护 FREE、READY、IN_FLIGHT 状态。DMA 完成后才释放正在发送的缓冲；发送拥塞时替换待发旧快照。格式化检查长度，毫米转米不依赖浮点 printf。

## 移植与验证

同步 `.ioc` 的 USART1/USART2 波特率、USART2 TX DMA 与 DMA/USART 中断。不要同时开启 RX DMA 和直接读取 DR 的 RXNE 路径。必须启用 DMA1 Channel7、USART1 和 USART2 IRQ，UART 发送完成由 USART2 TC 中断收尾。

识别规则和硬件接线见根目录 README。移植后先检查完整扫描、采样分组、错误恢复、JSON 与 DMA 缓冲测试，再验证真实串口。实际安装角、雷达角度方向和目标距离需用实物检查，不能仅依靠构建通过。

LDROBOT CRC 与协议参考其 [官方 SDK](https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk)，许可见 [第三方声明](THIRD_PARTY_NOTICES.md)。`tests/artifacts/hardware-rx.bin` 是开发板接收缓冲的实物采样，环形存储顺序不能直接当成串口时间顺序；回放测试按有效帧时间戳恢复顺序。
