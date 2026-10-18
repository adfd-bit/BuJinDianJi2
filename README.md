# BuJinDianJi2 - STM32 雷达障碍物监视工程

真实 STL-06NBJ 雷达数据由 STM32F103C8T6 接收并识别，开发板通过串口输出障碍物位置，电脑界面显示障碍物图标。工程仅保留雷达应用及必要的 STM32 基础支持。

本次实物采集确认使用 LDROBOT 兼容协议：`54 2C` 帧头、47 字节、12 点、CRC8 校验。固件自动识别该格式，同时保留旧 `A5 5A 6C` /108 字节输入兼容。

## 接线

| 信号 | 连接 |
| --- | --- |
| 雷达 TX | PA10 / USART1_RX |
| 雷达 GND | 开发板 GND |
| 雷达电源 | 按雷达规格独立稳定供电 |
| 3.3V TTL USB-UART RX | PA2 / USART2_TX |
| USB-UART GND | 开发板 GND |
| USB-UART TX | 不连接 |

雷达、开发板和串口适配器共地。USB-UART 的 3.3V 指信号电平，不表示可以用适配器的 3.3V 引脚给雷达供电。ST-LINK 用于 SWD 下载调试；电脑可视化需要 USB-UART 对应的 COM 口。

## 编译、烧录与运行

1. 在项目根目录运行 `build.bat` 编译（双击或在终端执行均可），输出在 `build\debug`；`build.bat clean` 清理产物，`build.bat release` 编译优化版本。VS Code 中按 `Ctrl+Shift+B` 执行同一个脚本。构建系统为 CMake + Ninja，预设见 `CMakePresets.json`；旧的 Makefile 方式保留在 `build-makefile.bat`。
2. 烧录可用 ST-Link（如 STM32CubeProgrammer 烧写 `build\debug\BuJinDianJi2.hex`，或用 cortex-debug 自行配置）。本副本未附带 `.vscode` 调试配置。
3. 双击 `run_visualizer.bat`，或在项目根目录执行以下命令，在界面选择 USB-UART 的 COM 口并连接。

```powershell
py -m pip install -r tools/requirements.txt
py tools/radar_visualizer.py
```

当前电脑已识别适配器为 `COM5`。可运行 `run_visualizer.bat --port COM5` 自动连接；重新插拔后以实际 COM 编号为准。

构建输出位于 `build\debug`，包含 ELF、BIN、HEX 和 map 文件。开发板主频 72 MHz，Flash 64 KB，RAM 20 KB；实际占用以构建的 map/size 为准。

## 代码跳转（Ctrl+click）

工程已改为 CMake，配置时自动生成 `compile_commands.json`，VS Code 的 C/C++ 扩展据此提供跳转、补全和错误提示。

1. 首次打开工程时，VS Code 会提示选择 CMake 预设，选择 **debug**（或按 `Ctrl+Shift+P` → `CMake: Select Configure Preset`）。
2. 等待状态栏 CMake 配置完成（显示 Ready）。之后按住 `Ctrl` 点击函数名、变量名即可跳转。
3. `.vscode/settings.json` 已配置好 `compile_commands.json` 路径，并禁用了 STM32 的 clangd 扩展——它默认启用但本机没有 clangd 可执行文件，会和 cpptools 抢占 C 文件导致跳转失效。
4. 若仍不能跳转：`Ctrl+Shift+P` → `C/C++: Reset IntelliSense Database`，然后 `Developer: Reload Window`。

## 工具链说明

本机工具链为 `D:\arm-gnu-toolchain-13.2.rel1-mingw-w64-i686-arm-none-eabi`（GCC 13.2），CMake 与 Ninja 由 pip 安装（`python -m pip install cmake ninja`），`build.bat` 会自动查找。工具链路径可在 `cmake/gcc-arm-none-eabi.cmake` 中修改，或配置时传入 `-DTOOLCHAIN_PATH=<工具链根目录>`。

旧的 Makefile 构建（`build-makefile.bat`）仍可使用：`Debug` 下的 makefile 由 CubeIDE 14.3 生成，已做两处适配——去掉仅 GCC 14 支持的 `-fcyclomatic-complexity`，并把链接脚本由 `D:\STM32_workstate\...` 改为相对路径 `../STM32F103C8TX_FLASH.ld`。若用 CubeIDE 重新生成 makefile，这两处需要重新处理，否则会报 `unrecognized command-line option` 或找不到链接脚本。

## 坐标与识别规则

- 输出以小车中心为原点，X 向右、Y 向前，单位米。
- 雷达安装位置为右 106 mm、前 106 mm，0° 指向车头；偏移由固件补偿，电脑不再重复补偿。
- 只使用完整扫描。上电第一次跨 0° 建立起点，第二次跨 0° 才发布第一圈；500 ms 没有新完整扫描则失效。
- 有效距离为 50–6000 mm；角度间隔超过 3° 或相邻有效点间距超过 150 mm 时分组。跨 0° 的首尾组先合并，再保留至少三个原始有效采样点的组。
- 每组位置为可见表面点群中心，最多输出最近八组，不代表物体几何中心，也不提供跨扫描目标 ID。
- 前方 ±30° 使用所有有效测距点计算，距小车中心小于 0.5 m 时 `blocked=true`。没有前方有效测距时输出未知，不能据此判定道路畅通。
- 本工程输出检测结果，不包含电机执行或自动绕行。

界面提供场地图和车体坐标障碍物图。场地图的车辆位置与朝向是显示基准，工程没有实时定位来源；位置数值始终是小车相对坐标。

## 数据接口与检查

PA2 / USART2 每 100 ms 输出一行 JSON，波特率为 230400；接口定义见 [雷达接口与移植](docs/RADAR_PORTING_GUIDE.md)。启动后即使雷达未连接，也会输出 `valid:false` 状态，便于区分“串口未通”和“雷达无有效扫描”。

```powershell
py -m unittest discover -s tests -p "test_*.py"
py -m py_compile tools/radar_visualizer.py tools/obstacle_protocol.py
```

固件算法和 DMA 缓冲测试位于 `tests`，使用本机 C 编译器配合其中的 HAL stub 执行。离线测试不代替实际雷达、COM 和方向距离验证。

## 无数据时检查

- COM 列表为空：检查 USB-UART USB 连接与驱动；仅接 ST-LINK 无法为界面提供位置数据。
- 串口有 JSON 但 `valid:false`：检查雷达供电、PA10 接线与 230400 波特率，查看接收与错误计数。
- 界面没有图标但 `valid:true`：可能没有满足分组条件的目标；查看障碍物数量与前方距离。
- 暂停调试或拔掉接线超过 500 ms：界面应清除图标，恢复后等待新完整扫描。

更多说明见 [电脑工具](tools/README.md) 和 [本次工程调整](docs/RADAR_MIGRATION_SUMMARY.md)。
