# ============================================================
#  BuJinDianJi2 —— STM32F103C8T6 雷达障碍物监视固件
#
#  构建（在项目根目录）：
#    cmake --preset debug      # 配置
#    cmake --build --preset debug
#  或：
#    cmake --workflow --preset debug
#
#  产物在 build/debug/：
#    BuJinDianJi2.elf / .bin / .hex / .map
#    compile_commands.json（给 VS Code 代码跳转/补全用）
# ============================================================
cmake_minimum_required(VERSION 3.22)

# 工具链文件必须在 project() 之前生效；presets 里也会传入同名缓存变量
if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
      CACHE FILEPATH "Cross toolchain file")
endif()

project(BuJinDianJi2 LANGUAGES C ASM)

# 与原 CubeIDE 工程保持一致：-std=gnu11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type (Debug/Release)" FORCE)
endif()

# ---------------- 目标 MCU ----------------
set(TARGET_MCU STM32F103xB)

# ---------------- 应用源文件 ----------------
set(APP_SOURCES
    Core/Src/dma.c
    Core/Src/gpio.c
    Core/Src/main.c
    Core/Src/n10_radar.c
    Core/Src/radar_telemetry.c
    Core/Src/stm32f1xx_hal_msp.c
    Core/Src/stm32f1xx_it.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/system_stm32f1xx.c
    Core/Src/usart.c
)

# ---------------- HAL 驱动源文件 ----------------
# 只编译本工程实际用到的模块，与原 Debug/*/subdir.mk 保持一致
set(HAL_SOURCES
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
)

# 启动文件（GNU 汇编，不需要 C 预处理器）
set(STARTUP_SOURCE Core/Startup/startup_stm32f103c8tx.s)

add_executable(${PROJECT_NAME}
    ${APP_SOURCES}
    ${HAL_SOURCES}
    ${STARTUP_SOURCE}
)

# ---------------- 头文件路径 ----------------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
)

# ---------------- 宏定义 ----------------
target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    ${TARGET_MCU}
)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG)
endif()

# ---------------- 编译选项 ----------------
target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -mfloat-abi=soft
    -ffunction-sections
    -fdata-sections
    -Wall
)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(${PROJECT_NAME} PRIVATE -g3 -O0)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -O2)
endif()

# ---------------- 链接选项 ----------------
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8TX_FLASH.ld")
if(NOT EXISTS "${LINKER_SCRIPT}")
  message(FATAL_ERROR "找不到链接脚本: ${LINKER_SCRIPT}")
endif()

target_link_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -mfloat-abi=soft
    -T${LINKER_SCRIPT}
    -specs=nosys.specs
    -specs=nano.specs
    -static
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)

# 输出 .elf 后缀，便于和 CubeIDE 的产物对应
set_target_properties(${PROJECT_NAME} PROPERTIES
    SUFFIX ".elf"
    LINK_DEPENDS "${LINKER_SCRIPT}"
)

# ---------------- 后处理：生成 bin/hex 并打印占用 ----------------
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "生成 ${PROJECT_NAME}.bin / ${PROJECT_NAME}.hex"
)

# ---------------- 复制 compile_commands.json 到项目根目录 ----------------
# VS Code 的 C/C++ 扩展默认从这里读取，实现 Ctrl+click 跳转
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
              ${CMAKE_SOURCE_DIR}/compile_commands.json
      COMMENT "更新 compile_commands.json"
  )
endif()
