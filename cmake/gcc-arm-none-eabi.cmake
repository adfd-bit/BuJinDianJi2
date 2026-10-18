# ============================================================
#  ARM GNU Toolchain 交叉编译配置（arm-none-eabi-gcc）
#
#  默认工具链路径可在 cmake/ 下修改，或在配置时用
#    cmake --preset debug -DTOOLCHAIN_PATH=<工具链根目录>
#  覆盖。
# ============================================================
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 工具链根目录（包含 bin/arm-none-eabi-gcc.exe）
if(NOT DEFINED TOOLCHAIN_PATH)
  set(TOOLCHAIN_PATH "D:/arm-gnu-toolchain-13.2.rel1-mingw-w64-i686-arm-none-eabi"
      CACHE PATH "ARM GNU toolchain root directory")
endif()

# 优先使用指定目录下的编译器，找不到再退回系统 PATH
find_program(ARM_GCC arm-none-eabi-gcc
  PATHS "${TOOLCHAIN_PATH}/bin"
  NO_DEFAULT_PATH)
if(NOT ARM_GCC)
  find_program(ARM_GCC arm-none-eabi-gcc)
endif()
if(NOT ARM_GCC)
  message(FATAL_ERROR
    "未找到 arm-none-eabi-gcc。请用 -DTOOLCHAIN_PATH=<工具链根目录> 指定安装位置。")
endif()

get_filename_component(ARM_TOOLCHAIN_BIN "${ARM_GCC}" DIRECTORY)

if(CMAKE_HOST_WIN32)
  set(_TOOL_SUFFIX ".exe")
else()
  set(_TOOL_SUFFIX "")
endif()

set(CMAKE_C_COMPILER   "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-gcc${_TOOL_SUFFIX}")
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-gcc${_TOOL_SUFFIX}")
set(CMAKE_OBJCOPY "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-objcopy${_TOOL_SUFFIX}" CACHE FILEPATH "")
set(CMAKE_SIZE    "${ARM_TOOLCHAIN_BIN}/arm-none-eabi-size${_TOOL_SUFFIX}"    CACHE FILEPATH "")

# 交叉编译环境下，CMake 无法运行目标程序，用静态库试编译代替
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 程序在主机上查找，头文件和库只在工具链内查找
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
