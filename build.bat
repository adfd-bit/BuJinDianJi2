@echo off
chcp 65001 >nul
rem ============================================================
rem  BuJinDianJi2 一键编译脚本（CMake）
rem  用法:
rem    build.bat          编译（Debug）
rem    build.bat release  编译（Release）
rem    build.bat clean    清理 build 目录
rem  输出: build\debug\BuJinDianJi2.elf / .bin / .hex
rem  旧的 Makefile 构建方式保留在 build-makefile.bat
rem ============================================================
setlocal
set "ROOT=%~dp0"

rem ---- 查找 cmake：优先 PATH，其次 pip 安装位置 ----
set "CMAKE="
where cmake >nul 2>nul
if not errorlevel 1 set "CMAKE=cmake"
if not defined CMAKE (
  if exist "%LOCALAPPDATA%\Programs\Python\Python312\Scripts\cmake.exe" set "CMAKE=%LOCALAPPDATA%\Programs\Python\Python312\Scripts\cmake.exe"
)
if not defined CMAKE (
  echo [错误] 找不到 cmake。请先安装 CMake 并加入 PATH。
  exit /b 1
)

rem ---- 确保 Ninja 可用（CMakePresets 里指定了 Ninja 生成器）----
where ninja >nul 2>nul
if errorlevel 1 (
  if exist "%LOCALAPPDATA%\Programs\Python\Python312\Scripts\ninja.exe" set "PATH=%LOCALAPPDATA%\Programs\Python\Python312\Scripts;%PATH%"
)
where ninja >nul 2>nul
if errorlevel 1 (
  echo [错误] 找不到 ninja。请先安装 Ninja 并加入 PATH。
  exit /b 1
)

rem ---- 确保 ARM 工具链在 PATH 中 ----
where arm-none-eabi-gcc >nul 2>nul
if errorlevel 1 set "PATH=D:\arm-gnu-toolchain-13.2.rel1-mingw-w64-i686-arm-none-eabi\bin;%PATH%"
where arm-none-eabi-gcc >nul 2>nul
if errorlevel 1 (
  echo [错误] 找不到 arm-none-eabi-gcc。
  echo        请检查工具链是否安装在 D:\arm-gnu-toolchain-13.2.rel1-mingw-w64-i686-arm-none-eabi
  exit /b 1
)

cd /d "%ROOT%"

if /i "%~1"=="clean" goto :clean

set "PRESET=debug"
if /i "%~1"=="release" set "PRESET=release"

echo [配置+编译] CMake preset: %PRESET%
"%CMAKE%" --workflow --preset %PRESET%
if errorlevel 1 (
  echo.
  echo [失败] 编译出错，请查看上面的信息。
  exit /b 1
)
echo.
echo [完成] 输出文件:
echo        build\%PRESET%\BuJinDianJi2.elf
echo        build\%PRESET%\BuJinDianJi2.bin
echo        build\%PRESET%\BuJinDianJi2.hex
exit /b 0

:clean
echo [清理] 删除 build 目录...
if exist "%ROOT%build" rmdir /S /Q "%ROOT%build"
echo [完成] 已清理。
exit /b 0
