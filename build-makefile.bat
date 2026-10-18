@echo off
chcp 65001 >nul
rem ============================================================
rem  BuJinDianJi2 一键编译脚本
rem  用法:
rem    build.bat          编译（增量）
rem    build.bat clean    清理编译产物
rem  输出: Debug\BuJinDianJi2.elf / .bin / .hex
rem ============================================================
setlocal
set "ROOT=%~dp0"

rem ---- 查找 make：优先 PATH，其次 Dev-Cpp 自带 ----
set "MAKE="
where make >nul 2>nul
if not errorlevel 1 set "MAKE=make"
if not defined MAKE (
  where mingw32-make >nul 2>nul
  if not errorlevel 1 set "MAKE=mingw32-make"
)
if not defined MAKE (
  if exist "D:\Dev-Cpp\TDM-GCC-64\bin\mingw32-make.exe" set "MAKE=D:\Dev-Cpp\TDM-GCC-64\bin\mingw32-make.exe"
)
if not defined MAKE (
  echo [错误] 找不到 make 或 mingw32-make。
  echo        请安装 make，或修改本脚本中 Dev-Cpp 的路径。
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

cd /d "%ROOT%Debug"

if /i "%~1"=="clean" goto :clean

echo [编译] 使用 %MAKE%
"%MAKE%" -j4 all
if errorlevel 1 (
  echo.
  echo [失败] 编译出错，请查看上面的信息。
  exit /b 1
)
echo.
echo [完成] 输出文件:
echo        Debug\BuJinDianJi2.elf
echo        Debug\BuJinDianJi2.bin
echo        Debug\BuJinDianJi2.hex
exit /b 0

:clean
echo [清理] 删除编译产物...
del /S /Q "%ROOT%Debug\Core\*.o"   2>nul
del /S /Q "%ROOT%Debug\Core\*.d"   2>nul
del /S /Q "%ROOT%Debug\Core\*.su"  2>nul
del /S /Q "%ROOT%Debug\Core\*.cyclo" 2>nul
del /S /Q "%ROOT%Debug\Drivers\*.o"   2>nul
del /S /Q "%ROOT%Debug\Drivers\*.d"   2>nul
del /S /Q "%ROOT%Debug\Drivers\*.su"  2>nul
del /S /Q "%ROOT%Debug\Drivers\*.cyclo" 2>nul
del /Q "%ROOT%Debug\BuJinDianJi2.elf" 2>nul
del /Q "%ROOT%Debug\BuJinDianJi2.bin" 2>nul
del /Q "%ROOT%Debug\BuJinDianJi2.hex" 2>nul
del /Q "%ROOT%Debug\BuJinDianJi2.map" 2>nul
del /Q "%ROOT%Debug\BuJinDianJi2.list" 2>nul
del /Q "%ROOT%Debug\default.size.stdout" 2>nul
echo [完成] 已清理。
exit /b 0
