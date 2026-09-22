@echo off
REM fetch_monitor.bat —— 队友端(Windows)拉图循环: 临时PNG + 原子替换 + 失败保留旧图
REM 2026-09-19 新增。用法: fetch_monitor.bat [车机IP] [用户名]
REM   默认 IP=192.168.5.100, 用户=orange-pi
REM 原理: scp 先写到 monitor.png.tmp, 成功才 move 成 monitor.png(同卷 rename 原子),
REM        浏览器读到的 monitor.png 要么是旧完整图要么是新完整图, 不会读到下载中的半张。
REM        失败(errorlevel 非0)则保留旧 monitor.png 并打印失败/时间, 循环串行执行不堆积进程。
setlocal
set "CARIP=%~1"
set "CARUSER=%~2"
if "%CARIP%"=="" set "CARIP=192.168.5.100"
if "%CARUSER%"=="" set "CARUSER=orange-pi"

echo 拉图循环已启动: %CARUSER%@%CARIP%:~/racecar_team/logs/slam/monitor.png
echo 每 2 秒一轮; Ctrl+C 退出。浏览器打开同目录 monitor.html 即可看实时图。

:loop
scp "%CARUSER%@%CARIP%:~/racecar_team/logs/slam/monitor.png" "monitor.png.tmp" >nul 2>nul
if errorlevel 1 (
    echo [%date% %time%] 拉图失败, 保留旧 monitor.png
    if exist "monitor.png.tmp" del "monitor.png.tmp" >nul 2>nul
) else (
    move /y "monitor.png.tmp" "monitor.png" >nul 2>nul
    echo [%date% %time%] 更新成功
)
timeout /t 2 /nobreak >nul
goto loop
