@echo off
chcp 65001 >nul
title Khởi Động Trạm Điều Khiển Xe Cân Bằng STM32F405 & Video Vision

echo =====================================================================
echo   KHỞI ĐỘNG GROUND CONTROL STATION & VISION (STM32F405)
echo =====================================================================
echo Đang kiểm tra môi trường Python...

python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [LỖI] Không tìm thấy Python trên máy! Vui lòng cài đặt Python 3.8+ và tích hợp vào PATH.
    pause
    exit /b 1
)

echo Đang khởi chạy Ground Station...
cd /d "%~dp0"
python app.py

if %errorlevel% neq 0 (
    echo.
    echo [THÔNG BÁO] Ứng dụng đã thoát với mã lỗi: %errorlevel%
    pause
)
