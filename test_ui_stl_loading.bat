@echo off
echo ========================================
echo STL Loading Test - Manual Instructions
echo ========================================
echo.
echo Log file location:
echo   src\ui\RobotController.UI\bin\Debug\net8.0-windows\logs\ui.log
echo.
echo Steps to test:
echo   1. Make sure Core is running (build\bin\Debug\Core.exe)
echo   2. Run UI: src\ui\RobotController.UI\bin\Debug\net8.0-windows\RobotController.exe
echo   3. Navigate to Robot Package Browser
echo   4. Select "KUKA KR 10 R1420"
echo   5. Click "Load Package"
echo   6. Check viewport (should show detailed KUKA model, not placeholder)
echo   7. Close UI
echo   8. Read log file below
echo.
echo ========================================
pause
echo.
echo Opening log file...
type "src\ui\RobotController.UI\bin\Debug\net8.0-windows\logs\ui.log"
echo.
echo ========================================
pause
