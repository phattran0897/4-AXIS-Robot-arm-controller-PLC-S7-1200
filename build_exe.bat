@echo off
setlocal
cd /d "%~dp0"
echo [1/3] Preparing build icon...
python tools\prepare_build.py --icon
if errorlevel 1 goto :failed
echo [2/3] Building executable...
python -m PyInstaller --noconfirm RobotController.spec
if errorlevel 1 goto :failed
echo [3/3] Copying config, configured model, calibration and assets...
python tools\prepare_build.py --destination dist\RobotController
if errorlevel 1 goto :failed
echo Build complete: dist\RobotController\RobotController.exe
exit /b 0
:failed
echo Build FAILED. Resolve the error above before distributing the application.
exit /b 1
