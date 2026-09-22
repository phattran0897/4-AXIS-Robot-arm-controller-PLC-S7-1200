@echo off
echo ==========================================
echo Building 4-Axis Robot System Executable...
echo ==========================================

echo [1/3] Installing PyInstaller...
pip install pyinstaller

echo [2/3] Running PyInstaller...
:: --onedir: Creates a folder containing the exe and dependencies (better for large apps like YOLO/PySide6)
:: --noconfirm: Overwrite existing build
:: --collect-all ultralytics: Include all YOLO dependencies
:: --collect-all snap7: Include snap7 DLLs
set KMP_DUPLICATE_LIB_OK=TRUE
pyinstaller --noconfirm ^
    --onedir ^
    --windowed ^
    --name "RobotController" ^
    --icon "assets\vaa_logo.png" ^
    --collect-all ultralytics ^
    --collect-all snap7 ^
    --exclude-module PyQt5 ^
    --exclude-module PyQt6 ^
    --exclude-module PySide2 ^
    --exclude-module torchaudio ^
    --exclude-module tensorboard ^
    --exclude-module matplotlib ^
    --exclude-module IPython ^
    --exclude-module pytest ^
    main.py

echo [3/3] Copying configuration, models, and assets...
:: Create models directory in dist
if not exist "dist\RobotController\models" mkdir "dist\RobotController\models"
:: Copy config.yaml to the same directory as the executable so the user can edit it
copy config.yaml dist\RobotController\
:: Copy assets folder
xcopy /E /I /Y "assets" "dist\RobotController\assets"
:: NOTE: You will need to manually copy your .pt model file into the dist\RobotController\models folder!

echo ==========================================
echo Build complete! 
echo Your application is located in the "dist\RobotController" folder.
echo Remember to copy your YOLO .pt model into the folder before running.
echo ==========================================
pause
