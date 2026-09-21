# 4-Axis Industrial Robot Control System

A complete software suite for controlling a 4-DOF articulated industrial robot arm using a Siemens S7-1200 PLC, integrated with AI Vision (YOLO) for real-time object detection and automatic sorting. Built with a modern, responsive GUI using PySide6.

## Features

- 🖥️ **Modern GUI (PySide6)**: Beautiful, responsive interface with Dark Mode, Auto/Manual controls, and real-time telemetry.
- 🧠 **AI Vision (YOLO)**: Real-time object detection and classification using Ultralytics YOLO to distinguish between good (TOT) and bad (XAU) objects.
- ⚙️ **PLC Integration**: Direct communication with Siemens S7-1200 PLCs.
- 🦾 **Kinematics Engine**: Built-in Forward and Inverse Kinematics using DH parameters for precise 4-axis motion planning.
- 📦 **Automated Sorting**: Fully automated cycle to pick detected objects and place them into designated zones based on AI classification.
- 🧪 **Mock/Test Mode**: Included `test_gui_no_plc.py` allows you to test the AI vision and GUI without needing physical PLC hardware.

## Prerequisites

- Python 3.10+
- Siemens S7-1200 PLC (or use the test script to bypass)
- USB Camera (for AI Vision)
- YOLO `.pt` model file

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/phattran0897/4-AXIS-Robot-arm-controller-PLC-S7-1200.git
   cd 4-AXIS-Robot-arm-controller-PLC-S7-1200
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Configure your settings:
   - Edit `config.yaml` to set your PLC IP address, camera index, YOLO model path, and sorting coordinates.

## Usage

**Production Mode (Requires PLC connection):**
```bash
python main.py
```

**Test Mode (No PLC required, uses camera for AI Vision):**
```bash
python test_gui_no_plc.py
```

## Configuration (`config.yaml`)

The system is highly configurable without touching the source code:
- **`plc`**: Set IP, Rack, Slot, and DB byte offsets.
- **`yolo`**: Set model path, confidence threshold (`thresh`), and `px2mm` conversion factor.
- **`kinematics`**: Adjust DH parameters (`a1`, `a2`, `d1`, etc.) and joint limits to match your specific hardware.
- **`sort_positions`**: Configure the precise XYZ coordinates for picking up and placing objects.

## System Architecture

- `main.py`: Application entry point, coordinates threads.
- `src/ui/`: PySide6 GUI components (Auto page, Manual page, Settings page).
- `src/plc/`: PLC communication layer (`plc_controller.py`).
- `src/ai/`: YOLO detection pipeline (`yolo_detector.py`) with real-time coordinate hysteresis stabilization.
- `src/robot/`: Logic for executing the sorting sequences (`sorting_controller.py`).

## License
MIT License
