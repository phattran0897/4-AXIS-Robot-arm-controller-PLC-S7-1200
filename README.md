# 4-Axis SCARA Robot Control System

A production-grade Python application for automated defect detection and
robotic pick-and-sort using **YOLOv11**, a **Siemens S7-1200 PLC**, and a
**CustomTkinter** GUI.

```
Camera → YOLODetector (YOLO inference)
                ↓ defect coords (mm)
        Inverse Kinematics
                ↓ joint angles
        PLCController (snap7) → S7-1200 → SCARA Robot
```

---

## Table of Contents

1. [Architecture](#architecture)
2. [Prerequisites](#prerequisites)
3. [Installation](#installation)
4. [Configuration](#configuration)
5. [Running the Application](#running-the-application)
6. [Project Structure](#project-structure)
7. [Running Tests](#running-tests)
8. [CI Pipeline](#ci-pipeline)
9. [Key Design Decisions](#key-design-decisions)

---

## Architecture

```
.
├── config.yaml                  # Single source-of-truth for all parameters
├── main.py                      # App entry point, thread management
├── src/
│   ├── config_loader.py         # Typed YAML loader (dataclasses, __slots__)
│   ├── kinematics/
│   │   ├── __init__.py
│   │   └── kinematics.py        # 2-DOF planar IK with workspace validation
│   ├── ai/
│   │   └── yolo_detector.py     # Camera + YOLO; read/annotate split, watchdog
│   ├── plc/
│   │   └── plc_controller.py    # snap7 S7-1200 wrapper; atomic writes
│   └── ui/
│       ├── base_page.py         # Abstract CTkFrame (deduplicated helpers)
│       ├── page_auto.py         # Automatic mode page
│       └── page_manual.py       # Manual mode page
├── tests/
│   └── test_robot_system.py     # 26 test cases; hardware tests skipped without SDKs
├── requirements.txt
└── .github/workflows/ci.yml     # GitHub Actions: lint + test
```

### Thread model

| Thread | Purpose | Shutdown |
|--------|---------|----------|
| Main (Tk) | GUI event loop | `on_closing()` → `_stop_event.set()` |
| `PLCPollThread` | Cyclic PLC read at 10 Hz | Wakes on `_stop_event` |
| `AIVisionThread` | YOLO inference + PLC dispatch | Wakes on `_stop_event` |

`threading.Event._stop_event` is used instead of a raw boolean so threads
wake **immediately** on shutdown rather than sleeping through their full
interval.

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Python | ≥ 3.11 |
| Siemens S7-1200 PLC | Firmware ≥ V4.x, PUT/GET enabled |
| USB camera | Any OpenCV-compatible device |
| YOLO model | `best.pt` trained with Ultralytics YOLOv8/v11 |
| OS | Windows 10/11, Ubuntu 22.04+ |

---

## Installation

```bash
# 1. Clone the repository
git clone https://github.com/your-org/robot-control-system.git
cd robot-control-system

# 2. Create and activate a virtual environment
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux / macOS
source .venv/bin/activate

# 3. Install dependencies
pip install -r requirements.txt
```

> **GPU acceleration (optional)**  
> Replace the `torch` line in `requirements.txt` with the CUDA-enabled wheel
> from [pytorch.org](https://pytorch.org/get-started/locally/).

---

## Configuration

All runtime parameters live in **`config.yaml`**. No source code changes
are needed for a new installation.

```yaml
plc:
  ip: "192.168.0.1"     # PLC IP address
  rack: 0
  slot: 1
  db_number: 10          # Data Block number

  offsets:               # Byte offsets within the DB
    cmd_word: 0          # INT  – command register
    status: 2            # INT  – PLC status word
    j1_target: 4         # REAL – joint 1 target angle
    j2_target: 8         # REAL – joint 2 target angle
    j3_target: 12        # REAL – joint 3 target angle
    j4_target: 16        # REAL – joint 4 target angle
    motion_done_byte: 20 # BOOL byte
    motion_done_bit: 0
    error_flag_byte: 20
    error_flag_bit: 1

  commands:
    idle: 0
    home: 1
    move: 2
    stop: 3

yolo:
  model_path: "models/best.pt"   # Path to your trained .pt file
  thresh: 0.50                   # Confidence threshold (0–1)
  px2mm: 0.50                    # Pixels → mm calibration factor
  home_x: 200.0                  # Robot home X in mm
  home_y: 0.0                    # Robot home Y in mm

camera:
  default_index: 0               # OpenCV camera index
  display_width: 440             # Preview width (px)
  display_height: 310            # Preview height (px)
  fps: 30                        # Target inference rate

kinematics:
  l1: 200.0                      # Link-1 length (mm)
  l2: 150.0                      # Link-2 length (mm)

app:
  plc_poll_interval: 0.10        # Seconds between PLC reads
  move_cooldown: 1.50            # Seconds to wait after issuing MOVE
```

### Calibrating `px2mm`

1. Place a known-size object (e.g. 100 mm ruler) in the camera field of view.
2. Measure its pixel width in the captured frame.
3. `px2mm = physical_width_mm / pixel_width_px`

---

## Running the Application

```bash
python main.py
```

The application opens in **Automatic Mode** by default. Use the
**"SWITCH TO MANUAL >>"** button in the status bar to navigate to Manual Mode.

### Automatic Mode

- Live YOLO inference runs continuously.
- Detected defects trigger inverse-kinematics computation and a `MOVE`
  command to the PLC.
- Joint angles are displayed in real time from the cyclic PLC read.

### Manual Mode

- Enter joint angles directly (Forward Kinematics) or target XY coordinates
  (toggle to Inverse Kinematics).
- Individual gripper GRIP / RELEASE buttons.
- **MOVE TO HOME** sends `CMD_HOME` to the PLC.

---

## Project Structure

```
src/config_loader.py
```

`load_config(path?)` → `RobotConfig` – fully-typed dataclass tree.
All other modules receive their configuration through constructor injection
(no global state).

```
src/plc/plc_controller.py
```

`PLCController(cfg)` – wraps `snap7.client.Client`.  All methods guard
against disconnected state and log errors rather than raising, keeping the
GUI alive during transient faults.

```
src/ai/yolo_detector.py
```

`YOLODetector(model_path, thresh, px2mm, home_x, home_y)` – owns a single
`cv2.VideoCapture`.  `process_frame()` returns
`(has_defect, robot_x_mm, robot_y_mm, PIL.Image)`.

```
src/ui/base_page.py
```

`BasePage(parent, controller, page_color)` – abstract `CTkFrame` providing
`build_camera_selector()`, `build_status_bar()`, and
`_refresh_error_status()` shared by both pages.

---

## Running Tests

```bash
# Run all tests with coverage
pytest tests/ --cov=src --cov-report=term-missing -v

# Run a single test class
pytest tests/test_robot_system.py::TestPLCController -v
```

Tests mock all hardware (`snap7.client.Client`, `cv2.VideoCapture`, YOLO
model) so they execute in any headless environment without physical devices.

### Test matrix

| ID | Class | What is tested |
|----|-------|---------------|
| TC-01 | `TestConfigLoader` | Full YAML parses to correct fields (incl. new camera fields) |
| TC-02 | `TestConfigLoader` | Missing file → `FileNotFoundError` |
| TC-03 | `TestConfigLoader` | Partial YAML uses safe defaults (incl. inference/resolution fields) |
| TC-04 | `TestPLCController` | `connect()` returns `True` on success |
| TC-05 | `TestPLCController` | `connect()` returns `False` on exception |
| TC-06 | `TestPLCController` | `read_status()` decodes DB bytes correctly |
| TC-07 | `TestPLCController` | `read_status()` returns `{}` when offline |
| TC-08 | `TestPLCController` | `send_command()` writes correct 2-byte buffer |
| TC-09 | `TestPLCController` | `send_command()` skips when disconnected |
| TC-10 | `TestPLCController` | `send_joint_targets()` writes 16-byte buffer |
| TC-11 | `TestPLCController` | `send_joint_targets_and_command()` atomic write |
| TC-12 | `TestYOLODetector` | `start_camera()` returns `True` on success |
| TC-13 | `TestYOLODetector` | `read_frame()` returns `None` when no camera |
| TC-14 | `TestYOLODetector` | `annotate_frame()` – no detections → home coordinates |
| TC-15 | `TestYOLODetector` | `annotate_frame()` – above-thresh defect → robot coords |
| TC-16 | `TestYOLODetector` | `switch_camera()` releases previous capture |
| TC-17 | `TestYOLODetector` | `DetectionResult.to_pil()` resizes and converts |
| TC-18 | `TestYOLODetector` | Model path not found → `FileNotFoundError` |
| TC-19 | `TestInverseKinematics` | Reachable point → non-zero joint angles |
| TC-20 | `TestInverseKinematics` | Out-of-reach point → `WorkspaceError` raised |
| TC-21 | `TestInverseKinematics` | J3 / J4 are always `0.0` |
| TC-22 | `TestInverseKinematics` | Negative link length → `ValueError` |
| TC-23 | `TestInverseKinematics` | Edge-of-workspace (fully extended) |
| TC-24 | `TestInverseKinematics` | Edge-of-workspace (fully folded) |
| TC-25 | `TestInverseKinematics` | `reachable()` correctly classifies workspace |
| TC-26 | `TestBasePage` | `update_video()` stores PhotoImage reference |
| TC-27 | `TestPLCDbReadSize` | `compute_db_read_size()` rounds up to multiple of 4 |

> Tests TC-04–TC-10 (PLC) and TC-12–TC-18 (YOLO) and TC-26 (BasePage) are
> skipped when the respective hardware SDK (`snap7`, `cv2`, `customtkinter`)
> is not installed, allowing the full suite to run in a headless CI environment.

---

## CI Pipeline

GitHub Actions workflow (`.github/workflows/ci.yml`) runs on every push and
pull request to `main` / `develop`:

```
Push / PR
  └── lint-and-test (ubuntu-latest, Python 3.11 & 3.12)
        ├── black --check          # Formatting gate
        ├── ruff check             # Linting gate
        └── pytest --cov-fail-under=80
```

The pipeline installs only lightweight CI-safe packages; hardware SDKs are
replaced by `unittest.mock` stubs at test time.

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| `threading.Event` for shutdown | Threads wake immediately on stop, eliminating zombie processes |
| Constructor-injected `RobotConfig` | No global state; fully testable without filesystem access |
| `self.after(0, callback)` for GUI updates | Ensures PLC data is dispatched on the Tk main thread (thread-safety) |
| `BasePage` abstract base class | Eliminates camera-selector and status-bar duplication across pages |
| `px2mm` in config | Camera-agnostic; recalibrate by changing one value, no code changes |
| Bare `except` replaced with typed catches | Prevents silent swallowing of `KeyboardInterrupt` / `SystemExit` |
| `src/kinematics/` module | Inverse kinematics isolated for unit-testing without GUI dependencies |
| `read_frame` / `annotate_frame` split | Camera lock held only during fast read; YOLO inference runs unlocked |
| `DetectionResult` dataclass | Separates detection state from PIL conversion; enables lazy rendering |
| Exponential backoff for PLC reconnect | Avoids network flood when PLC is offline (max 30 s backoff) |
| `__slots__` on all dataclasses | Reduces per-instance memory overhead during 10 Hz polling loops |
| Rotating file log handler | Production-grade log retention without manual rotation |
| Atomic `send_joint_targets_and_command` | Single `db_write` avoids PLC race condition between targets and command |
