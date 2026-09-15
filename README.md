# 4-Axis Robot Control System

A production-grade Python application for automated defect detection and
robotic pick-and-sort using **YOLOv11**, a **Siemens S7-1200 PLC**, and a
**CustomTkinter** GUI.

```
Camera → YOLODetector (YOLO inference on ROI crop)
                ↓ class (GOOD/BAD) + defect coords (mm)
        Inverse Kinematics (config-driven DH parameters)
                ↓ joint angles
        SortingController (cycle orchestration + safety polling)
        PLCController (snap7) → S7-1200 → 4-DOF Robot
```

---

## Table of Contents

1. [Architecture](#architecture)
2. [Prerequisites](#prerequisites)
3. [Installation](#installation)
4. [Configuration](#configuration)
5. [PLC Address Map](#plc-address-map)
6. [Running the Application](#running-the-application)
7. [Project Structure](#project-structure)
8. [Running Tests](#running-tests)
9. [CI Pipeline](#ci-pipeline)
10. [Key Design Decisions](#key-design-decisions)

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
│   │   └── kinematics.py        # 4-DOF FK / geometric IK, workspace validation
│   ├── ai/
│   │   └── yolo_detector.py     # Camera + YOLO; read/annotate split, watchdog
│   ├── plc/
│   │   └── plc_controller.py    # snap7 S7-1200 wrapper; ADDR map; atomic writes
│   ├── robot/
│   │   └── sorting_controller.py # Pick-and-place FSM + PLC health monitoring
│   └── ui/
│       ├── theme.py             # Central colour palette
│       ├── base_page.py         # Abstract CTkFrame (cards, headers, status bar)
│       ├── header.py            # VAAHeader branding + VAAFooter status chips
│       ├── page_auto.py         # Automatic mode page
│       └── page_manual.py       # Manual mode page (FK/IK, JOG)
├── tests/
│   ├── test_robot_system.py     # Config, IK, PLC, YOLO, sorting (mocked hw)
│   └── test_page_manual.py      # Manual-page JOG/validation logic
├── test_gui_no_plc.py           # Full GUI with MockPLCController (no hardware)
├── requirements.txt
├── requirements-dev.txt
└── .github/workflows/ci.yml     # GitHub Actions: lint + test
```

### Thread model

| Thread | Purpose | Shutdown |
|--------|---------|----------|
| Main (Tk) | GUI event loop | `on_closing()` → `_stop_event.set()` |
| `PLCPollThread` | Cyclic PLC read at 10 Hz | Wakes on `_stop_event` |
| `AIVisionThread` | Frame capture + ROI overlay + YOLO dispatch | Wakes on `_stop_event` |
| `SortCycleThread` | One pick-and-place cycle at a time | Joins on shutdown |

`threading.Event._stop_event` is used instead of a raw boolean so threads
wake **immediately** on shutdown rather than sleeping through their full
interval.

### Safety interlocks

* A sort cycle is **refused** while the PLC error flag is active.
* During every waypoint the sort controller polls the PLC and aborts
  immediately if the PLC goes offline or raises its error flag.
* A post-sort cooldown (3 s) suppresses immediate re-triggering on the
  same part, and only one `SortCycleThread` may run at a time.

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Python | ≥ 3.11 |
| Siemens S7-1200 PLC | Firmware ≥ V4.x, PUT/GET enabled |
| USB camera | Any OpenCV-compatible device |
| YOLO model | `my_model/my_model.pt` trained with Ultralytics YOLOv8/v11 |
| OS | Windows 10/11 (primary), Linux supported via CAP_ANY backend |

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

# 4. Development / CI tooling (optional)
pip install -r requirements-dev.txt
```

> **GPU acceleration (optional)**
> Uncomment the `torch` line in `requirements.txt` for CPU-only hosts or
> install the CUDA-enabled wheel from [pytorch.org](https://pytorch.org/get-started/locally/).

---

## Configuration

All runtime parameters live in **`config.yaml`** — including the DH link
lengths and joint limits used by the kinematics engine. No source code
changes are needed for a new installation.

```yaml
plc:
  ip: "192.168.0.1"     # PLC IP address
  rack: 0
  slot: 1
  db_number: 5          # Data Block number

  offsets:              # Byte offsets within the DB (see Address Map below)
    j1_target: 42
    ...
    phan_loai_hang_byte: 78   # Classification bits
    hang_tot_byte: 78
    hang_xau_byte: 78

yolo:
  model_path: "my_model/my_model.pt"
  thresh: 0.50          # Confidence threshold (0–1)
  px2mm: 0.50           # Pixels → mm calibration factor
  home_x: 200.0         # Robot home X in mm
  home_y: 0.0           # Robot home Y in mm
  roi_x: 190            # Manual-capture ROI crop (x, y, w, h)

camera:
  default_index: 0
  display_width: 440    # Preview width (px)
  display_height: 310
  fps: 60               # Camera read rate
  preview_fps: 20       # GUI redraw rate (decoupled to save CPU)
  inference_width: 640
  inference_height: 480
  read_timeout: 2.0

kinematics:             # ← drives the IK/FK engine at start-up
  a1: 40.0              # Base horizontal offset (mm)
  d1: 300.0             # Base height (mm)
  a2: 190.0             # Link 2 length (mm)
  a3: 110.0             # Link 3 length (mm)
  a4: 65.0              # End-effector length (mm)
  j2_min: 0.0           # Joint limits used for UI validation
  j2_max: 250.0
  default_phi: 0.0      # Default end-effector pitch for IK

app:
  plc_poll_interval: 0.10
  move_cooldown: 1.50

sort_positions:
  place_good: {x: 150.0, y: 100.0, z_down: 80.0, z_up: 150.0}
  place_bad:  {x: -150.0, y: 100.0, z_down: 80.0, z_up: 150.0}
  gripper_delay: 0.5
```

### Calibrating `px2mm`

1. Place a known-size object (e.g. 100 mm ruler) in the camera field of view.
2. Measure its pixel width in the captured frame.
3. `px2mm = physical_width_mm / pixel_width_px`

---

## PLC Address Map

Hard-wired bit addresses live in one place: the `ADDR` class in
`src/plc/plc_controller.py`. The GUI and sorting controller reference these
constants instead of scattering magic numbers.

| Address | Symbol | Meaning |
|---------|--------|---------|
| 0.0 | `START_AUTO` | Pulse starts the PLC motion sequence |
| 0.1 | `PAUSE` | Pause |
| 0.2 | `AUTO_MODE` | Switch to AUTO mode (pulsed when Auto page opens) |
| 14.0 | `GRIP` | Gripper close during sort cycle |
| 16.0 | MANUAL_MODE | Switch to MANUAL mode |
| 16.1 | `MOVE_TO_HOME` | Return to home position |
| 16.2 / 16.3 | `GAP_VAT` / `NHA_VAT` | Pick object / release object |
| 16.4 | `STOP_ROBOT` | Emergency stop |
| 16.5 / 16.6 | `KINE_FORWARD` / `KINE_INVERSE` | Select forward / inverse kinematics mode |
| 80.0–80.5 | `JOG_J1..J3_*` | Jog toggles per axis |
| cfg 20.x | `motion_done`, `error_flag` | Status flags (offsets configurable) |
| cfg 78.x | classification bits | PHAN_LOAI_HANG / HANG_TOT / HANG_XAU |

Words: 18–40 = FK results block · 42–64 = IK data block · joint targets at
the configured `offsets.j*_target`.

---

## Running the Application

```bash
python main.py          # Real hardware
python test_gui_no_plc.py   # Full GUI against an in-memory mock PLC
```

The application opens in **Automatic Mode** by default. Use the header
navigation tabs (or the footer status chips) to switch modes.

### Automatic Mode

* Live ROI-overlay camera preview (throttled to `preview_fps`).
* Press **CHỤP & PHÂN LOẠI** to capture the ROI crop, sharpen it, run YOLO,
  classify GOOD/BAD, write the classification bits, pulse START_AUTO, and
  orchestrate the pick-and-place cycle. Cycles are skipped when nothing is
  detected, while another cycle runs, during the post-sort cooldown, or
  when the PLC error flag is active.

### Manual Mode

* Enter joint angles directly (Forward Kinematics) or target XYZ coordinates
  (toggle to Inverse Kinematics); CALCULATE shows results, SEND transmits
  them to the PLC.
* Pick / release buttons, MOVE TO HOME, STOP ROBOT.
* Per-axis JOG toggles (J1 left/right, J2 up/down, J3 up/down).
* All inputs are validated against the configured joint limits.

Logs rotate under `logs/robot_app.log` (20 MB × 5 files).

---

## Project Structure

```
src/config_loader.py
```
`load_config(path?)` → `RobotConfig` – fully-typed dataclass tree with
validation. All other modules receive their configuration through
constructor injection (no global state), except the kinematics module whose
DH parameters are configured once at start-up via `configure()`.

```
src/plc/plc_controller.py
```
`PLCController(cfg)` – wraps `snap7.client.Client` behind an RLock (snap7 is
not thread-safe). Bit writes that share a byte are grouped into a single
read-modify-write cycle (`write_bits`) so concurrent writers cannot lose
each other's changes. Push-button pulses use tracked timers so repeated
pulses restart rather than stack. All methods guard against disconnected
state and log errors rather than raising, keeping the GUI alive during
transient faults.

```
src/ai/yolo_detector.py
```
`YOLODetector(model_path, thresh, px2mm, ...)` – owns a single
`cv2.VideoCapture` plus a keep-newest frame queue fed by a dedicated read
thread. `process_frame()` returns `(has_defect, robot_x_mm, robot_y_mm,
PIL.Image)`.

```
src/robot/sorting_controller.py
```
`SortingController(plc, positions, kinematics, commands)` – orchestrates one
pick-and-place cycle at a time, mirrors commanded waypoints for the GUI, and
polls PLC health (offline / error flag / motion_done) between steps.

```
src/ui/base_page.py
```
`BasePage(parent, controller, page_color)` – abstract `CTkFrame` providing
card/section-header factories, `build_camera_selector()`,
`build_status_bar()`, and `_refresh_error_status()` shared by both pages.

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

| Suite | What is covered |
|-------|-----------------|
| `TestConfigLoader` | Full/partial YAML parsing, missing file, validation |
| `TestInverseKinematicsModule` | FK zero pose, FK↔IK round trip, workspace errors, base rotation |
| `TestPLCController` | Connect/disconnect, DB decode, command pulses, target writes |
| `TestYOLODetector` | Camera lifecycle, detection→robot coords, PIL conversion |
| `TestBasePage` | Video-label reference handling |
| `TestPLCDbReadSize` | DB size rounding |
| `TestSortingController` | IK fallbacks, PLC call sequence, counters |
| `test_page_manual.py` | JOG toggling, button feedback, TX logging, joint limits |

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
| `configure()` for kinematics | config.yaml DH parameters drive IK/FK – no hardcoded duplicates |
| `self.after(0, callback)` for GUI updates | Ensures PLC data is dispatched on the Tk main thread (thread-safety) |
| Single `ADDR` address map | Eliminates magic bit numbers scattered across UI code |
| Atomic `write_bits` grouping | Concurrent writers can no longer clobber shared bytes |
| Tracked pulse timers | Repeated pulses restart cleanly; disconnect cancels pending resets |
| `BasePage` abstract base class | Card/header/status-bar helpers shared by all pages |
| `px2mm` + ROI in config | Camera-agnostic recalibration without code changes |
| Preview FPS decoupled from camera FPS | Smooth video without wasting CPU on 60 Hz widget redraws |
| Sort E-stop gate + health polling | Motion is refused/aborted whenever the PLC signals a fault |
| Post-sort cooldown + single sort thread | Prevents duplicate cycles on the same detection |
| Bare `except` replaced with typed catches | Prevents silent swallowing of `KeyboardInterrupt` / `SystemExit` |
| Exponential backoff for PLC reconnect | Avoids network flood when PLC is offline (max 30 s backoff) |
| Rotating file log handler | Production-grade log retention without manual rotation |
