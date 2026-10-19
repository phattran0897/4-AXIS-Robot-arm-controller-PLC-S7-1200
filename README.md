# 4-Axis Robot Arm Controller

Python/PySide6 GUI for an articulated robot arm, a Siemens S7-1200 PLC,
YOLO object detection and QR destination selection. Runtime settings live in
`config.yaml`.

## Operating modes

- **QR / PLC sequencer**: the camera debounces QR values, looks up joint presets
  in `sort_positions.locations`, and writes the destination targets. This path
  does not pulse START_AUTO. The PLC owns the conveyor sensor, gripping and
  pick/place sequence. A successful target write is counted as QR WRITTEN; this
  is not confirmation that a physical box was sorted. Unknown codes increment
  INVALID QR once per visible item; connection failures are retried, not counted
  as invalid codes.
- **Vision / PC Master**: the preview runs continuously; YOLO classification runs
  when the operator presses CAPTURE. The PC computes IK waypoints, commands each
  move, waits for fresh motion completion, operates the gripper and returns home.
  Continuous automatic inference is not implemented.

## Install and run

Use Python 3.11 or later in an isolated environment:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py
```

Put the trained detection model at the configured `yolo.model_path` (the supplied
configuration uses `my_model/my_model.pt`). Vision requires these weights; QR
starts without loading YOLO and loads it only if Vision inference is requested.
Camera access and a matching PLC project are required for operation. For a simulated PLC with the
real camera, use `python test_gui_no_plc.py`. The demo uses the same vision
loop as the app. In QR mode, target writes do not start motion: press
SIMULATE BOX SENSOR to simulate the conveyor sensor separately.

## Configuration and calibration

- `kinematics`: DH dimensions `a1, d1, a2, a3, a4`, joint limits and tool pitch.
  Keep the configured limits consistent with the physical robot. Elbow DOWN is
  disabled when the configured J3 range has no negative angles.
- `yolo`: threshold, pixel-to-mm scale, ROI and optional OpenCV calibration file.
  Model and calibration paths are relative to the source project root, or to the
  executable directory in a frozen build. Settings applies threshold, scale and
  cooldown when Apply is pressed; ROI edits remain a draft until Apply succeeds.
  Settings cannot apply during inference or a sorting cycle.
- `yolo.class_quality`: explicit model class name or numeric ID to GOOD/BAD map.
  Class names take precedence over IDs. Update this map with every retrained
  model; prefer names to protect against reordered IDs. Unknown classes are
  drawn as UNKNOWN and do not trigger motion. The supplied numeric mapping keeps
  the original model convention: 0 = BAD, 1 = GOOD.
- `yolo.coordinate_origin`: `roi` preserves the existing calibration: home_x/y
  describe the centre of the cropped ROI. `frame` uses the centre of the full
  camera image and transforms crop coordinates back into it. Set this to match
  the actual calibration, not to compensate a target by trial and error.
  Resizing for inference preserves the pixel-to-mm scale of the original image.
- `app.plc_poll_interval`: applied on the next connected polling iteration.
- `app.move_cooldown`: delay after a Vision sort attempt before accepting another.
- `app.qr_debounce_frames`, `qr_clear_frames`, `qr_miss_tolerance`: defaults are
  5, 3 and 2. Three frames with no visible QR outline rearm the same QR; up
  to two missed frames tolerate decoding flicker. A visible but undecodable QR
  does not rearm an item. CLEAR is emitted once per empty streak. Tune to frames actually processed, not the
  camera's advertised frame rate. Two identical boxes without a detectable gap
  cannot be distinguished by this camera-only method; production item identity
  should come from a conveyor sensor or PLC item counter.
- Settings rejects duplicate YAML keys, wrong types, non-finite numbers and
  fractional integer fields. Save preserves comments and unknown keys, adds
  missing settings in older files, and verifies the complete round trip before
  atomically replacing the file. Apply after the current sort cycle has finished.

Calibrate `px2mm` with a known physical length at the working plane. Validate the
ROI centre against `home_x/home_y`, camera mounting and image axis directions.
A scalar pixel conversion is not a substitute for camera-to-robot calibration
across varying object heights. The application does not plan collision-free
paths or check obstructions. MoveL samples Cartesian waypoints; the PLC decides
how it interpolates motion between those waypoints.

## PLC integration contract

Use an accessible, non-optimized DB with PUT/GET configured in the TIA project.
Check every address and field meaning against that project before commanding
motion. `PLCOffsets` contains configurable REAL and status addresses; `ADDR`
contains the fixed command, gripper and JOG bits.

**The supplied DB map needs verification:** status byte 20 overlaps the
`theta_j1` REAL at bytes 18-21. PC-supervised sorting refuses to move with this
layout. Startup disables START/CAPTURE with an explanatory dialog in Vision
mode; Settings rejects switching to Vision until the map is valid. The map
validation also rejects joint targets overlapping FK/IK views or command bytes.
Assign dedicated motion/error status bytes in both TIA and config;
changing only the Python address is insufficient. The current target offsets
42/46/50 also alias the XYZ input view `px_j1/py_j2/pz_j3`. Confirm whether the
PLC expects joint angles or Cartesian coordinates in each operating mode and
remap these fields together with the PLC program as necessary. J4 is currently
unmapped (`j4_target: -1`), so Python writes only J1-J3; a four-axis physical
sequence requires a corresponding PLC mapping.

For **each PC-commanded waypoint and autonomous sequence**:

1. Accept a fresh command edge and clear `motion_done`.
2. Hold `motion_done` LOW long enough for the PC to observe a busy state.
3. Raise it only after that command reaches its destination.
4. Keep the error flag accurate throughout the motion.

The PC polls every 50 ms and accepts completion only after seeing LOW then HIGH
following the command. A stale HIGH never advances the gripper sequence. A
reset pulse shorter than the polling/network interval can be missed; the PLC
must provide a visible busy interval or a request/acknowledgement protocol.
The default waypoint timeout is 30 seconds, not a joint-speed estimate. On a
fault, the cycle enters ERROR and requires correction and an explicit reset.
Two consecutive missing status samples are tolerated within the existing motion
deadline (no command is retried). Any missing sample invalidates the prior busy
observation: a fresh LOW then HIGH is required after communication recovers.
Three missing reads or a PLC error flag abort the cycle and request software STOP.

Joint targets are written before a command; a failed target write suppresses
that command. Noncontiguous target fields are written in separate groups so the
bytes in gaps are preserved. HOME issues one HOME command, without START_AUTO.
All REAL values are checked for finite float32 representation before the first
write. A logging/UI callback failure cannot turn a successful write into a retry.
When classification bits span multiple bytes, the valid flag is cleared before
updating the payload and asserted only after all payload writes succeed.
Bit-group writes are serialized against other PC threads; read-modify-write
cannot prevent the PLC itself from changing another bit in the same byte.
Keep PC-owned command bytes separate from PLC-owned status bytes.

Repeated command pulses are scheduled with at least a 50 ms LOW gap by the
pulse worker; callers never hold the PLC lock while sleeping for that gap.
On STOP or window close, waits are cancelled and subsequent PC motion writes
are inhibited. Pending pulse edges and JOG commands are cancelled. STOP_ROBOT
is held HIGH across disconnect so shutdown does not truncate it; GRIP is retained.
A failed STOP write is logged as unconfirmed, not reported as a physical stop.
CLEAR ERROR never pulses PAUSE: it waits for error_flag LOW and motion_done HIGH
before releasing STOP and resetting the local controller. For a PLC-owned fault,
configure a dedicated `plc.offsets.error_ack_byte/error_ack_bit` matching TIA;
byte -1 disables remote acknowledgement. If an acknowledgement is sent, wait for
the PLC fault to clear before resetting again. PLC logic must treat STOP as a
held stop request and report stopped/completed through motion_done; this handshake
and restart policy must be validated on the machine.

Software fault detection is not an emergency-stop system. Machine stopping,
interlocks and actuator safety belong in the PLC/hardware design.

## Source layout

```text
main.py                         Qt application, camera/PLC threads, sort worker
src/config_loader.py            Typed config, validation, atomic settings save
src/ai/yolo_detector.py          Camera read worker, inference, overlays
src/ai/qr_reader.py              QR decoding, debounce and rearming
src/ai/stability_tracker.py      Standalone tracker; not used by the live loop
src/kinematics/                  Articulated-arm FK/IK and joint limits
src/plc/plc_controller.py        Serialized snap7 access, grouped writes, pulses
src/robot/sorting_controller.py  QR destinations and PC pick/place state machine
src/ui/                         Auto, Manual, Settings, header and shared widgets
tests/                          Mock-based automated tests
tools/prepare_build.py          Icon conversion and external runtime asset copy
tools/diagnose.py               Offline dependency/configuration diagnostics
.github/workflows/checks.yml    Windows tests and static checks
pyproject.toml                 Pytest, coverage and Ruff settings
```

The Qt thread owns widgets. PLC polling, camera switching/processing, camera
reads, pulse resets and sorting run on workers. Signals deliver UI updates.
A failed status read invalidates the displayed PLC feedback immediately. START
and CAPTURE are disabled while offline, stopped or faulted. Busy captures are
discarded and switching to Manual is blocked during classification/sorting.
Manual calculations are invalidated when inputs change; failed sends never show
success. JOG reversal clears the opposite direction first, and leaving Manual
or deactivating the window requests release of active JOG bits.

The dark UI groups controls, camera preview and status into separate panels.
Dense pages scroll at smaller window sizes; **Apply & save settings** stays in
the Settings heading, and **Show log** expands the activity log. Hover over
section names, field labels, inputs, buttons or readouts for Vietnamese help
with units and operating steps. Help remains available on disabled controls;
Start and Capture also explain the current blocking condition.

Camera switches wait for the previous read worker to exit. If a device driver
is blocked, the switch fails explicitly and the old worker retains its own
capture until it returns; it cannot read from the replacement camera.

Application and `src` diagnostics share rotating logs in `logs/` beside the
source or EXE. If that directory is not writable, Windows uses
`%LOCALAPPDATA%/RobotController/logs/`. The UI telemetry keeps the latest 100
lines and renders QR text literally. Startup errors are shown in a dialog even
in a windowed executable.

Run `python tools/diagnose.py` to check the selected interpreter, dependency
imports, config, model path and Vision DB layout without opening a camera or
connecting to the PLC. Use `--json` for machine-readable output or `--config PATH`
to inspect another configuration. Exit code 1 indicates an error; the known DB
overlap is a warning in QR mode and an error in Vision mode.

## Tests

```powershell
python -m pip install -r requirements-dev.txt
$env:QT_QPA_PLATFORM = "offscreen"
$env:PYTEST_QT_API = "pyside6"
python -m pytest tests -q
python -m ruff check .
python -m pytest --cov --cov-report=term-missing
```

Tests mock the PLC/camera/model and do not certify the physical robot or TIA
logic. If importing `PySide6.QtCore` fails with a DLL error, repair that Python
environment or use an environment with a working matching PySide6 installation.
The GitHub Actions workflow runs these checks on Windows with Python 3.11 and
3.12 for pushes to main and pull requests. Its action configuration follows the
[checkout](https://github.com/actions/checkout) and
[setup-python](https://github.com/actions/setup-python) documentation. A local
test result does not imply that the hosted workflow or packaged EXE was executed.

## Windows executable

```powershell
python -m pip install pyinstaller
.\build_exe.bat
```

Run the build in the same environment that imports the application dependencies.
The batch script generates an ICO, builds `RobotController.spec`, and copies
config, assets, the configured model and optional calibration file beside
`dist/RobotController/RobotController.exe`. It stops on failures. For portable
builds, model/calibration paths must be relative and stay inside the project.
Distribute the entire output directory. Validate the EXE on the target Windows
machine with the actual camera, model and PLC before use.
