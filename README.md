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
configuration uses `my_model/my_model.pt`). Camera access, the model file and a
matching PLC project are required for operation. For a simulated PLC with the
real camera/model, use `python test_gui_no_plc.py`.

## Configuration and calibration

- `kinematics`: DH dimensions `a1, d1, a2, a3, a4`, joint limits and tool pitch.
  Keep the configured limits consistent with the physical robot. Elbow DOWN is
  disabled when the configured J3 range has no negative angles.
- `yolo`: threshold, pixel-to-mm scale, ROI and optional OpenCV calibration file.
  Model and calibration paths are relative to the source project root, or to the
  executable directory in a frozen build. Settings applies threshold, scale and
  cooldown immediately; ROI edits update the live overlay.
- `yolo.class_quality`: explicit model class name or numeric ID to GOOD/BAD map.
  Class names take precedence over IDs. Update this map with every retrained
  model; prefer names to protect against reordered IDs. Unknown classes are
  drawn as UNKNOWN and do not trigger motion. The supplied numeric mapping keeps
  the original model convention: 0 = BAD, 1 = GOOD.
- `app.move_cooldown`: delay after a Vision sort attempt before accepting another.
- `app.qr_debounce_frames`, `qr_clear_frames`, `qr_miss_tolerance`: defaults are
  5, 3 and 2. Three decoded blank frames rearm the same QR; up to two missed
  frames tolerate decoding flicker. Tune to frames actually processed, not the
  camera's advertised frame rate. Two identical boxes without a detectable gap
  cannot be distinguished by this camera-only method; production item identity
  should come from a conveyor sensor or PLC item counter.
- Settings is validated before save, and the YAML file is replaced atomically.
  Apply settings after the current sort cycle has finished.

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
layout. Assign dedicated motion/error status bytes in both TIA and config;
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

Joint targets are written before a command; a failed target write suppresses
that command. Noncontiguous target fields are written in separate groups so the
bytes in gaps are preserved. HOME issues one HOME command, without START_AUTO.
Bit-group writes are serialized against other PC threads; read-modify-write
cannot prevent the PLC itself from changing another bit in the same byte.
Keep PC-owned command bytes separate from PLC-owned status bytes.

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
```

The Qt thread owns widgets. PLC polling, camera processing, camera reads, pulse
resets and sorting run on workers. Signals deliver UI updates to the Qt thread.
No CI workflow is currently shipped; run the tests locally.

## Tests

```powershell
python -m pip install -r requirements-dev.txt
$env:QT_QPA_PLATFORM = "offscreen"
$env:PYTEST_QT_API = "pyside6"
python -m pytest tests -q
```

Tests mock the PLC/camera/model and do not certify the physical robot or TIA
logic. If importing `PySide6.QtCore` fails with a DLL error, repair that Python
environment or use an environment with a working matching PySide6 installation.

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
