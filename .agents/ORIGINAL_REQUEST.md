# Original User Request

## 2026-09-15T08:56:53Z

Refactor and enhance the 4-Axis Robot Arm PLC S7-1200 Control System to resolve critical motion control architecture flaws, memory safety issues in PLC communication, joint limit violations, and deliver key vision/UI features (StabilityTracker, 2D Digital Twin, interactive ROI, and i18n).

Working directory: d:\Newfolder\1234
Integrity mode: development

## Requirements

### R1. Fix Motion Control Architecture in SortingController (PC Master Model)
- Refactor `SortingController.execute_sort()` and `_move_and_wait()` to implement the PC Master model:
  - Remove the premature `self.plc.send_pulse(*ADDR.START_AUTO)` trigger from `execute_sort()`.
  - For each waypoint (`pick_up`, `pick_down`, `place_up`, `place_down`): call `self.plc.send_joint_targets(j1, j2, j3, j4)` and trigger motion execution via configured PLC command (e.g. `send_command(cfg.plc.commands.move)`).
  - Supervise motion by polling `motion_done` from `plc.read_status()` with a deterministic timeout. Raise an explicit `RuntimeError` if the deadline is exceeded or if an error flag is raised.
  - Actuate gripper (`write_bit(*ADDR.GRIP, True/False)`) strictly after `motion_done` confirms target arrival.
  - If PLC autonomous sequence mode is retained for future use, isolate it into a dedicated `execute_sort_plc_sequence()` method rather than interleaving with PC Master logic.

### R2. Dynamic PLC Target Buffer & Memory Safety (Fix j4_target == -1 Overwrite)
- Update `PLCController.send_joint_targets()` in `src/plc/plc_controller.py`:
  - Dynamically assemble payload buffer based on valid target offsets (ignoring unmapped offsets such as `j4_target = -1`).
  - When `j4_target == -1`, write only 12 bytes (`j1..j3`), preventing memory corruption of bytes 54–57 (`theta_j2_ht`).
  - Keep PLC DB structure intact (do not modify existing PLC data block offsets or data types).

### R3. Comprehensive Joint Limits Validation across Kinematics & Auto Cycle
- Centralize `JOINT_LIMITS` into a single source-of-truth module (e.g., `src/kinematics/joint_limits.py` or within `src/kinematics/__init__.py`) loaded from `config.yaml`.
- In `src/kinematics/kinematics.py:inverse_kinematics()`, validate computed joint angles against `JOINT_LIMITS`. Raise `WorkspaceError` if any angle exceeds hardware limits.
- In `SortingController._ik()`, ensure joint limit `WorkspaceError` aborts the sort cycle safely, logs the diagnostic error, and surfaces the issue to the UI without commanding hazardous motion.

### R4. Integrate StabilityTracker & Continuous Auto-Pick Mode
- Integrate `src/ai/stability_tracker.py` into the camera/AI vision thread in `main.py`:
  - Feed detection coordinates from YOLO into `StabilityTracker` per frame.
  - Render lock progress and bounding box feedback via `draw_lock_overlay()`.
  - Add a toggle in GUI for "Continuous Auto-Pick": when active, automatically invoke `execute_sort()` upon achieving `LOCKED` state. Default to OFF so manual button "CHỤP & PHÂN LOẠI" remains functional.

### R5. Optimize Pulse Mechanism to Reduce Thread Churn
- Refactor `PLCController.send_pulse()` to eliminate creating transient `threading.Timer` instances on every pulse.
- Use a dedicated worker queue or UI-safe timer dispatch to handle pulse falling edges sequentially, ensuring pending pulses can be cancelled cleanly during JOG button release or shutdown.

### R6. UI/UX Enhancements
- **Interactive ROI Selection**: Allow interactive dragging/adjustment of the ROI rectangle directly over the camera feed (or via sliders), with ability to persist changes to config.
- **2D Digital Twin Visualizer**: Embed a 2D kinematic arm visualizer on GUI (using Tkinter Canvas or embedded lightweight plotting) showing real-time link/joint poses and Elbow-Up/Down state for both Manual and Auto modes.
- **AI Preprocessing Config**: Add `camera.enable_unsharp_mask` boolean flag in `config.yaml` (default `true`) allowing users to bypass unsharp masking if unnecessary. Document ONNX/TensorRT export in README.
- **i18n Localization**: Create a clean dictionary-based localization system (`src/ui/i18n.py`) supporting Vietnamese (`vi`) and English (`en`), with consistent terminology across all pages.

---

## Acceptance Criteria

### Automated Testing & Linting
- [ ] `pytest tests/ --cov=src` passes 100% with no regression in test coverage.
- [ ] New unit tests in `tests/test_robot_system.py` asserting that `send_joint_targets()` writes exactly 12 bytes when `j4_target == -1` without touching byte 54+.
- [ ] New unit tests verifying that `inverse_kinematics()` raises `WorkspaceError` when a geometric pose produces joint angles exceeding configured limits.
- [ ] Unit tests for `SortingController` verifying the new PC Master sequence (`send_joint_targets` -> `motion_done` poll -> `GRIP` toggle).
- [ ] Code formatted and checked with `black --check` and `ruff check`.

### Operational Verification
- [ ] `test_gui_no_plc.py` runs cleanly without hardware, supporting all simulated motions in PC Master mode.
- [ ] Manual mode JOG and kinematics calculation/sending operate reliably without thread explosion.
- [ ] Toggle between Continuous Auto-Pick and Manual Capture works as expected with visual lock overlay.
- [ ] 2D arm visualizer renders valid forward-kinematic link positions corresponding to current joint angles.
- [ ] Summary report provided for each group detailing modified files, before-vs-after behavior, and parameters requiring user hardware confirmation.
