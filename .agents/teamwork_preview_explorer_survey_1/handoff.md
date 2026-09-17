# Handoff Report: Codebase Investigation for Requirements R1, R2, R3, and R5

- **Agent**: `teamwork_preview_explorer_survey_1`
- **Milestone**: Survey & Architectural Investigation
- **Target Requirements**: R1, R2, R3, R5
- **Timestamp**: 2026-09-15T09:38:00Z
- **Working Directory**: `d:\Newfolder\1234\.agents\teamwork_preview_explorer_survey_1`

---

## 1. Observation

### 1.1 Codebase File Locations
- **SortingController Location**: The user prompt and original request referenced `src/control/sorting_controller.py`. Direct filesystem inspection via `find_by_name` confirms the actual file path is:
  `d:\Newfolder\1234\src\robot\sorting_controller.py`.
- **Other Key Files**:
  - `d:\Newfolder\1234\config.yaml`
  - `d:\Newfolder\1234\src\config_loader.py`
  - `d:\Newfolder\1234\src\plc\plc_controller.py`
  - `d:\Newfolder\1234\src\kinematics\kinematics.py`
  - `d:\Newfolder\1234\src\kinematics\__init__.py`
  - `d:\Newfolder\1234\src\ui\page_manual.py`
  - `d:\Newfolder\1234\main.py`
  - `d:\Newfolder\1234\tests\test_robot_system.py`
  - `d:\Newfolder\1234\test_gui_no_plc.py`

### 1.2 Requirement R1: Motion Control Flaws in `SortingController`
Direct inspection of `src/robot/sorting_controller.py`:
- **Premature START_AUTO trigger** (`execute_sort`, lines 193–198):
  ```python
  # 0. Write classification to PLC (atomic bit-group write)
  self.plc.write_classification(result == SortResult.GOOD)

  # TRIGGER PLC TO START ITS INTERNAL MOTION SEQUENCE
  self.plc.send_pulse(*ADDR.START_AUTO)

  # 1. Pick
  self._state = RobotState.MOVING_PICK
  self._pick(pick_x, pick_y)
  ```
  `self.plc.send_pulse(*ADDR.START_AUTO)` is pulsed *before* any trajectory or waypoint kinematics are verified or sent.
- **Waypoint target transmission missing in `_move_and_wait()`** (lines 326–341):
  ```python
  target = (j1, j2, j3, j4)
  self._current_joints = target
  log.info(
      "Waypoint (commanded): J1=%.2f J2=%.2f J3=%.2f J4=%.2f",
      j1, j2, j3, j4,
  )

  if cmd is not None:
      self.plc.send_command(cmd)

  estimated = self._estimate_motion_seconds(target)
  deadline = time.monotonic() + min(timeout, estimated + _MOTION_GRACE_S)
  ```
  `self.plc.send_joint_targets(j1, j2, j3, j4)` is **never called anywhere in `_move_and_wait()` or `execute_sort()`**. Furthermore, `cmd` is `None` for all pick and place waypoints (`_pick`, `_place`), so `send_command()` is not invoked either. Only `_return_home()` passes `cmd=self.plc_commands.home`.
- **Silent failure on timeout** (lines 350–355):
  ```python
  if time.monotonic() >= deadline:
      log.debug(
          "Motion window elapsed (%.2fs est.) – continuing cycle.",
          estimated,
      )
      break
  ```
  If `motion_done` is never received and the deadline expires, `_move_and_wait()` merely logs a debug message and **breaks out of the loop without raising an error**, proceeding to actuate the gripper in `_pick()` and `_place()` regardless of robot position.

### 1.3 Requirement R2: Target Buffer Overwrite & Memory Corruption
Direct inspection of `src/plc/plc_controller.py` and `config.yaml`:
- **`config.yaml` offsets** (lines 17–20):
  ```yaml
  offsets:
    cmd_word: 0
    status: 2
    j1_target: 42
    j2_target: 46
    j3_target: 50
    j4_target: -1  # Not mapped in PLC Image, set to -1 to ignore
  ```
- **PLC DB layout** documented in `src/plc/plc_controller.py` (lines 16, 237–244):
  - Offset 42: `px_j1` (J1 target / PX_J1) — 4 bytes REAL (bytes 42–45)
  - Offset 46: `py_j2` (J2 target / PY_J2) — 4 bytes REAL (bytes 46–49)
  - Offset 50: `pz_j3` (J3 target / PZ_J3) — 4 bytes REAL (bytes 50–53)
  - Offset 54: `theta_j2_ht` (actual J2 angle feedback) — 4 bytes REAL (bytes 54–57)
  - Offset 58: `theta_j3_ht` (actual J3 angle feedback) — 4 bytes REAL (bytes 58–61)
  - Offset 62: `theta_j1_ht` (actual J1 angle feedback) — 4 bytes REAL (bytes 62–65)
- **`PLCController.send_joint_targets()`** (lines 427–435):
  ```python
  buf = bytearray(16)
  set_real(buf, 0, j1)
  set_real(buf, 4, j2)
  set_real(buf, 8, j3)
  set_real(buf, 12, j4)
  self._client.db_write(
      self._cfg.db_number, self._cfg.offsets.j1_target, buf
  )
  ```
  Regardless of `j4_target == -1`, `send_joint_targets` allocates a fixed 16-byte buffer and writes 16 bytes starting at byte 42 (`j1_target`). Range: bytes 42 to 57.
  Bytes 54–57 (`theta_j2_ht` in PLC DB5) are **unconditionally overwritten** with `j4` (float 0.0), corrupting PLC shoulder feedback memory on every target send.

### 1.4 Requirement R3: Joint Limits Fragmentation & Missing IK Enforcement
Direct inspection of `src/kinematics/kinematics.py`, `src/config_loader.py`, and `src/ui/page_manual.py`:
- **Hardcoded joint limits in `src/kinematics/kinematics.py`** (lines 65–70):
  ```python
  JOINT_LIMITS: dict[str, tuple[float, float]] = {
      "j1": (-180.0, 180.0),
      "j2": (0.0, 250.0),
      "j3": (0.0, 200.0),
      "j4": (-180.0, 180.0),
  }
  ```
- **`inverse_kinematics()` does NOT enforce joint limits** (lines 258–337):
  `_inverse_kinematics_impl()` calculates `(theta1, theta2, theta3, theta4)` using trigonometry and returns `(math.degrees(theta1), math.degrees(theta2), math.degrees(theta3), math.degrees(theta4))` without verifying any angle against `JOINT_LIMITS`. If a reachable geometric point yields `θ₁ = 195°` or `θ₂ = 260°`, it returns the violating angle tuple without raising `WorkspaceError`.
- **Existing joint validation in `PageManual`** (`src/ui/page_manual.py` lines 848–863):
  Uses `low, high = JOINT_LIMITS[key]` in `_validate_joints()`, importing `JOINT_LIMITS` from `src.kinematics`.
- **Elbow Down vs Limits in existing tests**:
  In `tests/test_robot_system.py`:
  - TC-19 (`test_elbow_up_vs_down_different_solutions`, line 1150): tests `elbow="down"` returning negative `θ₃`.
  - TC-20 (`test_fk_ik_round_trip_elbow_down`, line 1168): tests `j3 = -20.0` with `elbow="down"`.
  When `elbow="down"` is used, `θ₃` is mathematically negative. In `config.yaml`, `j3_min: 0.0`.

### 1.5 Requirement R5: Thread Churn in `send_pulse()`
Direct inspection of `src/plc/plc_controller.py`:
- **Transient `threading.Timer` creation** (lines 359–372):
  ```python
  self.write_bit(byte_offset, bit_offset, True)

  timer = threading.Timer(
      self.PULSE_WIDTH_S,
      self._finish_pulse,
      args=(byte_offset, bit_offset, generation),
  )
  timer.daemon = True
  with self._lock:
      if self._pulse_generations.get(key) == generation:
          self._pulse_timers[key] = (timer, generation)
  timer.start()
  ```
  Every single pulse creates and starts a new OS thread. Under rapid button presses, JOG toggles, or automated cycles, dozens of threads are spawned and destroyed, causing thread overhead and lock contention.
- **Cancellation limitations**:
  `timer.cancel()` only prevents the callback if it hasn't yet started. Once `_finish_pulse` enters, it proceeds. Furthermore, there is no public method to cancel active pulses on JOG button release or during emergency stop without completely disconnecting the PLC client (`disconnect()`).
- **Existing unit tests**:
  `tests/test_robot_system.py` lines 850–879 directly access `ctrl._pulse_timers` and `ctrl._finish_pulse(0, 0, generation)`.

---

## 2. Logic Chain

### 2.1 Logic Chain for Requirement R1 (PC Master Architecture)
1. *From Observation 1.2*: Currently, `execute_sort()` pulses `START_AUTO` at line 197 and then proceeds to execute its own python waypoint loop (`_pick`, `_place`, `_return_home`).
2. *Deduction 1*: If the PLC ladder logic interprets `START_AUTO` as triggering an autonomous hardware sequence, running both the PLC autonomous cycle and PC waypoint commands concurrently produces a race condition where the PC and PLC fight for motion control.
3. *From Observation 1.2*: In `_move_and_wait()`, `send_joint_targets()` is never called, and `send_command()` is called only if `cmd is not None`.
4. *Deduction 2*: The PC Master model requires the PC to be the authoritative sequencer. For every waypoint (`pick_up`, `pick_down`, `place_up`, `place_down`, `home`), the PC must:
   - Compute IK targets `(j1, j2, j3, j4)`.
   - Write targets to PLC via `self.plc.send_joint_targets(j1, j2, j3, j4)`.
   - Command the move via `self.plc.send_command(self.plc_commands.move)` (or `self.plc_commands.home` for homing).
5. *From Observation 1.2*: When the timeout expires in `_move_and_wait()`, it logs debug and breaks, allowing subsequent gripper actuation without confirming target arrival.
6. *Deduction 3*: In PC Master mode, motion supervision must be deterministic. The polling loop must check `motion_done` from `self.plc.read_status()`. If `time.monotonic() >= deadline` without `motion_done`, or if `error_flag` is set, or if PLC disconnects, an explicit `RuntimeError` must be raised. Gripper toggling (`write_bit(*ADDR.GRIP, True/False)`) must only occur after `motion_done` is asserted.
7. *Deduction 4*: To preserve the legacy autonomous mode for optional future use, the old sequence must be cleanly isolated into `execute_sort_plc_sequence(self, pick_x, pick_y, result)`.

### 2.2 Logic Chain for Requirement R2 (Dynamic Target Buffer & Offset Safety)
1. *From Observation 1.3*: In `config.yaml`, `j4_target` is `-1` to represent "Not mapped in PLC Image".
2. *From Observation 1.3*: In the PLC DB memory map, `j1_target` is at byte 42, `j2_target` is at byte 46, `j3_target` is at byte 50. Bytes 54–57 store `theta_j2_ht` (actual shoulder feedback from PLC).
3. *From Observation 1.3*: `send_joint_targets()` allocates `bytearray(16)` and calls `db_write(db_number, 42, buf)`. The 16-byte write covers byte offsets 42 to 57.
4. *Deduction 1*: When `j4_target == -1`, bytes 12..15 of the buffer correspond to DB offsets 54..57. Overwriting byte 54..57 with `j4` corrupts `theta_j2_ht` in PLC memory.
5. *Deduction 2*: When `j4_target == -1` (or `< 0`), `send_joint_targets()` must dynamically assemble a 12-byte payload (`bytearray(12)`) containing only `j1`, `j2`, and `j3` (3 × 4-byte REAL).
6. *Deduction 3*: Writing exactly 12 bytes starting at offset 42 writes bytes 42..53 only. Byte 54 and above remain completely untouched, ensuring total memory safety for the PLC Data Block.

### 2.3 Logic Chain for Requirement R3 (Centralized Joint Limits & WorkspaceError)
1. *From Observation 1.4*: `JOINT_LIMITS` is hardcoded in `src/kinematics/kinematics.py:65`, while `config.yaml` contains `kinematics.j1_min..j4_max`.
2. *From Observation 1.4*: `inverse_kinematics()` currently performs geometric reachability checks (r_total, spherical distance) but never checks whether computed angles exceed hardware limits.
3. *Deduction 1*: `JOINT_LIMITS` should be centralized in `src/kinematics/joint_limits.py`, providing helper functions:
   - `get_joint_limits() -> dict[str, tuple[float, float]]`
   - `set_joint_limits(limits: dict[str, tuple[float, float]]) -> None`
   - `validate_joint_angles(j1: float, j2: float, j3: float, j4: float, elbow: str = "up") -> None`
   Re-exporting `JOINT_LIMITS` in `src/kinematics/__init__.py` maintains 100% backward compatibility with `src/ui/page_manual.py`.
4. *Deduction 2*: `inverse_kinematics()` must call `validate_joint_angles()`. If any computed joint angle is outside its valid range, raise `WorkspaceError`. Because `WorkspaceError` inherits from `InverseKinematicsError`, any existing caller handling `InverseKinematicsError` handles it naturally.
5. *Deduction 3 (Elbow Down handling)*: As shown in Observation 1.4, TC-19 and TC-20 specifically test `elbow="down"` where `θ₃` is negative. Physical joint travel for elbow-down bends inversely. Therefore, `validate_joint_angles` must account for `elbow="down"` by validating `|θ₃| <= j3_max` (or `[-j3_max, -j3_min]`) so that existing elbow-down test cases pass while genuinely excessive angles (e.g. `θ₁ > 180°`, `θ₂ > 250°`, `|θ₃| > 200°`) are rejected with `WorkspaceError`.
6. *Deduction 4*: In `SortingController._ik()`, catching `WorkspaceError` aborts the sort cycle immediately, sets `_state = RobotState.ERROR`, clears classification, and bubbles the error up to `main.py` where `self.log_tx()` surfaces it to the GUI without commanding motion.

### 2.4 Logic Chain for Requirement R5 (Pulse Thread Churn & Dedicated Queue)
1. *From Observation 1.5*: Spawning a new `threading.Timer` on every `send_pulse` invocation causes thread churn, high thread lifecycle overhead on Windows, and potential lock starvation.
2. *Deduction 1*: A dedicated single background daemon worker thread (`_pulse_worker`) using a condition variable (`threading.Condition(self._lock)`) completely eliminates thread churn.
3. *Deduction 2*: The worker maintains a dictionary of active pulses:
   `self._pending_pulses[key] = (deadline, generation)`
   When `send_pulse(byte, bit)` is called:
   - Immediately write `True` (rising edge).
   - Record `deadline = time.monotonic() + self.PULSE_WIDTH_S` and bump generation.
   - Notify the condition variable.
4. *Deduction 3*: The worker sleeps on the condition variable until the earliest deadline. When a deadline expires, it writes `False` (falling edge) and removes the pulse entry.
5. *Deduction 4*: Clean cancellation is achieved via `cancel_pulse(byte, bit)` and `cancel_all_pulses()`:
   - Immediately removes pending pulse(s) and writes `False`.
   - Used during PLC disconnect, emergency stop, or JOG button release.
6. *Deduction 5 (Test Compatibility)*: `tests/test_robot_system.py` directly checks `ctrl._pulse_timers` and calls `ctrl._finish_pulse(0, 0, generation)`. Retaining an internal shim/property for `_pulse_timers` and retaining `_finish_pulse` ensures existing unit tests remain green.

---

## 3. Caveats

1. **Physical PLC vs. Mock Environment**:
   Hardware tests with a live physical S7-1200 PLC require verifying whether the PLC ladder program has separate command inputs for `commands.move` (cmd 2) vs `START_AUTO`. In the current software, both map to command word 2 / bit 0.0 (`ADDR.START_AUTO`). In PC Master mode, the PC commands each waypoint individually via `send_command(cfg.plc.commands.move)`, requiring the PLC to execute a single point-to-point move and set `motion_done = True` upon arrival.
2. **Motion Duration Estimation in Tests**:
   `SortingController._estimate_motion_seconds()` calculates travel time from the maximum joint delta divided by `_JOINT_SPEED_DPS` (45 deg/s). In unit tests where `mock_plc` is used, `mock_plc.read_status()` must return `motion_done=True` to prevent the supervision loop from waiting the full timeout.
3. **Elbow-Down Mechanical Clearance**:
   While mathematical IK supports `elbow="down"`, user hardware confirmation is needed on whether the physical robot arm geometry physically permits elbow-down motion without collision with the base or workpiece table.

---

## 4. Conclusion & Concrete Recommendations

### 4.1 Recommendation for R1 (PC Master in `SortingController`)
Refactor `src/robot/sorting_controller.py`:
1. **Remove premature pulse**: Delete `self.plc.send_pulse(*ADDR.START_AUTO)` from `execute_sort()`.
2. **Update `_move_and_wait(self, j1, j2, j3, j4, cmd=None, timeout=30.0)`**:
   - Send joint targets: `self.plc.send_joint_targets(j1, j2, j3, j4)`.
   - Issue move command: `effective_cmd = self.plc_commands.move if cmd is None else cmd; self.plc.send_command(effective_cmd)`.
   - In supervision loop:
     - Check `if not status: raise RuntimeError("PLC offline during active pick-and-place motion.")`.
     - Check `if status.get("error_flag", False): raise RuntimeError("PLC error flag activated during active motion.")`.
     - Check `if status.get("motion_done", False): break`.
     - Check `if time.monotonic() >= deadline: raise RuntimeError(f"Motion timeout ({min(timeout, max_wait):.1f}s) waiting for target {target} arrival.")`.
3. **Gripper actuation**: Maintain `self.plc.write_bit(*ADDR.GRIP, True/False)` strictly after `_move_and_wait()` returns.
4. **Isolate PLC Autonomous sequence**: Add `execute_sort_plc_sequence(self, pick_x: float, pick_y: float, result: SortResult) -> None` containing the autonomous classification pulse and sequence polling.

### 4.2 Recommendation for R2 (Dynamic Target Buffer in `PLCController`)
Refactor `PLCController.send_joint_targets()` in `src/plc/plc_controller.py`:
1. Check `self._cfg.offsets.j4_target`:
   - If `self._cfg.offsets.j4_target >= 0`:
     - Allocate 16 bytes (`bytearray(16)`).
     - Write `j1` (offset 0), `j2` (offset 4), `j3` (offset 8), `j4` (offset 12).
   - If `self._cfg.offsets.j4_target < 0` (e.g. `-1`):
     - Allocate 12 bytes (`bytearray(12)`).
     - Write `j1` (offset 0), `j2` (offset 4), `j3` (offset 8).
2. Write buffer to `self._cfg.offsets.j1_target`:
   - `self._client.db_write(self._cfg.db_number, self._cfg.offsets.j1_target, buf)`
   - Buffer length is 12 bytes when unmapped, perfectly protecting bytes 54–57 (`theta_j2_ht`).

### 4.3 Recommendation for R3 (Centralized Limits & IK Validation)
1. **Create `src/kinematics/joint_limits.py`**:
   - Holds default limits dict matching `config.yaml`:
     ```python
     JOINT_LIMITS: dict[str, tuple[float, float]] = {
         "j1": (-180.0, 180.0),
         "j2": (0.0, 250.0),
         "j3": (0.0, 200.0),
         "j4": (-180.0, 180.0),
     }
     ```
   - Functions `set_joint_limits()`, `get_joint_limits()`, and `validate_joint_angles(j1, j2, j3, j4, elbow="up")`.
   - Re-export in `src/kinematics/__init__.py`.
2. **In `src/kinematics/kinematics.py:inverse_kinematics()`**:
   - Call `validate_joint_angles(theta1, theta2, theta3, theta4, elbow=elbow)`.
   - If limits exceeded, raise `WorkspaceError(f"Calculated joint angle θ={val:.2f}° exceeds hardware limits [{low:g}°, {high:g}°]")`.
3. **In `SortingController._ik()`**:
   - Catches `WorkspaceError`, logs diagnostic details, and re-raises.
   - `execute_sort()` transitions state to `RobotState.ERROR` and clears classification without commanding motion.

### 4.4 Recommendation for R5 (Worker Queue in `PLCController`)
Refactor pulse handling in `src/plc/plc_controller.py`:
1. Replace per-pulse `threading.Timer` with a single daemon worker thread `_pulse_worker` using a `threading.Condition(self._lock)`.
2. `send_pulse(byte, bit)` writes `True`, records `(deadline, generation)` in `self._pending_pulses`, and wakes worker via condition variable notify.
3. Worker sleeps on condition variable with timeout matching the nearest deadline, writes `False`, and removes expired pulse.
4. Add `cancel_pulse(byte, bit)` and `cancel_all_pulses()` for clean cancellation on JOG release and PLC disconnect.
5. Retain `_pulse_timers` structure / property shim and `_finish_pulse` method to ensure existing tests pass without modification.

---

## 5. Verification Method

To independently verify these findings and future implementations:

1. **Automated Unit Tests**:
   Run the project test suite via powershell:
   ```powershell
   pytest tests/ --cov=src
   ```
   *Baseline*: All 82 existing unit tests pass.

2. **R1 Verification**:
   - Inspect `src/robot/sorting_controller.py`.
   - Assert in test that `execute_sort()` does not call `send_pulse(*ADDR.START_AUTO)`.
   - Assert in test that `_move_and_wait()` invokes `send_joint_targets(j1, j2, j3, j4)` and `send_command(cfg.plc.commands.move)`.
   - Assert in test that `_move_and_wait()` raises `RuntimeError` if `motion_done` is never received before timeout.
   - Assert in test that `write_bit(*ADDR.GRIP, True)` occurs only after `motion_done` is received.

3. **R2 Verification**:
   - In `tests/test_robot_system.py`, configure `offsets.j4_target = -1`.
   - Call `ctrl.send_joint_targets(10.0, 20.0, 30.0, 0.0)`.
   - Inspect mock `db_write` arguments:
     - Verify `offset == 42`.
     - Verify `len(buf) == 12`.
     - Verify byte 54+ is never written.

4. **R3 Verification**:
   - In `tests/test_robot_system.py`, call `inverse_kinematics()` for a pose that produces joint angles exceeding configured limits (e.g. `j1 > 180°` or `j2 > 250°`).
   - Verify `WorkspaceError` is raised.
   - Verify `SortingController.execute_sort()` catches the error, transitions to `RobotState.ERROR`, and does not call `_move_and_wait()` or gripper actuation.

5. **R5 Verification**:
   - In `tests/test_robot_system.py`, call `send_pulse(0, 0)` multiple times rapidly.
   - Verify `threading.active_count()` does not increase with each pulse.
   - Verify `cancel_pulse(0, 0)` immediately resets bit and cancels pending reset without leaving orphan threads.
