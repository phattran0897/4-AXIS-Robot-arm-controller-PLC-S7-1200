# Fixing PLC Handshake and Product Counter in Automatic Sort

This plan outlines the changes to resolve the issue where the robot gets stuck during/after the first sort cycle, subsequent items are not detected, and the product counter does not increment.

## Proposed Changes

### Sorter Controller

#### [MODIFY] [sorting_controller.py](file:///d:/New%20folder/1234/src/robot/sorting_controller.py)

- Modify `_move_and_wait` to introduce a robust handshake.
  - Wait `200ms` for the PLC to clear `motion_done` to `False` (acknowledging the commanded motion starts).
  - If `motion_done` remains `True` (e.g. because targets are already met or CPU/simulation mode is active), exit immediately.
  - Otherwise, loop and check status until `motion_done` becomes `True` (completes).
- This prevents race conditions where Python reads the stale `motion_done = True` from a previous step, advances instantly, floods the PLC, and triggers a timeout/fault.

---

## Verification Plan

### Automated Tests
Run the project's test suite to ensure no regressions:
```bash
python -m pytest tests/test_robot_system.py -v
```

### Manual Verification
1. Connect to the real S7-1200 PLC and run in Auto mode.
2. Put several test items under the camera.
3. Verify that:
   - Each item is picked and placed one by one.
   - The camera continues to processes subsequent items after the first one is completed.
   - The GUI product counter ("Tốt: X", "Xấu: Y") increments correctly at the end of each cycle.
   - The FSM returns to `IDLE` ready for the next cycle.
