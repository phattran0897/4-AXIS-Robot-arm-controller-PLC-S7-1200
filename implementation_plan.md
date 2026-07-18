# Adding Sorter Error Clearing Logic

This plan outlines the changes to allow resetting the `SortingController` state back to `IDLE` when the "Clear Error" button is clicked in the GUI. Currently, when the robot encounters a movement timeout or PLC offline error, `SortingController` gets permanently stuck in `RobotState.ERROR` and ignores all subsequent detections.

## Proposed Changes

### 1. Sorter Controller
#### [MODIFY] [sorting_controller.py](file:///d:/New%20folder/1234/src/robot/sorting_controller.py)
- Implement `clear_error` method on the `SortingController` class to reset `self._state` to `RobotState.IDLE`.

### 2. Main Application
#### [MODIFY] [main.py](file:///d:/New%20folder/1234/main.py)
- Expose `clear_all_errors` on `RobotApp` which calls both `plc.send_command(idle)` and `_sorter.clear_error()`.

### 3. Base Page
#### [MODIFY] [base_page.py](file:///d:/New%20folder/1234/src/ui/base_page.py)
- Change the command of the "Clear Error" button to trigger the new `self.controller.clear_all_errors` method.

---

## Verification Plan

### Automated Tests
Run pytest to ensure compile and test execution stability:
```bash
python -m pytest tests/ -v
```

### Manual Verification
1. Click the "Clear Error" button on the GUI during an error state.
2. Confirm the GUI operation label changes back to "● IDLE" (meaning `SortingController` is ready again).
3. Verify that subsequent items are once again detected and sorted after clearing the error.
