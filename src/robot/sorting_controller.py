"""
src/robot/sorting_controller.py – Automated pick-and-place sorting controller.
"""

from __future__ import annotations

import logging
import math
import threading
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.config_loader import KinematicsConfig, PLCCommands, SortPositionsConfig
    from src.plc.plc_controller import PLCController

log = logging.getLogger(__name__)


class SortResult(Enum):
    """Result of defect detection for sorting classification."""

    GOOD = "GOOD"
    BAD = "BAD"


class RobotState(Enum):
    """FSM states for the pick-and-place sorting process."""

    IDLE = "IDLE"
    MOVING_PICK = "MOVING_PICK"
    GRIPPING = "GRIPPING"
    MOVING_PLACE = "MOVING_PLACE"
    RELEASING = "RELEASING"
    RETURNING = "RETURNING"
    ERROR = "ERROR"


class SortingController:
    """Controls the automated pick-and-place defect sorting cycle."""

    def __init__(
        self,
        plc: PLCController,
        positions: SortPositionsConfig,
        kinematics: KinematicsConfig,
        plc_commands: PLCCommands,
    ) -> None:
        """Initialise SortingController with required hardware clients and configs."""
        self.plc: PLCController = plc
        self.positions: SortPositionsConfig = positions
        self.kinematics: KinematicsConfig = kinematics
        self.plc_commands: PLCCommands = plc_commands

        self._state: RobotState = RobotState.IDLE
        self._counter_good: int = 0
        self._counter_bad: int = 0
        self._last_sort_result: SortResult | None = None

        # Synchronization event used for precise, mockable delays (avoiding bare time.sleep)
        self._wait_event: threading.Event = threading.Event()

    @property
    def state(self) -> RobotState:
        """Get current FSM state of the sorting robot."""
        return self._state

    @property
    def counter_good(self) -> int:
        """Get count of good items processed."""
        return self._counter_good

    @property
    def counter_bad(self) -> int:
        """Get count of bad items processed."""
        return self._counter_bad

    def is_idle(self) -> bool:
        """Check if robot state is IDLE (ready for next task)."""
        return self._state == RobotState.IDLE

    @property
    def last_sort_result(self) -> SortResult | None:
        """Get the classification result of the most recent sort cycle."""
        return self._last_sort_result

    def reset_counters(self) -> None:
        """Reset good and bad sorting statistics counters."""
        self._counter_good = 0
        self._counter_bad = 0
        log.info("Sorting counters reset to zero.")

    def _ik(self, x: float, y: float, z: float) -> tuple[float, float, float, float]:
        """
        Compute 4-DOF articulated-arm inverse kinematics.
        """
        from src.kinematics import inverse_kinematics, InverseKinematicsError
        try:
            return inverse_kinematics(x, y, z, phi=self.kinematics.default_phi)
        except InverseKinematicsError as exc:
            log.warning("IK failed for target (%.2f, %.2f, %.2f): %s", x, y, z, exc)
            return 0.0, 0.0, 0.0, 0.0

    def execute_sort(
        self, pick_x: float, pick_y: float, result: SortResult
    ) -> None:
        """Execute a complete pick-and-place sorting cycle."""
        log.info(
            "Starting sort cycle: Pick=(%.2f, %.2f), Result=%s",
            pick_x,
            pick_y,
            result.name,
        )

        try:
            # Store the classification for GUI display
            self._last_sort_result = result

            # 0. Write classification to PLC
            self.plc.write_classification(result == SortResult.GOOD)

            # 1. Pick
            self._state = RobotState.MOVING_PICK
            self._pick(pick_x, pick_y)

            # 2. Place
            self._state = RobotState.MOVING_PLACE
            self._place(result)

            # 3. Return Home
            self._state = RobotState.RETURNING
            self._return_home()

            # Increment Statistics
            if result == SortResult.GOOD:
                self._counter_good += 1
            else:
                self._counter_bad += 1

            # 4. Clear classification bits after cycle complete
            self.plc.clear_classification()

            log.info("Sort cycle completed successfully.")

        except Exception as exc:
            self._state = RobotState.ERROR
            self.plc.clear_classification()
            log.error("Error during sorting cycle: %s", exc)
            raise exc

    def _pick(self, x: float, y: float) -> None:
        """Move to coordinates, lower Z, grab, and raise Z."""
        # Calculate joints for z_up
        j1_up, j2_up, j3_up, j4_up = self._ik(x, y, self.positions.pick_z_up)
        # Calculate joints for z_down
        j1_down, j2_down, j3_down, j4_down = self._ik(x, y, self.positions.pick_z_down)

        log.info("Picking target (%.2f, %.2f) -> Up: [%.2f, %.2f, %.2f, %.2f], Down: [%.2f, %.2f, %.2f, %.2f]",
                 x, y, j1_up, j2_up, j3_up, j4_up, j1_down, j2_down, j3_down, j4_down)

        # Move to XY at z_up
        self._move_and_wait(j1_up, j2_up, j3_up, j4_up)

        # Lower to z_down
        self._move_and_wait(j1_down, j2_down, j3_down, j4_down)

        # Actuate gripper close
        self._state = RobotState.GRIPPING
        log.info("Closing gripper (GRIP = True)")
        self.plc.write_bit(14, 0, True)  # Write True to AUTO.GRIP (Offset 14.0)
        self._wait_event.wait(self.positions.gripper_delay)

        # Raise back to z_up
        self._state = RobotState.MOVING_PICK
        self._move_and_wait(j1_up, j2_up, j3_up, j4_up)

    def _place(self, result: SortResult) -> None:
        """Move to designated bin, lower Z, release target, and raise Z."""
        target = (
            self.positions.place_good
            if result == SortResult.GOOD
            else self.positions.place_bad
        )
        # Calculate joints for z_up
        j1_up, j2_up, j3_up, j4_up = self._ik(target.x, target.y, target.z_up)
        # Calculate joints for z_down
        j1_down, j2_down, j3_down, j4_down = self._ik(target.x, target.y, target.z_down)

        log.info("Placing target to %s -> Up: [%.2f, %.2f, %.2f, %.2f], Down: [%.2f, %.2f, %.2f, %.2f]",
                 result.name, j1_up, j2_up, j3_up, j4_up, j1_down, j2_down, j3_down, j4_down)

        # Move to placement XY at z_up
        self._move_and_wait(j1_up, j2_up, j3_up, j4_up)

        # Lower to place z_down
        self._move_and_wait(j1_down, j2_down, j3_down, j4_down)

        # Actuate gripper open
        self._state = RobotState.RELEASING
        log.info("Opening gripper (GRIP = False)")
        self.plc.write_bit(14, 0, False)  # Write False to AUTO.GRIP (Offset 14.0)
        self._wait_event.wait(self.positions.gripper_delay)

        # Raise back to z_up
        self._state = RobotState.MOVING_PLACE
        self._move_and_wait(j1_up, j2_up, j3_up, j4_up)

    def _return_home(self) -> None:
        """Bring all joints back to home coordinates and idle state."""
        log.info("Returning to home (0,0,0,0)")
        self._move_and_wait(0.0, 0.0, 0.0, 0.0, cmd=self.plc_commands.home)
        self._state = RobotState.IDLE

    def _move_and_wait(
        self,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        cmd: int | None = None,
        timeout: float = 10.0,
    ) -> None:
        """Command joint motion and wait until completed or timed out."""
        target_cmd = self.plc_commands.move if cmd is None else cmd
        self.plc.send_joint_targets_and_command(j1, j2, j3, j4, target_cmd)

        import time
        t_start = time.monotonic()

        while True:
            if time.monotonic() - t_start > timeout:
                raise TimeoutError(
                    f"Motion timeout ({timeout}s) exceeded during move."
                )

            status = self.plc.read_status()
            if not status:
                # If PLC disconnects during cycle, abort immediately
                raise RuntimeError(
                    "PLC offline during active pick-and-place motion."
                )

            if status.get("error_flag", False):
                raise RuntimeError(
                    "PLC error flag activated during active motion."
                )

            if status.get("motion_done", False):
                break

            self._wait_event.wait(0.05)
