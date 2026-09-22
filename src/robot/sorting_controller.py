"""
src/robot/sorting_controller.py – Automated pick-and-place sorting controller.

Contract
--------
Implements the PC Master motion model:
1. Writes classification result (GOOD/BAD) to the PLC Data Block.
2. Computes IK targets for each waypoint (approach, pick, retreat, bin, place).
3. Transmits joint targets to the PLC (via plc.send_joint_targets).
4. Supervise motion by polling motion_done bit with timeout.
5. Actuates gripper (GRIP) strictly after target arrival is confirmed.
6. Returns robot to home pose and clears classification.

Alternatively, execute_sort_plc_sequence() is available if the PLC is
configured with an autonomous internal sequencer.
"""

from __future__ import annotations

import logging
import threading
import time
from enum import Enum
from typing import TYPE_CHECKING

from src.kinematics import InverseKinematicsError, inverse_kinematics, validate_joint_angles
from src.plc.plc_controller import ADDR

if TYPE_CHECKING:
    from src.config_loader import KinematicsConfig, PLCCommands, SortPositionsConfig
    from src.plc.plc_controller import PLCController

log = logging.getLogger(__name__)

#: Assumed worst-case joint speed used to estimate travel times (deg/s).
_JOINT_SPEED_DPS: float = 45.0
#: Extra seconds added on top of the estimated travel time before giving up.
_MOTION_GRACE_S: float = 1.0


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
        self._last_qr_value: str | None = None

        # Commanded joint angles mirrored to the GUI (NOT the motion feedback)
        self._current_joints: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
        # Last waypoint used as the reference for travel-time estimation
        self._previous_joints: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

        # Synchronization event used for precise, mockable delays (avoiding bare time.sleep)
        self._wait_event: threading.Event = threading.Event()

        self._validate_waypoints()

    def _validate_waypoints(self) -> None:
        """
        Fail fast when a fixed place waypoint lies outside the reachable
        workspace (config typo, wrong DH parameters, …).

        Dynamic pick coordinates from the vision system are validated at
        cycle time by :meth:`_ik` raising on unreachable targets.
        """
        phi = self.kinematics.default_phi
        unreachable: list[str] = []
        for name, pos in (
            ("place_good", self.positions.place_good),
            ("place_bad", self.positions.place_bad),
        ):
            for label, z in (("z_up", pos.z_up), ("z_down", pos.z_down)):
                try:
                    inverse_kinematics(pos.x, pos.y, z, phi=phi)
                except InverseKinematicsError:
                    unreachable.append(f"{name}.{label} (x={pos.x}, y={pos.y}, z={z})")

        # Validate QR presets (Joint targets)
        if self.positions.pick is not None:
            try:
                p = self.positions.pick
                validate_joint_angles(p.j1, p.j2, p.j3, p.j4)
            except InverseKinematicsError as exc:
                unreachable.append(f"pick ({exc})")
        
        for name, loc in self.positions.locations.items():
            try:
                validate_joint_angles(loc.j1, loc.j2, loc.j3, loc.j4)
            except InverseKinematicsError as exc:
                unreachable.append(f"locations[{name}] ({exc})")

        if unreachable:
            raise ValueError(
                "Sort waypoints outside the reachable workspace – check "
                "config.yaml and kinematics DH parameters: " + "; ".join(unreachable)
            )

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
    def current_joints(self) -> tuple[float, float, float, float]:
        """Commanded joint angles computed by IK (for GUI display only)."""
        return self._current_joints

    @property
    def last_sort_result(self) -> SortResult | None:
        """Get the classification result of the most recent sort cycle."""
        return self._last_sort_result

    def reset_counters(self) -> None:
        """Reset good and bad sorting statistics counters."""
        self._counter_good = 0
        self._counter_bad = 0
        log.info("Sorting counters reset to zero.")

    def clear_error(self) -> None:
        """Reset the internal controller state to IDLE after an error has been resolved."""
        self._state = RobotState.IDLE
        self._last_sort_result = None
        self._last_qr_value = None
        # Unblock any _wait_event.wait() calls so the sort thread can exit
        self._wait_event.set()
        self._wait_event.clear()
        log.info("SortingController error state cleared; reset to IDLE.")

    def _ik(self, x: float, y: float, z: float) -> tuple[float, float, float, float]:
        """
        Compute 4-DOF articulated-arm inverse kinematics.

        Raises
        ------
        InverseKinematicsError
            When the target is unreachable.  Aborting the cycle is the only
            safe response – substituting a fallback pose such as
            ``(0, 0, 0, 0)`` would command *real* motion to full horizontal
            extension while the cycle logic continues as if nothing happened.
        """
        try:
            return inverse_kinematics(x, y, z, phi=self.kinematics.default_phi)
        except InverseKinematicsError as exc:
            log.error("IK failed for target (%.2f, %.2f, %.2f): %s", x, y, z, exc)
            raise

    def process_qr_target(self, qr_value: str) -> bool:
        """
        Process a decoded QR code in autonomous PLC mode.
        Does not wait for motion to finish. Only writes to DB if it's a new QR code.
        
        Returns:
            True if the target was written successfully, False if skipped or error.
        """
        if qr_value == "CLEAR":
            self._last_qr_value = None
            return False

        if qr_value == self._last_qr_value:
            # Already sent this one, wait until a new box comes along
            return False

        if qr_value not in self.positions.locations:
            log.warning("Decoded QR '%s' but it is not configured in locations map.", qr_value)
            # Update last_qr_value so we don't spam the warning
            self._last_qr_value = qr_value
            return False

        loc = self.positions.locations[qr_value]
        
        log.info(
            "QR Target Valid: [%s] -> Writing targets: J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f",
            qr_value, loc.j1, loc.j2, loc.j3, loc.j4
        )
        
        try:
            # db_write directly to joint targets in DB
            self.plc.send_joint_targets(loc.j1, loc.j2, loc.j3, loc.j4)
            self._last_qr_value = qr_value
            self._current_joints = (loc.j1, loc.j2, loc.j3, loc.j4)
            self._counter_good += 1  # Or generic count
            return True
        except Exception as exc:
            log.error("Failed to write QR targets to PLC: %s", exc)
            return False

    def execute_sort(self, pick_x: float, pick_y: float, result: SortResult) -> None:
        """
        Execute a complete pick-and-place sorting cycle under PC Master control.

        Transmits joint targets for each waypoint sequentially, verifies motion_done
        before each gripper action, and never triggers an uncoordinated PLC sequence.
        """
        log.info(
            "Starting PC Master sort cycle: Pick=(%.2f, %.2f), Result=%s",
            pick_x,
            pick_y,
            result.name,
        )

        try:
            # Store the classification for GUI display
            self._last_sort_result = result

            # 0. Write classification to PLC (atomic bit-group write)
            self.plc.write_classification(result == SortResult.GOOD)

            # 1. Pick (approach -> lower -> close gripper -> retreat)
            self._state = RobotState.MOVING_PICK
            self._pick(pick_x, pick_y)

            # 2. Place (approach bin -> lower -> open gripper -> retreat)
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

            log.info("PC Master sort cycle completed successfully.")

        except Exception as exc:
            self._state = RobotState.ERROR
            self.plc.clear_classification()
            log.error("Error during sorting cycle: %s", exc)
            raise

    def execute_sort_plc_sequence(
        self, pick_x: float, pick_y: float, result: SortResult
    ) -> None:
        """
        Alternative sort execution for PLC autonomous sequence mode.
        Isolated from PC Master mode to prevent race conditions.
        """
        log.info(
            "Starting PLC sequence sort cycle: Pick=(%.2f, %.2f), Result=%s",
            pick_x,
            pick_y,
            result.name,
        )
        try:
            self._last_sort_result = result
            self.plc.write_classification(result == SortResult.GOOD)
            self.plc.send_pulse(*ADDR.START_AUTO)
            self._state = RobotState.MOVING_PICK

            # Wait for PLC sequence to complete
            deadline = time.monotonic() + 60.0
            while True:
                status = self.plc.read_status()
                if not status or status.get("error_flag", False):
                    raise RuntimeError("PLC error during autonomous sequence.")
                if status.get("motion_done", False):
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError("PLC autonomous sequence timed out.")
                self._wait_event.wait(0.1)

            if result == SortResult.GOOD:
                self._counter_good += 1
            else:
                self._counter_bad += 1
            self.plc.clear_classification()
            self._state = RobotState.IDLE
        except Exception as exc:
            self._state = RobotState.ERROR
            self.plc.clear_classification()
            log.error("PLC sequence cycle failed: %s", exc)
            raise

    def _pick(self, x: float, y: float) -> None:
        """Mirror pick waypoints, monitor PLC health, actuate the gripper."""
        j_up = self._ik(x, y, self.positions.pick_z_up)
        j_down = self._ik(x, y, self.positions.pick_z_down)

        log.info(
            "Picking target (%.2f, %.2f) -> Up: [%s], Down: [%s]",
            x,
            y,
            ", ".join(f"{a:.2f}" for a in j_up),
            ", ".join(f"{a:.2f}" for a in j_down),
        )

        # Move to pick XY at z_up, then linearly interpolate down to z_down
        self._move_and_wait(*j_up)
        self._move_l_and_wait(
            x, y, self.positions.pick_z_up,
            x, y, self.positions.pick_z_down,
            steps=5
        )

        # Actuate gripper close
        self._state = RobotState.GRIPPING
        log.info("Closing gripper (GRIP = True)")
        self.plc.write_bit(*ADDR.GRIP, True)
        self._wait_event.wait(self.positions.gripper_delay)

        # Raise back to z_up linearly
        self._state = RobotState.MOVING_PICK
        self._move_l_and_wait(
            x, y, self.positions.pick_z_down,
            x, y, self.positions.pick_z_up,
            steps=5
        )

    def _place(self, result: SortResult) -> None:
        """Move to designated bin, lower Z, release target, and raise Z."""
        target = (
            self.positions.place_good
            if result == SortResult.GOOD
            else self.positions.place_bad
        )
        j_up = self._ik(target.x, target.y, target.z_up)
        j_down = self._ik(target.x, target.y, target.z_down)

        log.info(
            "Placing target to %s -> Up: [%s], Down: [%s]",
            result.name,
            ", ".join(f"{a:.2f}" for a in j_up),
            ", ".join(f"{a:.2f}" for a in j_down),
        )

        # Move to placement XY at z_up, then linearly interpolate down to z_down
        self._move_and_wait(*j_up)
        self._move_l_and_wait(
            target.x, target.y, target.z_up,
            target.x, target.y, target.z_down,
            steps=5
        )

        # Actuate gripper open
        self._state = RobotState.RELEASING
        log.info("Opening gripper (GRIP = False)")
        self.plc.write_bit(*ADDR.GRIP, False)
        self._wait_event.wait(self.positions.gripper_delay)

        # Raise back to z_up linearly
        self._state = RobotState.MOVING_PLACE
        self._move_l_and_wait(
            target.x, target.y, target.z_down,
            target.x, target.y, target.z_up,
            steps=5
        )

    def _return_home(self) -> None:
        """Bring all joints back to home coordinates and idle state."""
        log.info("Returning to home (0,0,0,0)")
        self._move_and_wait(0.0, 0.0, 0.0, 0.0, cmd=self.plc_commands.home)
        self._state = RobotState.IDLE

    def _estimate_motion_seconds(
        self, target: tuple[float, float, float, float]
    ) -> float:
        """Estimate travel time from the largest joint delta (deg/s model)."""
        delta = max(abs(t - c) for t, c in zip(target, self._previous_joints))
        return max(0.5, delta / _JOINT_SPEED_DPS)

    def _move_l_and_wait(
        self,
        x0: float,
        y0: float,
        z0: float,
        x1: float,
        y1: float,
        z1: float,
        steps: int = 5,
        timeout: float = 30.0,
    ) -> None:
        """
        Cartesian linear interpolation between two points.
        Divides the path into multiple waypoints to guarantee a straight line.
        """
        log.info(
            "MoveL from (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f) in %d steps",
            x0, y0, z0, x1, y1, z1, steps
        )
        for i in range(1, steps + 1):
            t = i / float(steps)
            x_step = x0 + (x1 - x0) * t
            y_step = y0 + (y1 - y0) * t
            z_step = z0 + (z1 - z0) * t
            
            j1, j2, j3, j4 = self._ik(x_step, y_step, z_step)
            self._move_and_wait(j1, j2, j3, j4, timeout=timeout)

    def _move_and_wait(
        self,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        cmd: int | None = None,
        timeout: float = 30.0,
    ) -> None:
        """
        Command one waypoint to the PLC and wait for motion completion.

        Transmits (j1, j2, j3, j4) to the PLC via send_joint_targets(), then
        polls the status DB every 50 ms until motion_done is observed or the
        timeout expires. Aborts immediately if the PLC goes offline, raises
        its error flag, or times out.

        Parameters
        ----------
        j1..j4:
            Commanded joint angles in degrees.
        cmd:
            Optional command word to pulse (e.g. CMD_HOME).
        timeout:
            Absolute upper bound on the wait in seconds.
        """
        target = (j1, j2, j3, j4)
        self._current_joints = target
        log.info(
            "Waypoint (commanded): J1=%.2f J2=%.2f J3=%.2f J4=%.2f",
            j1,
            j2,
            j3,
            j4,
        )

        # 1. Transmit joint targets to PLC
        self.plc.send_joint_targets(j1, j2, j3, j4)

        # 2. Issue optional specific command (e.g. HOME)
        if cmd is not None:
            self.plc.send_command(cmd)

        estimated = self._estimate_motion_seconds(target)
        deadline = time.monotonic() + min(timeout, estimated + _MOTION_GRACE_S)

        # Allow brief settling time for PLC to register motion start
        self._wait_event.wait(0.05)

        motion_confirmed = False
        while True:
            status = self.plc.read_status()
            if not status:
                raise RuntimeError("PLC offline during active pick-and-place motion.")
            if status.get("error_flag", False):
                raise RuntimeError("PLC error flag activated during active motion.")
            if status.get("motion_done", False):
                motion_confirmed = True
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Motion to targets (J1={j1:.1f}°, J2={j2:.1f}°, J3={j3:.1f}°, J4={j4:.1f}°) "
                    f"timed out after {estimated + _MOTION_GRACE_S:.1f}s without motion_done signal."
                )
            self._wait_event.wait(0.05)

        self._previous_joints = target
