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

        self._qr_lock = threading.RLock()
        self._state: RobotState = RobotState.IDLE
        self._counter_good: int = 0
        self._counter_bad: int = 0
        self._qr_written: int = 0
        self._qr_invalid: int = 0
        self._last_sort_result: SortResult | None = None
        self._last_qr_value: str | None = None
        # Timestamp of the last (failed) write attempt for the same QR code.
        # Used to rate-limit retries to at most once per second.
        self._last_qr_retry_time: float = 0.0

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

    @property
    def qr_written(self) -> int:
        return self._qr_written

    @property
    def qr_invalid(self) -> int:
        return self._qr_invalid

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
        with self._qr_lock:
            self._counter_good = 0
            self._counter_bad = 0
            self._qr_written = 0
            self._qr_invalid = 0
        log.info("Sorting counters reset to zero.")

    def clear_error(self) -> None:
        """Reset the internal controller state to IDLE after an error has been resolved."""
        self._state = RobotState.IDLE
        self._last_sort_result = None
        with self._qr_lock:
            self._last_qr_value = None
            self._last_qr_retry_time = 0.0
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

    def is_qr_handled(self, qr_value: str) -> bool:
        with self._qr_lock:
            return self._last_qr_value == qr_value

    def process_qr_target(self, qr_value: str) -> bool:
        with self._qr_lock:
            return self._process_qr_target(qr_value)

    def _process_qr_target(self, qr_value: str) -> bool:
        """
        Process a decoded QR code in autonomous PLC mode.

        Writes joint targets to the PLC DB via :meth:`PLCController.write_joint_targets`
        (no START_AUTO pulse – the PLC ladder triggers motion after the NPN sensor).

        Returns
        -------
        bool
            ``True`` only when the targets were successfully written to the PLC.
            ``False`` when: the code is a duplicate, not in the location table,
            or the PLC write failed (so the caller may retry).
        """
        if qr_value == "CLEAR":
            self._last_qr_value = None
            self._last_qr_retry_time = 0.0
            return False

        if qr_value == self._last_qr_value:
            # Already sent this one, wait until a new box comes along
            return False

        if qr_value not in self.positions.locations:
            log.warning("Decoded QR '%s' but it is not configured in locations map.", qr_value)
            # Update last_qr_value so we don't spam the warning for the same code
            self._last_qr_value = qr_value
            self._qr_invalid += 1  # One invalid QR per visible item.
            return False

        # Rate-limit retries: if the last attempt for a DIFFERENT code failed
        # (last_qr_value is still None) do not spam the PLC more than once/s.
        now = time.monotonic()
        if self._last_qr_retry_time > 0.0 and now - self._last_qr_retry_time < 1.0:
            return False

        loc = self.positions.locations[qr_value]

        log.info(
            "QR Target Valid: [%s] -> Writing targets: J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f",
            qr_value, loc.j1, loc.j2, loc.j3, loc.j4
        )

        # Write targets only – no START_AUTO pulse.
        # The PLC ladder reads these values and executes the pick-place sequence
        # autonomously once the NPN sensor detects a box.
        wrote = self.plc.write_joint_targets(loc.j1, loc.j2, loc.j3, loc.j4)

        if wrote:
            self._last_qr_value = qr_value
            self._last_qr_retry_time = 0.0
            self._current_joints = (loc.j1, loc.j2, loc.j3, loc.j4)
            self._qr_written += 1  # Successful destination writes, not completed moves.
            return True
        else:
            # Write failed (PLC offline / not connected).  Do NOT set
            # _last_qr_value so the next camera frame will retry the same code.
            self._last_qr_retry_time = now
            log.warning(
                "QR target write failed for [%s] – will retry after 1 s.", qr_value
            )
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
            # Reject an invalid DB layout before writing any cycle trigger bits.
            self.plc.validate_motion_layout()
            # Store the classification for GUI display
            self._last_sort_result = result

            # 0. Write classification to PLC (atomic bit-group write)
            if self.plc.write_classification(result == SortResult.GOOD) is False:
                raise RuntimeError("Failed to write classification; cycle aborted.")

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
            self.plc.validate_motion_layout()
            self._last_sort_result = result
            if self.plc.write_classification(result == SortResult.GOOD) is False:
                raise RuntimeError("Failed to write classification; cycle aborted.")
            if self.plc.send_pulse(*ADDR.START_AUTO) is False:
                raise RuntimeError("Failed to dispatch sequence command.")
            self._state = RobotState.MOVING_PICK

            # Wait for PLC sequence to complete
            deadline = time.monotonic() + 60.0
            saw_busy = False
            while True:
                status = self.plc.read_status()
                if not status or status.get("error_flag", False):
                    raise RuntimeError("PLC error during autonomous sequence.")
                if not status.get("motion_done", False):
                    saw_busy = True
                elif saw_busy:
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
        if self.plc.write_bit(*ADDR.GRIP, True) is False:
            raise RuntimeError("Failed to close gripper.")
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
        if self.plc.write_bit(*ADDR.GRIP, False) is False:
            raise RuntimeError("Failed to open gripper.")
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
        Samples a straight Cartesian path; the PLC controls interpolation
        between joint targets, so this alone cannot guarantee straight travel.
        """
        log.info(
            "MoveL from (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f) in %d steps",
            x0, y0, z0, x1, y1, z1, steps
        )
        if steps <= 0:
            raise ValueError("MoveL steps must be positive.")
        # Preflight every waypoint so a bad path fails before any motion starts.
        waypoints = []
        for i in range(1, steps + 1):
            t = i / float(steps)
            x_step = x0 + (x1 - x0) * t
            y_step = y0 + (y1 - y0) * t
            z_step = z0 + (z1 - z0) * t
            
            j1, j2, j3, j4 = self._ik(x_step, y_step, z_step)
            waypoints.append((j1, j2, j3, j4))
        for joints in waypoints:
            self._move_and_wait(*joints, timeout=timeout)

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
        self.plc.validate_motion_layout()
        if timeout <= 0:
            raise ValueError("Motion timeout must be positive.")
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
        if cmd is not None:
            # Write targets WITHOUT START_AUTO, then fire the specific command pulse
            # (e.g. MOVE_TO_HOME).  This guarantees exactly one pulse per move.
            if self.plc.write_joint_targets(j1, j2, j3, j4) is False:
                raise RuntimeError("Failed to write joint targets; command suppressed.")
            if self.plc.send_command(cmd) is False:
                raise RuntimeError("Failed to dispatch motion command.")
        else:
            # Standard vision/auto mode: write targets + pulse START_AUTO (one combined call)
            if self.plc.send_joint_targets(j1, j2, j3, j4) is False:
                raise RuntimeError("Failed to send joint targets.")

        deadline = time.monotonic() + timeout

        # Allow brief settling time for PLC to register motion start
        self._wait_event.wait(0.05)

        # PLC contract: each command must hold motion_done LOW until the PC
        # observes it, then raise it only after this waypoint is reached.
        # A stale HIGH must never authorize the next gripper action.
        saw_busy = False
        while True:
            status = self.plc.read_status()
            if not status:
                raise RuntimeError("PLC offline during active pick-and-place motion.")
            if status.get("error_flag", False):
                raise RuntimeError("PLC error flag activated during active motion.")
            if not status.get("motion_done", False):
                saw_busy = True
            elif saw_busy:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Motion to targets (J1={j1:.1f}°, J2={j2:.1f}°, J3={j3:.1f}°, J4={j4:.1f}°) "
                    f"timed out after {timeout:.1f}s without a fresh motion_done transition."
                )
            self._wait_event.wait(0.05)

        self._previous_joints = target
