"""
src/plc/plc_controller.py – Siemens S7-1200 PLC communication layer.

Wraps ``python-snap7`` to provide a clean, type-hinted API for reading
robot status and writing motion commands to a Siemens S7-1200 PLC.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable

import snap7
from snap7.util import get_bool, get_int, get_real, set_int, set_real

from src.config_loader import PLCConfig, PLCOffsets, compute_db_read_size

log = logging.getLogger(__name__)


def _compute_db_read_size(offsets: PLCOffsets) -> int:
    """Backward-compatibility alias – delegates to config_loader.compute_db_read_size."""
    from src.config_loader import compute_db_read_size as _func
    return _func(offsets)


class PLCController:
    """
    Thread-safe interface to the Siemens S7-1200 PLC.

    All public methods are safe to call from background threads; each
    method guards against a disconnected state and logs errors rather
    than raising, keeping the GUI alive during transient network faults.

    Parameters
    ----------
    cfg:
        Typed PLC configuration block produced by :func:`~src.config_loader.load_config`.
    """

    def __init__(self, cfg: PLCConfig) -> None:
        self._cfg: PLCConfig = cfg
        self._client: snap7.client.Client = snap7.client.Client()
        self._db_read_size: int = max(80, compute_db_read_size(cfg.offsets))
        self._last_read_time: float = 0.0
        self._lock: threading.RLock = threading.RLock()
        self._is_connected_cached: bool = False
        self.tx_callback: Callable[[str], None] | None = None
        
        # Simulated motion state tracking
        self._target_j1: float = 0.0
        self._target_j2: float = 0.0
        self._target_j3: float = 0.0
        self._target_j4: float = 0.0
        self._last_motion_start_time: float = 0.0
        self._motion_duration: float = 0.0
        
        log.info(
            "PLCController created – target %s rack=%d slot=%d db=%d read_size=%d",
            cfg.ip,
            cfg.rack,
            cfg.slot,
            cfg.db_number,
            self._db_read_size,
        )

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """
        Establish a connection to the PLC if not already connected.

        Returns
        -------
        bool
            ``True`` on success.
        """
        if self.is_connected():
            return True

        try:
            self._client.connect(
                self._cfg.ip,
                self._cfg.rack,
                self._cfg.slot,
            )
            connected: bool = bool(self._client.get_connected())
            self._is_connected_cached = connected
            if connected:
                log.info("PLC connected (%s).", self._cfg.ip)
            else:
                log.warning(
                    "PLC connection handshake failed (%s).",
                    self._cfg.ip,
                )
            return connected
        except Exception as exc:
            log.error("PLC connection failed (%s): %s", self._cfg.ip, exc)
            self._is_connected_cached = False
            return False

    def disconnect(self) -> None:
        """Gracefully disconnect from the PLC."""
        with self._lock:
            try:
                self._client.disconnect()
                self._is_connected_cached = False
                log.info("PLC disconnected.")
            except Exception as exc:
                log.warning("Error during PLC disconnect: %s", exc)

    def is_connected(self) -> bool:
        """Return ``True`` if the snap7 client reports an active connection."""
        try:
            return self._is_connected_cached or bool(self._client.get_connected())
        except Exception:
            return False

    # ------------------------------------------------------------------
    # Data Block reads
    # ------------------------------------------------------------------

    def read_status(self) -> dict[str, Any]:
        """
        Read the robot status Data Block from the PLC.

        Returns
        -------
        dict
            Parsed status values keyed by field name, or an empty dict on
            failure (disconnected or communication error).
        """
        with self._lock:
            if not self.is_connected():
                return {}

            try:
                raw: bytearray = self._client.db_read(
                    self._cfg.db_number, 0, self._db_read_size
                )
                return {
                    # AUTO block (offset 0.0)
                    "start_auto": get_bool(raw, 0, 0),
                    "pause": get_bool(raw, 0, 1),
                    "auto_mode": get_bool(raw, 0, 2),
                    "j1_target": get_real(raw, self._cfg.offsets.j1_target),
                    "j2_target": get_real(raw, self._cfg.offsets.j2_target),
                    "j3_target": get_real(raw, self._cfg.offsets.j3_target),
                    "j4_target": get_real(raw, self._cfg.offsets.j4_target) if hasattr(self._cfg.offsets, 'j4_target') else 0.0,
                    "grip": get_bool(raw, 14, 0),
                    "auto_mode_1": get_bool(raw, 14, 1),

                    # MANUAL block (offset 16.0)
                    "manual_mode": get_bool(raw, 16, 0),
                    "move_to_home": get_bool(raw, 16, 1),
                    "gap_vat": get_bool(raw, 16, 2),
                    "nha_vat": get_bool(raw, 16, 3),
                    "stop_robot": get_bool(raw, 16, 4),
                    "dong_hoc_thuan": get_bool(raw, 16, 5),
                    "dong_hoc_nghich": get_bool(raw, 16, 6),

                    # DONG_HOC_THUAN block (offset 18.0)
                    "theta_j1": get_real(raw, 18),
                    "theta_j2": get_real(raw, 22),
                    "theta_j3": get_real(raw, 26),
                    "py_j2_ht": get_real(raw, 30),
                    "pz_j3_ht": get_real(raw, 34),
                    "px_j1_ht": get_real(raw, 38),

                    # DONG_HOC_NGHICH block (offset 42.0)
                    "px_j1": get_real(raw, 42),
                    "py_j2": get_real(raw, 46),
                    "pz_j3": get_real(raw, 50),
                    "theta_j2_ht": get_real(raw, 54),
                    "theta_j3_ht": get_real(raw, 58),
                    "theta_j1_ht": get_real(raw, 62),

                    # Status flags (use dynamic offsets with simulation fallbacks)
                    "motion_done": get_bool(raw, self._cfg.offsets.motion_done_byte, self._cfg.offsets.motion_done_bit) if hasattr(self._cfg.offsets, 'motion_done_byte') else self._check_simulated_motion_done(),
                    "error_flag": get_bool(raw, self._cfg.offsets.error_flag_byte, self._cfg.offsets.error_flag_bit) if hasattr(self._cfg.offsets, 'error_flag_byte') else False,

                    # Classification status (GIAO_DIEN offset 78.x)
                    "phan_loai_hang": get_bool(raw, self._cfg.offsets.phan_loai_hang_byte, self._cfg.offsets.phan_loai_hang_bit),
                    "hang_tot": get_bool(raw, self._cfg.offsets.hang_tot_byte, self._cfg.offsets.hang_tot_bit),
                    "hang_xau": get_bool(raw, self._cfg.offsets.hang_xau_byte, self._cfg.offsets.hang_xau_bit),
                }
            except Exception as exc:
                log.error("DB read error: %s", exc)
                self._is_connected_cached = False
                return {}
            finally:
                self._last_read_time = time.monotonic()

    def _check_simulated_motion_done(self) -> bool:
        """Check if simulated travel time has elapsed."""
        return (time.monotonic() - self._last_motion_start_time) >= self._motion_duration

    # ------------------------------------------------------------------
    # Data Block writes
    # ------------------------------------------------------------------

    def write_bit(self, byte_offset: int, bit_offset: int, value: bool) -> None:
        """Write a single bit to the PLC Data Block (thread-safe)."""
        with self._lock:
            if not self.is_connected():
                return
            try:
                # Read 1 byte
                data = self._client.db_read(self._cfg.db_number, byte_offset, 1)
                # Set the bit
                from snap7.util import set_bool
                set_bool(data, 0, bit_offset, value)
                # Write 1 byte back
                self._client.db_write(self._cfg.db_number, byte_offset, data)
            except Exception as exc:
                log.error("Failed to write bit at %d.%d: %s", byte_offset, bit_offset, exc)
                self._is_connected_cached = False

    def write_classification(self, is_good: bool) -> None:
        """Write classification result bits to PLC (PHAN_LOAI_HANG, HANG_TOT, HANG_XAU)."""
        off = self._cfg.offsets
        self.write_bit(off.phan_loai_hang_byte, off.phan_loai_hang_bit, True)
        self.write_bit(off.hang_tot_byte, off.hang_tot_bit, is_good)
        self.write_bit(off.hang_xau_byte, off.hang_xau_bit, not is_good)
        if self.tx_callback:
            label = "HANG_TOT" if is_good else "HANG_XAU"
            self.tx_callback(f"Classification written: {label}")
        log.info("Classification written to PLC: is_good=%s", is_good)

    def clear_classification(self) -> None:
        """Reset all classification bits to False on the PLC."""
        off = self._cfg.offsets
        self.write_bit(off.phan_loai_hang_byte, off.phan_loai_hang_bit, False)
        self.write_bit(off.hang_tot_byte, off.hang_tot_bit, False)
        self.write_bit(off.hang_xau_byte, off.hang_xau_bit, False)
        log.info("Classification bits cleared on PLC.")

    def send_pulse(self, byte_offset: int, bit_offset: int) -> None:
        """Write True, wait 150ms, then write False to simulate a push button."""
        def _run():
            self.write_bit(byte_offset, bit_offset, True)
            time.sleep(0.15)
            self.write_bit(byte_offset, bit_offset, False)
        threading.Thread(target=_run, daemon=True).start()

    def send_command(self, cmd: int) -> None:
        """
        Write a command word to the PLC command register.
        Mapped to push buttons for backward compatibility.
        """
        if cmd == 1:
            self.send_pulse(16, 1)  # MOVE_TO_HOME (MANUAL.MOVE_TO_HOME)
        elif cmd == 2:
            self.send_pulse(0, 0)   # START_AUTO (AUTO.START_AUTO)
        elif cmd == 3:
            self.send_pulse(16, 4)  # STOP_ROBOT (MANUAL.STOP_ROBOT)
        elif cmd == 0:
            self.send_pulse(0, 1)   # PAUSE (AUTO.PAUSE)
        elif cmd == 4:
            self.send_pulse(16, 2)  # GAP_VAT (MANUAL.GAP_VAT)

    def send_joint_targets(
        self,
        j1: float = 0.0,
        j2: float = 0.0,
        j3: float = 0.0,
        j4: float = 0.0,
    ) -> None:
        """
        Write four joint-angle targets (degrees) to the PLC Data Block (AUTO.J1 - AUTO.J4)
        and pulse START_AUTO to trigger motion.
        """
        with self._lock:
            # Estimate travel time based on distance
            dist = max(
                abs(j1 - self._target_j1),
                abs(j2 - self._target_j2),
                abs(j3 - self._target_j3),
                abs(j4 - self._target_j4)
            )
            # Travel duration: max change / speed (deg/s) plus minimum latency
            duration = max(0.5, dist / 45.0)

            self._target_j1 = j1
            self._target_j2 = j2
            self._target_j3 = j3
            self._target_j4 = j4

            if not self.is_connected():
                log.warning("send_joint_targets skipped – PLC not connected.")
                return

            try:
                buf = bytearray(16)
                set_real(buf, 0, j1)
                set_real(buf, 4, j2)
                set_real(buf, 8, j3)
                set_real(buf, 12, j4)
                self._client.db_write(self._cfg.db_number, self._cfg.offsets.j1_target, buf)
                
                # Start simulated travel timer
                self._last_motion_start_time = time.monotonic()
                self._motion_duration = duration

                # Pulse the START_AUTO bit to command motion execution
                self.send_pulse(0, 0)

                log.debug("Joint targets written – J1=%.2f J2=%.2f J3=%.2f J4=%.2f. Triggered START_AUTO.", j1, j2, j3, j4)
                if self.tx_callback:
                    self.tx_callback(
                        f"Write Targets: J1={j1:.2f}°, J2={j2:.2f}°, J3={j3:.2f}°, J4={j4:.2f}°"
                    )
            except Exception as exc:
                log.error("send_joint_targets failed: %s", exc)
                self._is_connected_cached = False

    def send_joint_targets_and_command(
        self,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        cmd: int,
    ) -> None:
        """Wrapper for joint targets send, triggering command mapping if needed."""
        self.send_joint_targets(j1, j2, j3, j4)
        if cmd != 2: # If not standard move, also send the mapped command pulse
            self.send_command(cmd)

    def send_forward_kinematics_data(
        self,
        j1: float,
        j2: float,
        j3: float,
        x: float,
        y: float,
        z: float,
    ) -> None:
        """Write THETA_J1, THETA_J2, THETA_J3, Py_J2_HT, Pz_J3_HT, Px_J1_HT to PLC."""
        with self._lock:
            if not self.is_connected():
                return
            try:
                buf = bytearray(24)
                set_real(buf, 0, j1)    # THETA_J1 at 18.0
                set_real(buf, 4, j2)    # THETA_J2 at 22.0
                set_real(buf, 8, j3)    # THETA_J3 at 26.0
                set_real(buf, 12, y)   # Py_J2_HT at 30.0
                set_real(buf, 16, z)   # Pz_J3_HT at 34.0
                set_real(buf, 20, x)   # Px_J1_HT at 38.0
                self._client.db_write(self._cfg.db_number, 18, buf)
                log.info("Sent FK data: Theta=[%.2f, %.2f, %.2f], XYZ=[%.2f, %.2f, %.2f]", j1, j2, j3, x, y, z)
            except Exception as exc:
                log.error("Failed to send FK data: %s", exc)
                self._is_connected_cached = False

    def send_inverse_kinematics_data(
        self,
        x: float,
        y: float,
        z: float,
        j1: float,
        j2: float,
        j3: float,
    ) -> None:
        """Write PX_J1, PY_J2, PZ_J3, THETA_J2_HT, THETA_J3_HT, THETA_J1_HT to PLC."""
        with self._lock:
            if not self.is_connected():
                return
            try:
                buf = bytearray(24)
                set_real(buf, 0, x)     # PX_J1 at 42.0
                set_real(buf, 4, y)     # PY_J2 at 46.0
                set_real(buf, 8, z)     # PZ_J3 at 50.0
                set_real(buf, 12, j2)   # THETA_J2_HT at 54.0
                set_real(buf, 16, j3)   # THETA_J3_HT at 58.0
                set_real(buf, 20, j1)   # THETA_J1_HT at 62.0
                self._client.db_write(self._cfg.db_number, 42, buf)
                log.info("Sent IK data: XYZ=[%.2f, %.2f, %.2f], Theta=[%.2f, %.2f, %.2f]", x, y, z, j1, j2, j3)
            except Exception as exc:
                log.error("Failed to send IK data: %s", exc)
                self._is_connected_cached = False
