"""
src/plc/plc_controller.py – Siemens S7-1200 PLC communication layer.

Wraps ``python-snap7`` to provide a clean, type-hinted API for reading
robot status and writing motion commands to a Siemens S7-1200 PLC.
"""

from __future__ import annotations

import logging
import time
from typing import Any

import snap7
from snap7.util import get_bool, get_int, get_real, set_int, set_real

from src.config_loader import PLCConfig, compute_db_read_size

log = logging.getLogger(__name__)


def _compute_db_read_size(offsets: PLCConfig.offsets) -> int:
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
        self._db_read_size: int = compute_db_read_size(cfg.offsets)
        self._last_read_time: float = 0.0
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
        try:
            if self._client.get_connected():
                return True
            self._client.connect(
                self._cfg.ip,
                self._cfg.rack,
                self._cfg.slot,
            )
            connected: bool = self._client.get_connected()
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
            return False

    def disconnect(self) -> None:
        """Gracefully disconnect from the PLC."""
        try:
            self._client.disconnect()
            log.info("PLC disconnected.")
        except Exception as exc:
            log.warning("Error during PLC disconnect: %s", exc)

    def is_connected(self) -> bool:
        """Return ``True`` if the snap7 client reports an active connection."""
        try:
            return bool(self._client.get_connected())
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
        if not self.is_connected():
            return {}

        try:
            raw: bytearray = self._client.db_read(
                self._cfg.db_number, 0, self._db_read_size
            )
            off = self._cfg.offsets
            return {
                "cmd_word": get_int(raw, off.cmd_word),
                "plc_status": get_int(raw, off.status),
                "j1_target": get_real(raw, off.j1_target),
                "j2_target": get_real(raw, off.j2_target),
                "j3_target": get_real(raw, off.j3_target),
                "j4_target": get_real(raw, off.j4_target),
                "motion_done": get_bool(raw, off.motion_done_byte, off.motion_done_bit),
                "error_flag": get_bool(raw, off.error_flag_byte, off.error_flag_bit),
            }
        except Exception as exc:
            log.error("DB read error: %s", exc)
            return {}
        finally:
            self._last_read_time = time.monotonic()

    # ------------------------------------------------------------------
    # Data Block writes
    # ------------------------------------------------------------------

    def send_command(self, cmd: int) -> None:
        """
        Write a command word to the PLC command register.

        Parameters
        ----------
        cmd:
            One of the ``PLCCommands`` integer values (idle/home/move/stop).
        """
        if not self.is_connected():
            log.warning("send_command skipped – PLC not connected.")
            return
        try:
            buf = bytearray(2)
            set_int(buf, 0, cmd)
            self._client.db_write(
                self._cfg.db_number, self._cfg.offsets.cmd_word, buf
            )
            log.debug("Command %d sent to PLC.", cmd)
        except Exception as exc:
            log.error("send_command failed (cmd=%d): %s", cmd, exc)

    def send_joint_targets(
        self,
        j1: float = 0.0,
        j2: float = 0.0,
        j3: float = 0.0,
        j4: float = 0.0,
    ) -> None:
        """
        Write four joint-angle targets (degrees) to the PLC Data Block.

        Parameters
        ----------
        j1, j2, j3, j4:
            Target joint angles in degrees for axes 1–4.
        """
        if not self.is_connected():
            log.warning("send_joint_targets skipped – PLC not connected.")
            return
        try:
            buf = bytearray(16)
            set_real(buf, 0, j1)
            set_real(buf, 4, j2)
            set_real(buf, 8, j3)
            set_real(buf, 12, j4)
            self._client.db_write(
                self._cfg.db_number,
                self._cfg.offsets.j1_target,
                buf,
            )
            log.debug(
                "Joint targets written – J1=%.2f J2=%.2f J3=%.2f J4=%.2f",
                j1, j2, j3, j4,
            )
        except Exception as exc:
            log.error("send_joint_targets failed: %s", exc)

    def send_joint_targets_and_command(
        self,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        cmd: int,
    ) -> None:
        """
        Write all four joint targets and a command word in a single ``db_write``
        call. This is more efficient than calling ``send_joint_targets`` and
        ``send_command`` separately, and avoids a race condition where the PLC
        might act on a stale command before the targets arrive.

        Parameters
        ----------
        j1, j2, j3, j4:
            Target joint angles in degrees.
        cmd:
            Command word value (e.g. ``PLCCommands.move``).
        """
        if not self.is_connected():
            log.warning("send_joint_targets_and_command skipped – PLC not connected.")
            return
        off = self._cfg.offsets
        try:
            # Calculate safe buffer size to cover all fields
            buf_size = max(off.j4_target + 4, off.cmd_word + 2) + 1
            buf = bytearray(buf_size)
            set_int(buf, off.cmd_word, cmd)
            set_real(buf, off.j1_target, j1)
            set_real(buf, off.j2_target, j2)
            set_real(buf, off.j3_target, j3)
            set_real(buf, off.j4_target, j4)
            self._client.db_write(self._cfg.db_number, off.cmd_word, buf)
            log.debug(
                "Atomic write – cmd=%d J1=%.2f J2=%.2f J3=%.2f J4=%.2f",
                cmd, j1, j2, j3, j4,
            )
        except Exception as exc:
            log.error("send_joint_targets_and_command failed: %s", exc)
