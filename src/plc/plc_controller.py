"""
src/plc/plc_controller.py – Siemens S7-1200 PLC communication layer.

Wraps ``python-snap7`` to provide a clean, type-hinted API for reading
robot status and writing motion commands to a Siemens S7-1200 PLC.

Wire protocol
-------------
The DB layout below is shared with ``MockPLCController`` (test_gui_no_plc.py)
and must stay in sync with the TIA Portal project:

    Byte   0.x   AUTO block      (start_auto / pause / auto_mode)
    Byte  14.x   Gripper         (grip / auto_mode_1)
    Byte  16.x   MANUAL block    (home / gap_vat / nha_vat / stop / kine mode)
    Word  18-40  FK results      (theta_j1..3 + xyz_ht)
    Word  42-64  IK data         (px_j1 / py_j2 / pz_j3 + theta_ht)
    Byte  20.x   Status flags    (motion_done / error_flag) – configurable
    Byte  78.x   Classification  (phan_loai_hang / hang_tot / hang_xau)
    Byte  80.x   JOG buttons     (j1 left/right, j2 up/down, j3 up/down)

All public methods are safe to call from any thread; every access to the
underlying snap7 client is serialised through an RLock because the snap7
client itself is NOT thread-safe.
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Callable

import snap7
from snap7.util import get_bool, get_real, set_bool, set_real

from src.config_loader import PLCConfig, PLCOffsets, compute_db_read_size

log = logging.getLogger(__name__)


class ADDR:
    """
    Canonical PLC bit addresses as ``(byte_offset, bit_offset)`` tuples.

    Single source of truth for every hard-wired bit used by the GUI and
    the sorting controller.  Values must match the TIA Portal DB layout.
    """

    # ── AUTO block (byte 0) ──────────────────────────────────────────────
    START_AUTO: tuple[int, int] = (0, 0)
    PAUSE: tuple[int, int] = (0, 1)
    AUTO_MODE: tuple[int, int] = (0, 2)

    # ── Gripper (byte 14) ─────────────────────────────────────────────────
    GRIP: tuple[int, int] = (14, 0)
    AUTO_MODE_1: tuple[int, int] = (14, 1)

    # ── MANUAL block (byte 16) ───────────────────────────────────────────
    MANUAL_MODE: tuple[int, int] = (16, 0)
    MOVE_TO_HOME: tuple[int, int] = (16, 1)
    GAP_VAT: tuple[int, int] = (16, 2)
    NHA_VAT: tuple[int, int] = (16, 3)
    STOP_ROBOT: tuple[int, int] = (16, 4)
    KINE_FORWARD: tuple[int, int] = (16, 5)
    KINE_INVERSE: tuple[int, int] = (16, 6)

    # ── JOG control (bytes 80+) ──────────────────────────────────────────
    JOG_J1_LEFT: tuple[int, int] = (80, 0)
    JOG_J1_RIGHT: tuple[int, int] = (80, 1)
    JOG_J2_UP: tuple[int, int] = (80, 2)
    JOG_J2_DOWN: tuple[int, int] = (80, 3)
    JOG_J3_UP: tuple[int, int] = (80, 4)
    JOG_J3_DOWN: tuple[int, int] = (80, 5)


def _compute_db_read_size(offsets: PLCOffsets) -> int:
    """Backward-compatibility alias – delegates to config_loader.compute_db_read_size."""
    return compute_db_read_size(offsets)


class _PulseHandle:
    """Handle to a scheduled pulse that allows cancellation without thread creation."""

    def __init__(self, cancel_fn: Callable[[], None]) -> None:
        self._cancel_fn = cancel_fn

    def cancel(self) -> None:
        self._cancel_fn()


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

    #: Duration of the True edge of a push-button pulse (seconds).
    PULSE_WIDTH_S: float = 0.15

    def __init__(self, cfg: PLCConfig) -> None:
        self._cfg: PLCConfig = cfg
        self._client: snap7.client.Client = snap7.client.Client()
        self._db_read_size: int = max(80, compute_db_read_size(cfg.offsets))
        self._lock: threading.RLock = threading.RLock()
        self._is_connected_cached: bool = False
        self.tx_callback: Callable[[str], None] | None = None

        # Active push-button pulses keyed by (byte, bit).
        # Uses a single dedicated background worker thread instead of spawning
        # a new threading.Timer on every pulse, eliminating thread churn.
        self._pulse_timers: dict[tuple[int, int], tuple[Any, int]] = {}
        self._pulse_generations: dict[tuple[int, int], int] = {}
        self._pulse_expiries: dict[tuple[int, int], float] = {}
        self._pulse_worker_event = threading.Event()
        self._pulse_worker_stop = threading.Event()
        self._pulse_thread = threading.Thread(
            target=self._pulse_worker_loop,
            name="PLCPulseWorker",
            daemon=True,
        )
        self._pulse_thread.start()

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
        """Gracefully disconnect from the PLC and cancel pending pulses."""
        with self._lock:
            for handle, _generation in list(self._pulse_timers.values()):
                if hasattr(handle, "cancel"):
                    handle.cancel()
            self._pulse_timers.clear()
            self._pulse_generations.clear()
            self._pulse_expiries.clear()
            self._pulse_worker_event.set()
            try:
                self._client.disconnect()
                self._is_connected_cached = False
                log.info("PLC disconnected.")
            except Exception as exc:
                log.warning("Error during PLC disconnect: %s", exc)

    def is_connected(self) -> bool:
        """Return ``True`` if the snap7 client reports an active connection."""
        with self._lock:
            try:
                return self._is_connected_cached or bool(self._client.get_connected())
            except Exception:
                return False

    def _mark_disconnected(self) -> None:
        """Flag the connection as lost after an I/O error (lock must be held)."""
        self._is_connected_cached = False

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

            off = self._cfg.offsets
            try:
                raw: bytearray = self._client.db_read(
                    self._cfg.db_number, 0, self._db_read_size
                )
                return {
                    # AUTO block (offset 0.x)
                    "start_auto": get_bool(raw, *ADDR.START_AUTO),
                    "pause": get_bool(raw, *ADDR.PAUSE),
                    "auto_mode": get_bool(raw, *ADDR.AUTO_MODE),
                    # Joint targets / IK input block (config-driven offsets)
                    "j1_target": get_real(raw, off.j1_target),
                    "j2_target": get_real(raw, off.j2_target),
                    "j3_target": get_real(raw, off.j3_target),
                    "j4_target": (
                        get_real(raw, off.j4_target) if off.j4_target >= 0 else 0.0
                    ),
                    # Gripper (14.x)
                    "grip": get_bool(raw, *ADDR.GRIP),
                    "auto_mode_1": get_bool(raw, *ADDR.AUTO_MODE_1),
                    # MANUAL block (16.x)
                    "manual_mode": get_bool(raw, *ADDR.MANUAL_MODE),
                    "move_to_home": get_bool(raw, *ADDR.MOVE_TO_HOME),
                    "gap_vat": get_bool(raw, *ADDR.GAP_VAT),
                    "nha_vat": get_bool(raw, *ADDR.NHA_VAT),
                    "stop_robot": get_bool(raw, *ADDR.STOP_ROBOT),
                    "dong_hoc_thuan": get_bool(raw, *ADDR.KINE_FORWARD),
                    "dong_hoc_nghich": get_bool(raw, *ADDR.KINE_INVERSE),
                    # FK results block (word offsets 18-38)
                    "theta_j1": get_real(raw, 18),
                    "theta_j2": get_real(raw, 22),
                    "theta_j3": get_real(raw, 26),
                    "py_j2_ht": get_real(raw, 30),
                    "pz_j3_ht": get_real(raw, 34),
                    "px_j1_ht": get_real(raw, 38),
                    # IK data block (word offsets 42-62)
                    "px_j1": get_real(raw, 42),
                    "py_j2": get_real(raw, 46),
                    "pz_j3": get_real(raw, 50),
                    "theta_j2_ht": get_real(raw, 54),
                    "theta_j3_ht": get_real(raw, 58),
                    "theta_j1_ht": get_real(raw, 62),
                    # Status flags (config-driven offsets)
                    "motion_done": get_bool(
                        raw, off.motion_done_byte, off.motion_done_bit
                    ),
                    "error_flag": get_bool(
                        raw, off.error_flag_byte, off.error_flag_bit
                    ),
                    # Classification status (config-driven offsets, default 78.x)
                    "phan_loai_hang": get_bool(
                        raw, off.phan_loai_hang_byte, off.phan_loai_hang_bit
                    ),
                    "hang_tot": get_bool(raw, off.hang_tot_byte, off.hang_tot_bit),
                    "hang_xau": get_bool(raw, off.hang_xau_byte, off.hang_xau_bit),
                }
            except Exception as exc:
                log.error("DB read error: %s", exc)
                self._mark_disconnected()
                return {}

    # ------------------------------------------------------------------
    # Data Block writes
    # ------------------------------------------------------------------

    def write_bit(self, byte_offset: int, bit_offset: int, value: bool) -> None:
        """Write a single bit to the PLC Data Block (thread-safe)."""
        self.write_bits(byte_offset, {bit_offset: value})

    def write_bits(self, byte_offset: int, bit_values: dict[int, bool]) -> None:
        """
        Set multiple bits within ONE byte using a single read-modify-write
        cycle (thread-safe).

        Grouping bits that share a byte into one cycle prevents concurrent
        writers (poll thread, pulse threads, GUI) from interleaving their
        own RMW sequences and silently losing each other's changes.
        """
        if not bit_values:
            return
        with self._lock:
            if not self.is_connected():
                return
            try:
                data = bytearray(
                    self._client.db_read(self._cfg.db_number, byte_offset, 1)
                )
                for bit_offset, value in bit_values.items():
                    set_bool(data, 0, bit_offset, value)
                self._client.db_write(self._cfg.db_number, byte_offset, data)
            except Exception as exc:
                log.error(
                    "Failed to write bits at byte %d %s: %s",
                    byte_offset,
                    sorted(bit_values),
                    exc,
                )
                self._mark_disconnected()

    def write_classification(self, is_good: bool) -> None:
        """
        Write the classification result bits to the PLC
        (PHAN_LOAI_HANG, HANG_TOT, HANG_XAU) in a single atomic cycle
        when they share a byte (the default layout: byte 78).
        """
        off = self._cfg.offsets
        groups: dict[int, dict[int, bool]] = {}
        for byte_off, bit_off, value in (
            (off.phan_loai_hang_byte, off.phan_loai_hang_bit, True),
            (off.hang_tot_byte, off.hang_tot_bit, is_good),
            (off.hang_xau_byte, off.hang_xau_bit, not is_good),
        ):
            groups.setdefault(byte_off, {})[bit_off] = value

        for byte_off, bit_values in groups.items():
            self.write_bits(byte_off, bit_values)

        label = "HANG_TOT" if is_good else "HANG_XAU"
        if self.tx_callback:
            self.tx_callback(f"Classification written: {label}")
        log.info("Classification written to PLC: is_good=%s", is_good)

    def clear_classification(self) -> None:
        """Reset all classification bits to False on the PLC."""
        off = self._cfg.offsets
        groups: dict[int, dict[int, bool]] = {}
        for byte_off, bit_off in (
            (off.phan_loai_hang_byte, off.phan_loai_hang_bit),
            (off.hang_tot_byte, off.hang_tot_bit),
            (off.hang_xau_byte, off.hang_xau_bit),
        ):
            groups.setdefault(byte_off, {})[bit_off] = False

        for byte_off, bit_values in groups.items():
            self.write_bits(byte_off, bit_values)

        log.info("Classification bits cleared on PLC.")

    def _pulse_worker_loop(self) -> None:
        """Background daemon: sequentially handles pulse falling edges without thread creation."""
        import time as _time

        while not self._pulse_worker_stop.is_set():
            now = _time.monotonic()
            due: list[tuple[int, int, int]] = []

            with self._lock:
                # Find all expired pulses
                for key, expiry in list(self._pulse_expiries.items()):
                    if now >= expiry:
                        gen = self._pulse_generations.get(key, 0)
                        due.append((key[0], key[1], gen))
                        self._pulse_expiries.pop(key, None)

            for byte_off, bit_off, gen in due:
                self._finish_pulse(byte_off, bit_off, gen)

            # Determine next sleep duration
            timeout = 0.5
            with self._lock:
                if self._pulse_expiries:
                    earliest = min(self._pulse_expiries.values())
                    timeout = max(0.01, earliest - _time.monotonic())

            self._pulse_worker_event.wait(timeout)
            self._pulse_worker_event.clear()

    def send_pulse(self, byte_offset: int, bit_offset: int) -> None:
        """
        Simulate a push-button press: write True immediately, then reset to
        False after :data:`PULSE_WIDTH_S` seconds via a single worker thread.

        A repeated pulse on the same bit cancels the pending reset first,
        so pulses never stack up threads or leave a bit stuck at True.
        Generation counters guarantee a stale reset callback can never
        truncate the rising edge of a newer pulse.
        """
        import time as _time

        key = (byte_offset, bit_offset)

        with self._lock:
            previous = self._pulse_timers.pop(key, None)
            generation = self._pulse_generations.get(key, 0) + 1
            self._pulse_generations[key] = generation
            if previous is not None and hasattr(previous[0], "cancel"):
                previous[0].cancel()

            self._pulse_expiries[key] = _time.monotonic() + self.PULSE_WIDTH_S
            handle = _PulseHandle(lambda b=byte_offset, o=bit_offset: self.cancel_pulse(b, o))
            self._pulse_timers[key] = (handle, generation)

        self.write_bit(byte_offset, bit_offset, True)
        self._pulse_worker_event.set()

    def cancel_pulse(self, byte_offset: int, bit_offset: int) -> None:
        """Immediately cancel a pending pulse and reset the bit to False."""
        key = (byte_offset, bit_offset)
        with self._lock:
            self._pulse_timers.pop(key, None)
            self._pulse_expiries.pop(key, None)
        self.write_bit(byte_offset, bit_offset, False)

    def _finish_pulse(self, byte_offset: int, bit_offset: int, generation: int) -> None:
        """
        Complete a pulse by resetting the bit (runs sequentially or on demand).

        Skips the reset when superseded by a newer pulse on the same bit —
        otherwise the stale ``False`` write would erase the new pulse's
        ``True`` edge before the PLC has sampled it.
        """
        key = (byte_offset, bit_offset)
        with self._lock:
            entry = self._pulse_timers.get(key)
            if entry is None or entry[1] != generation:
                return  # Superseded by a newer pulse – do not reset.
            self._pulse_timers.pop(key, None)
            self._pulse_expiries.pop(key, None)
        self.write_bit(byte_offset, bit_offset, False)

    def send_command(self, cmd: int) -> None:
        """
        Map a legacy command word to its push-button pulse.

        Mapping (kept for backward compatibility):
            0 → PAUSE · 1 → MOVE_TO_HOME · 2 → START_AUTO
            3 → STOP_ROBOT · 4 → GAP_VAT
        """
        mapping: dict[int, tuple[int, int]] = {
            0: ADDR.PAUSE,
            1: ADDR.MOVE_TO_HOME,
            2: ADDR.START_AUTO,
            3: ADDR.STOP_ROBOT,
            4: ADDR.GAP_VAT,
        }
        addr = mapping.get(cmd)
        if addr is None:
            log.warning("Unknown PLC command word: %d", cmd)
            return
        self.send_pulse(addr[0], addr[1])

    def send_joint_targets(
        self,
        j1: float = 0.0,
        j2: float = 0.0,
        j3: float = 0.0,
        j4: float = 0.0,
    ) -> None:
        """
        Write joint-angle targets (degrees) to the PLC Data Block in a
        single dynamic transaction and pulse START_AUTO to trigger motion.
        When j4_target < 0 (unmapped), only J1..J3 are written (12 bytes),
        preventing memory overwrite of subsequent data.
        """
        with self._lock:
            if not self.is_connected():
                log.warning("send_joint_targets skipped – PLC not connected.")
                return

            try:
                off = self._cfg.offsets
                # Collect mapped joint targets (offset, value)
                targets: list[tuple[int, float]] = []
                if off.j1_target >= 0:
                    targets.append((off.j1_target, j1))
                if off.j2_target >= 0:
                    targets.append((off.j2_target, j2))
                if off.j3_target >= 0:
                    targets.append((off.j3_target, j3))
                if off.j4_target >= 0:
                    targets.append((off.j4_target, j4))

                if not targets:
                    log.warning(
                        "send_joint_targets skipped – no valid target offsets configured."
                    )
                    return

                min_offset = min(o for o, _ in targets)
                max_offset = max(o for o, _ in targets)
                buf_len = (max_offset - min_offset) + 4
                buf = bytearray(buf_len)

                for offset, val in targets:
                    set_real(buf, offset - min_offset, val)

                self._client.db_write(self._cfg.db_number, min_offset, buf)

                # Pulse the START_AUTO bit to command motion execution
                self.send_pulse(*ADDR.START_AUTO)

                log.debug(
                    "Joint targets written to DB%d offset %d (len=%d bytes) – "
                    "J1=%.2f J2=%.2f J3=%.2f J4=%.2f. Triggered START_AUTO.",
                    self._cfg.db_number,
                    min_offset,
                    buf_len,
                    j1,
                    j2,
                    j3,
                    j4,
                )
                if self.tx_callback:
                    if off.j4_target >= 0:
                        self.tx_callback(
                            f"Write Targets: J1={j1:.2f}°, J2={j2:.2f}°, "
                            f"J3={j3:.2f}°, J4={j4:.2f}°"
                        )
                    else:
                        self.tx_callback(
                            f"Write Targets (3-DOF): J1={j1:.2f}°, J2={j2:.2f}°, "
                            f"J3={j3:.2f}°"
                        )
            except Exception as exc:
                log.error("send_joint_targets failed: %s", exc)
                self._mark_disconnected()

    def send_joint_targets_and_command(
        self, j1: float, j2: float, j3: float, j4: float, cmd: int
    ) -> None:
        """Write joint targets and issue a specific command (e.g. MOVE)."""
        self.send_joint_targets(j1, j2, j3, j4)
        # Note: send_joint_targets already pulses START_AUTO (cmd 2);
        # avoid double-pulsing when the caller requests exactly that.
        if cmd != 2:
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
                set_real(buf, 0, j1)  # THETA_J1 at 18.0
                set_real(buf, 4, j2)  # THETA_J2 at 22.0
                set_real(buf, 8, j3)  # THETA_J3 at 26.0
                set_real(buf, 12, y)  # Py_J2_HT at 30.0
                set_real(buf, 16, z)  # Pz_J3_HT at 34.0
                set_real(buf, 20, x)  # Px_J1_HT at 38.0
                self._client.db_write(self._cfg.db_number, 18, buf)
                log.info(
                    "Sent FK data: Theta=[%.2f, %.2f, %.2f], XYZ=[%.2f, %.2f, %.2f]",
                    j1,
                    j2,
                    j3,
                    x,
                    y,
                    z,
                )
            except Exception as exc:
                log.error("Failed to send FK data: %s", exc)
                self._mark_disconnected()

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
                set_real(buf, 0, x)  # PX_J1 at 42.0
                set_real(buf, 4, y)  # PY_J2 at 46.0
                set_real(buf, 8, z)  # PZ_J3 at 50.0
                set_real(buf, 12, j2)  # THETA_J2_HT at 54.0
                set_real(buf, 16, j3)  # THETA_J3_HT at 58.0
                set_real(buf, 20, j1)  # THETA_J1_HT at 62.0
                self._client.db_write(self._cfg.db_number, 42, buf)
                log.info(
                    "Sent IK data: XYZ=[%.2f, %.2f, %.2f], Theta=[%.2f, %.2f, %.2f]",
                    x,
                    y,
                    z,
                    j1,
                    j2,
                    j3,
                )
            except Exception as exc:
                log.error("Failed to send IK data: %s", exc)
                self._mark_disconnected()
