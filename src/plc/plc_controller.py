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
import math
import threading
import time
from typing import Any, Callable

import snap7
from snap7.util import get_bool, get_real, set_bool, set_real

from src.config_loader import PLCConfig, PLCOffsets, compute_db_read_size, validate_motion_layout

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
        # Reconnect must wait until disconnect has joined the old pulse worker.
        self._lifecycle_lock = threading.Lock()
        self._is_connected_cached: bool = False
        self._needs_connect = False
        # TTL cache for is_connected() – avoids calling get_connected() on every poll
        self._connection_cache_time: float = 0.0
        self._CONNECTION_CACHE_TTL: float = 0.5
        self.tx_callback: Callable[[str], None] | None = None

        # Active push-button pulses keyed by (byte, bit).
        # Uses a single dedicated background worker thread instead of spawning
        # a new threading.Timer on every pulse, eliminating thread churn.
        self._pulse_timers: dict[tuple[int, int], tuple[Any, int]] = {}
        self._pulse_generations: dict[tuple[int, int], int] = {}
        self._pulse_expiries: dict[tuple[int, int], float] = {}
        self._pulse_starts: dict[tuple[int, int], tuple[float, int]] = {}
        self._pulse_low_since: dict[tuple[int, int], float] = {}
        self._motion_inhibited = False
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
        with self._lifecycle_lock, self._lock:
            try:
                # A previous disconnect can still be finishing a blocked I/O.
                # Never accept commands when no worker can clear their pulses.
                if self._pulse_worker_stop.is_set():
                    if self._pulse_thread.is_alive():
                        log.warning("PLC reconnect deferred: previous pulse worker is still stopping.")
                        return False
                    self._pulse_worker_stop.clear()
                    self._pulse_thread = threading.Thread(
                        target=self._pulse_worker_loop, name="PLCPulseWorker", daemon=True
                    )
                    self._pulse_thread.start()
                if self.is_connected():
                    return True
                if self._needs_connect:
                    try:
                        self._client.disconnect()
                    except Exception:
                        log.debug("Discarding failed PLC transport before reconnect.", exc_info=True)
                self._client.connect(self._cfg.ip, self._cfg.rack, self._cfg.slot)
                connected = bool(self._client.get_connected())
                self._is_connected_cached = connected
                self._needs_connect = not connected
                self._connection_cache_time = time.monotonic()
                if connected:
                    log.info("PLC connected (%s).", self._cfg.ip)
                else:
                    log.warning("PLC connection handshake failed (%s).", self._cfg.ip)
                return connected
            except Exception as exc:
                log.error("PLC connection failed (%s): %s", self._cfg.ip, exc)
                self._mark_disconnected()
                return False

    def disconnect(self) -> None:
        """Gracefully disconnect from the PLC and cancel pending pulses."""
        with self._lifecycle_lock:
            with self._lock:
                self._pulse_worker_stop.set()
                for handle, _generation in list(self._pulse_timers.values()):
                    if hasattr(handle, "cancel"):
                        handle.cancel()
                self._pulse_timers.clear()
                self._pulse_generations.clear()
                self._pulse_expiries.clear()
                self._pulse_starts.clear()
                self._pulse_worker_event.set()
                try:
                    self._client.disconnect()
                    log.info("PLC disconnected.")
                except Exception as exc:
                    log.warning("Error during PLC disconnect: %s", exc)
                finally:
                    self._is_connected_cached = False
                    self._needs_connect = True
                    self._connection_cache_time = time.monotonic()

            if self._pulse_thread is not threading.current_thread():
                self._pulse_thread.join(timeout=1.0)

    def is_connected(self) -> bool:
        """
        Return ``True`` if the PLC is connected.

        Uses ``get_connected()`` as the primary source of truth.  A short TTL
        cache (0.5 s) prevents hammering the snap7 API on every poll cycle.
        Cache is invalidated immediately by :meth:`_mark_disconnected`.
        """
        import time as _time

        with self._lock:
            if self._needs_connect:
                return False
            now = _time.monotonic()
            if now - self._connection_cache_time < self._CONNECTION_CACHE_TTL:
                return self._is_connected_cached
            try:
                live = bool(self._client.get_connected())
            except Exception:
                live = False
            self._is_connected_cached = live
            self._connection_cache_time = now
            return live

    def _mark_disconnected(self) -> None:
        """Flag the connection as lost after an I/O error (lock must be held)."""
        self._is_connected_cached = False
        self._needs_connect = True
        self._connection_cache_time = 0.0  # Invalidate TTL cache immediately

    def _notify_tx(self, message: str) -> None:
        """A display/log callback must never turn successful I/O into a retry."""
        if self.tx_callback is not None:
            try:
                self.tx_callback(message)
            except Exception:
                log.exception("PLC transmit notification failed after successful write.")

    # ------------------------------------------------------------------
    # Data Block reads
    # ------------------------------------------------------------------

    def validate_motion_layout(self) -> None:
        """PC-supervised motion requires dedicated status bits, never REAL bytes."""
        validate_motion_layout(self._cfg.offsets)

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
                status = {
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
                    "theta_j1": get_real(raw, off.theta_j1),
                    "theta_j2": get_real(raw, off.theta_j2),
                    "theta_j3": get_real(raw, off.theta_j3),
                    "py_j2_ht": get_real(raw, off.py_j2_ht),
                    "pz_j3_ht": get_real(raw, off.pz_j3_ht),
                    "px_j1_ht": get_real(raw, off.px_j1_ht),
                    # IK data block (word offsets 42-62)
                    "px_j1": get_real(raw, off.px_j1),
                    "py_j2": get_real(raw, off.py_j2),
                    "pz_j3": get_real(raw, off.pz_j3),
                    "theta_j2_ht": get_real(raw, off.theta_j2_ht),
                    "theta_j3_ht": get_real(raw, off.theta_j3_ht),
                    "theta_j1_ht": get_real(raw, off.theta_j1_ht),
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
                if any(isinstance(value, float) and not math.isfinite(value)
                       for value in status.values()):
                    raise ValueError("PLC status contains a non-finite REAL value.")
                return status
            except Exception as exc:
                log.error("DB read error: %s", exc)
                self._mark_disconnected()
                return {}

    # ------------------------------------------------------------------
    # Data Block writes
    # ------------------------------------------------------------------

    def write_bit(self, byte_offset: int, bit_offset: int, value: bool) -> bool:
        """Write a single bit to the PLC Data Block (thread-safe)."""
        return self.write_bits(byte_offset, {bit_offset: value})

    def write_bits(self, byte_offset: int, bit_values: dict[int, bool]) -> bool:
        """
        Set multiple bits within ONE byte using a single read-modify-write
        cycle (thread-safe).

        Grouping bits that share a byte into one cycle prevents concurrent
        writers (poll thread, pulse threads, GUI) from interleaving their
        own RMW sequences and silently losing each other's changes.
        """
        if not bit_values:
            return True
        with self._lock:
            if self._motion_inhibited:
                allowed = {ADDR.STOP_ROBOT, (self._cfg.offsets.error_ack_byte, self._cfg.offsets.error_ack_bit)}
                if any(value and (byte_offset, bit) not in allowed for bit, value in bit_values.items()):
                    return False
            if not self.is_connected():
                return False
            try:
                data = bytearray(
                    self._client.db_read(self._cfg.db_number, byte_offset, 1)
                )
                for bit_offset, value in bit_values.items():
                    set_bool(data, 0, bit_offset, value)
                self._client.db_write(self._cfg.db_number, byte_offset, data)
                return True
            except Exception as exc:
                log.error(
                    "Failed to write bits at byte %d %s: %s",
                    byte_offset,
                    sorted(bit_values),
                    exc,
                )
                self._mark_disconnected()
                return False

    def write_classification(self, is_good: bool) -> bool:
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

        with self._lock:
            if len(groups) == 1:
                # Default layout remains a single atomic byte update.
                byte_off, bit_values = next(iter(groups.items()))
                if not self.write_bits(byte_off, bit_values):
                    return False
            else:
                # A PLC scan may run between DB writes. Deassert VALID first,
                # stage every label bit, and assert VALID only after success.
                trigger = (off.phan_loai_hang_byte, off.phan_loai_hang_bit)
                if not self.write_bit(*trigger, False):
                    return False
                groups[trigger[0]][trigger[1]] = False
                for byte_off, bit_values in groups.items():
                    if not self.write_bits(byte_off, bit_values):
                        return False
                if not self.write_bit(*trigger, True):
                    return False

        label = "HANG_TOT" if is_good else "HANG_XAU"
        self._notify_tx(f"Classification written: {label}")
        log.info("Classification written to PLC: is_good=%s", is_good)
        return True

    def clear_classification(self) -> bool:
        """Reset all classification bits to False on the PLC."""
        off = self._cfg.offsets
        groups: dict[int, dict[int, bool]] = {}
        for byte_off, bit_off in (
            (off.phan_loai_hang_byte, off.phan_loai_hang_bit),
            (off.hang_tot_byte, off.hang_tot_bit),
            (off.hang_xau_byte, off.hang_xau_bit),
        ):
            groups.setdefault(byte_off, {})[bit_off] = False

        with self._lock:
            # Preserve insertion order: the classification-valid bit is first.
            results = [self.write_bits(byte_off, bit_values)
                       for byte_off, bit_values in groups.items()]
        cleared = all(results)
        if cleared:
            log.info("Classification bits cleared on PLC.")
        return cleared

    def _pulse_worker_loop(self) -> None:
        """Schedule both pulse edges; never sleep with the client lock held."""
        import time
        while not self._pulse_worker_stop.is_set():
            self._pulse_worker_event.clear()
            with self._lock:
                now = time.monotonic()
                for key, (due, generation) in list(self._pulse_starts.items()):
                    if now >= due:
                        self._start_pulse(*key, generation)
                for key, due in list(self._pulse_expiries.items()):
                    if now >= due:
                        self._finish_pulse(*key, self._pulse_generations.get(key, 0))
                deadlines = list(self._pulse_expiries.values())
                deadlines += [due for due, _ in self._pulse_starts.values()]
                timeout = max(0.001, min(deadlines) - time.monotonic()) if deadlines else 0.5
            self._pulse_worker_event.wait(timeout)

    def send_pulse(self, byte_offset: int, bit_offset: int) -> bool:
        """Accept a pulse; repeated pulses rise after a scheduled 50 ms LOW gap.

        True means the first edge was written or a repeated edge was queued.
        Later I/O failures are logged and observed by motion status supervision.
        STOP uses stop_motion() so disconnect cannot truncate its assertion.
        """
        import time
        key = (byte_offset, bit_offset)
        if key == ADDR.STOP_ROBOT:
            return self.stop_motion()
        with self._lock:
            if self._pulse_worker_stop.is_set():
                return False
            ack = (self._cfg.offsets.error_ack_byte, self._cfg.offsets.error_ack_bit)
            if self._motion_inhibited and key != ack:
                return False
            active = key in self._pulse_timers
            if active and not self.write_bit(*key, False):
                return False
            if active:
                self._pulse_low_since[key] = time.monotonic()
            self._pulse_starts.pop(key, None)
            self._pulse_expiries.pop(key, None)
            generation = self._pulse_generations.get(key, 0) + 1
            self._pulse_generations[key] = generation
            handle = _PulseHandle(lambda: self._finish_pulse(*key, generation))
            self._pulse_timers[key] = (handle, generation)
            due = self._pulse_low_since.get(key, -1.0) + 0.05
            if due > time.monotonic():
                self._pulse_starts[key] = (due, generation)
                accepted = True
            else:
                accepted = self._start_pulse(*key, generation)
            self._pulse_worker_event.set()
            return accepted

    def _start_pulse(self, byte_offset: int, bit_offset: int, generation: int) -> bool:
        import time
        key = (byte_offset, bit_offset)
        with self._lock:
            entry = self._pulse_timers.get(key)
            if entry is None or entry[1] != generation:
                return False
            self._pulse_starts.pop(key, None)
            if not self.write_bit(*key, True):
                self._pulse_timers.pop(key, None)
                return False
            self._pulse_expiries[key] = time.monotonic() + self.PULSE_WIDTH_S
            return True

    def cancel_pulse(self, byte_offset: int, bit_offset: int) -> None:
        import time
        key = (byte_offset, bit_offset)
        with self._lock:
            self._pulse_timers.pop(key, None)
            self._pulse_starts.pop(key, None)
            self._pulse_expiries.pop(key, None)
            self.write_bit(*key, False)
            self._pulse_low_since[key] = time.monotonic()

    def _finish_pulse(self, byte_offset: int, bit_offset: int, generation: int) -> None:
        key = (byte_offset, bit_offset)
        with self._lock:
            entry = self._pulse_timers.get(key)
            if entry is None or entry[1] != generation:
                return
            self.cancel_pulse(*key)

    def stop_motion(self) -> bool:
        """Inhibit further commands and latch STOP; retain gripper state.

        STOP remains asserted through disconnect until an explicit healthy reset.
        This is a software stop request, not proof that the machine has stopped.
        """
        with self._lock:
            self._motion_inhibited = True
            for key in list(self._pulse_timers):
                self.cancel_pulse(*key)
            self._pulse_starts.clear()
            self._pulse_expiries.clear()
            jog_cleared = self.write_bits(80, {bit: False for bit in range(6)})
            ok = self.write_bit(*ADDR.STOP_ROBOT, True)
            if not jog_cleared:
                log.error("Could not clear JOG commands while requesting STOP.")
            self._pulse_worker_event.set()
            return ok

    def resume_commands(self) -> bool:
        with self._lock:
            # Clear level-held motion inputs before releasing STOP. In
            # particular, a failed JOG clear at stop time must not restart JOG.
            for byte, bits in ((0, {0: False, 1: False}),
                               (16, {1: False, 2: False, 3: False, 5: False, 6: False}),
                               (80, {bit: False for bit in range(6)})):
                if not self.write_bits(byte, bits):
                    return False
            if not self.write_bit(*ADDR.STOP_ROBOT, False):
                return False
            self._motion_inhibited = False
            return True

    def acknowledge_error(self) -> bool:
        off = self._cfg.offsets
        if off.error_ack_byte < 0:
            log.warning("No PLC error-ack bit configured; acknowledge the fault on the PLC.")
            return False
        return self.send_pulse(off.error_ack_byte, off.error_ack_bit)

    def send_command(self, cmd: int) -> bool:
        """
        Map a legacy command word to its push-button pulse.

        Mapping (kept for backward compatibility):
            0 → PAUSE · 1 → MOVE_TO_HOME · 2 → START_AUTO
            3 → STOP_ROBOT · 4 → GAP_VAT
        """
        mapping: dict[int, tuple[int, int]] = {
            self._cfg.commands.idle: ADDR.PAUSE,
            self._cfg.commands.home: ADDR.MOVE_TO_HOME,
            self._cfg.commands.move: ADDR.START_AUTO,
            self._cfg.commands.stop: ADDR.STOP_ROBOT,
            self._cfg.commands.grip: ADDR.GAP_VAT,
        }
        addr = mapping.get(cmd)
        if addr is None:
            log.warning("Unknown PLC command word: %d", cmd)
            return False
        return self.send_pulse(addr[0], addr[1])

    def write_joint_targets(
        self,
        j1: float = 0.0,
        j2: float = 0.0,
        j3: float = 0.0,
        j4: float = 0.0,
    ) -> bool:
        """
        Write joint-angle targets (degrees) to the PLC Data Block **without**
        pulsing any command bit.

        This is the correct primitive for the QR autonomous mode where the PLC
        ladder program reads the targets and drives the sequence itself after the
        NPN sensor fires.  Calling this function never triggers START_AUTO.

        When j4_target < 0 (unmapped), only J1..J3 are written (12 bytes),
        preventing memory overwrite of subsequent data.

        Returns
        -------
        bool
            ``True`` when db_write succeeds; ``False`` when the PLC is not
            connected, no valid offset is configured, or the write raises.
        """
        try:
            valid = all(math.isfinite(value) for value in (j1, j2, j3, j4))
        except (TypeError, ValueError):
            valid = False
        if not valid:
            log.error("Non-finite joint targets rejected.")
            return False
        with self._lock:
            if self._motion_inhibited:
                return False
            if not self.is_connected():
                log.warning("write_joint_targets skipped – PLC not connected.")
                return False

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
                    "write_joint_targets skipped – no valid target offsets configured."
                )
                return False

            try:
                # Never write across unmapped gaps: the PLC may own those bytes.
                targets.sort()
                groups = []
                for offset, value in targets:
                    if groups and offset == groups[-1][-1][0] + 4:
                        groups[-1].append((offset, value))
                    else:
                        groups.append([(offset, value)])
                encoded = []
                for group in groups:
                    buf = bytearray(4 * len(group))
                    for index, (_, value) in enumerate(group):
                        set_real(buf, 4 * index, value)
                    encoded.append((group[0][0], buf))
            except (TypeError, ValueError, OverflowError) as exc:
                log.error("Invalid PLC REAL target: %s", exc)
                return False

            try:
                # Encode the entire request before the first write. A bad last
                # joint must not leave earlier groups partially updated.
                for offset, buf in encoded:
                    self._client.db_write(self._cfg.db_number, offset, buf)
                min_offset = targets[0][0]
                buf_len = 4 * len(targets)

                log.debug(
                    "write_joint_targets: DB%d offset %d len=%d – "
                    "J1=%.2f J2=%.2f J3=%.2f J4=%.2f (no pulse).",
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
                        self._notify_tx(
                            f"Write Targets: J1={j1:.2f}°, J2={j2:.2f}°, "
                            f"J3={j3:.2f}°, J4={j4:.2f}°"
                        )
                    else:
                        self._notify_tx(
                            f"Write Targets (3-DOF): J1={j1:.2f}°, J2={j2:.2f}°, "
                            f"J3={j3:.2f}°"
                        )
                return True
            except Exception as exc:
                log.error("write_joint_targets failed: %s", exc)
                self._mark_disconnected()
                return False

    def send_joint_targets(
        self,
        j1: float = 0.0,
        j2: float = 0.0,
        j3: float = 0.0,
        j4: float = 0.0,
    ) -> bool:
        """
        Write joint-angle targets (degrees) to the PLC Data Block and then
        pulse START_AUTO to trigger motion (vision / manual mode).

        For the QR autonomous mode use :meth:`write_joint_targets` instead –
        it writes the same data but does **not** pulse any command bit.
        """
        with self._lock:
            wrote = self.write_joint_targets(j1, j2, j3, j4)
            if wrote:
                # Pulse the START_AUTO bit to command motion execution
                if not self.send_pulse(*ADDR.START_AUTO):
                    return False
                log.debug(
                    "send_joint_targets: triggered START_AUTO after writing "
                    "J1=%.2f J2=%.2f J3=%.2f J4=%.2f.",
                    j1,
                    j2,
                    j3,
                    j4,
                )
            return wrote

    def send_joint_targets_and_command(
        self, j1: float, j2: float, j3: float, j4: float, cmd: int
    ) -> bool:
        """
        Write joint targets then issue a specific command pulse.

        If *cmd* maps to START_AUTO (cmd=2) use :meth:`send_joint_targets`
        directly.  For all other commands (e.g. MOVE_TO_HOME = cmd=1) this
        method writes the targets via :meth:`write_joint_targets` (no pulse)
        and then calls :meth:`send_command` once – ensuring exactly one pulse
        per move command regardless of which variant the caller uses.
        """
        with self._lock:
            if cmd == self._cfg.commands.move:
                # START_AUTO is already included in send_joint_targets
                return self.send_joint_targets(j1, j2, j3, j4)
            else:
                # Write targets without START_AUTO, then pulse the requested command
                if self.write_joint_targets(j1, j2, j3, j4):
                    return self.send_command(cmd)
                return False

    def _write_real_fields(self, fields: tuple[tuple[int, float], ...], label: str) -> bool:
        """Validate the complete payload before any DB write or motion trigger."""
        try:
            encoded = []
            for offset, value in fields:
                if not math.isfinite(value):
                    raise ValueError("values must be finite")
                buf = bytearray(4)
                set_real(buf, 0, value)
                encoded.append((offset, buf))
        except (TypeError, ValueError, OverflowError) as exc:
            log.error("Invalid %s payload: %s", label, exc)
            return False
        with self._lock:
            if self._motion_inhibited or not self.is_connected():
                return False
            try:
                for offset, buf in encoded:
                    self._client.db_write(self._cfg.db_number, offset, buf)
            except Exception as exc:
                log.error("Failed to send %s data: %s", label, exc)
                self._mark_disconnected()
                return False
        log.info("Sent %s data.", label)
        return True

    def send_forward_kinematics_data(
        self,
        j1: float,
        j2: float,
        j3: float,
        x: float,
        y: float,
        z: float,
    ) -> bool:
        """Write THETA_J1, THETA_J2, THETA_J3, Py_J2_HT, Pz_J3_HT, Px_J1_HT to PLC."""
        off = self._cfg.offsets
        return self._write_real_fields((
            (off.theta_j1, j1), (off.theta_j2, j2), (off.theta_j3, j3),
            (off.py_j2_ht, y), (off.pz_j3_ht, z), (off.px_j1_ht, x),
        ), "FK")

    def send_inverse_kinematics_data(
        self,
        x: float,
        y: float,
        z: float,
        j1: float,
        j2: float,
        j3: float,
    ) -> bool:
        """Write PX_J1, PY_J2, PZ_J3, THETA_J2_HT, THETA_J3_HT, THETA_J1_HT to PLC."""
        off = self._cfg.offsets
        return self._write_real_fields((
            (off.px_j1, x), (off.py_j2, y), (off.pz_j3, z),
            (off.theta_j2_ht, j2), (off.theta_j3_ht, j3), (off.theta_j1_ht, j1),
        ), "IK")
