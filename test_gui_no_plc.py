"""
test_gui_no_plc.py – Launch the full GUI with a Mock PLC (no real PLC needed).

Usage
-----
    python test_gui_no_plc.py

What it does
------------
* Replaces PLCController with MockPLCController that:
  - Always reports "connected"
  - Simulates motion_done after a short delay
  - Logs all PLC writes to the console (and TX telemetry)
  - Simulates classification bit writes
* Camera and YOLO still run normally (you need a camera + model file).
* All GUI pages (Auto / Manual) work as expected.
* Lock + cooldown mechanisms are fully testable.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable

from src.config_loader import PLCConfig, RobotConfig, load_config

# ── Logging ─────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s – %(message)s",
)
log = logging.getLogger("MockPLC")


# ═══════════════════════════════════════════════════════════════════════════
# MockPLCController – drop-in replacement for PLCController
# ═══════════════════════════════════════════════════════════════════════════


class MockPLCController:
    """
    Simulated PLC that behaves like a real PLCController but runs entirely
    in-memory.  All writes are logged; reads return plausible status data.
    """

    def __init__(self, cfg: PLCConfig) -> None:
        self._cfg = cfg
        self._lock = threading.RLock()
        self.tx_callback: Callable[[str], None] | None = None

        # Simulated joint targets (for status readback)
        # Interpolation state for smooth GUI updates
        self._start_j1: float = 0.0
        self._start_j2: float = 0.0
        self._start_j3: float = 0.0
        self._start_j4: float = 0.0
        self._target_j1: float = 0.0
        self._target_j2: float = 0.0
        self._target_j3: float = 0.0
        self._target_j4: float = 0.0

        self._motion_start: float = 0.0
        self._motion_duration: float = 0.0

        # Classification bits (simulated)
        self._phan_loai_hang: bool = False
        self._hang_tot: bool = False
        self._hang_xau: bool = False

        # Bit storage for write_bit (simulated DB)
        self._bits: dict[tuple[int, int], bool] = {}

        log.info(
            "MockPLCController created – simulating PLC at %s (DB%d)",
            cfg.ip,
            cfg.db_number,
        )

    # ── Connection lifecycle ─────────────────────────────────────────────

    def connect(self) -> bool:
        log.info("Mock PLC connected (simulated).")
        return True

    def disconnect(self) -> None:
        log.info("Mock PLC disconnected.")

    def is_connected(self) -> bool:
        return True  # Always connected in simulation mode

    # ── Data Block reads ─────────────────────────────────────────────────

    def read_status(self) -> dict[str, Any]:
        """Return simulated PLC status data with smooth interpolation."""
        now = time.monotonic()
        elapsed = now - self._motion_start

        if self._motion_duration > 0:
            progress = min(1.0, elapsed / self._motion_duration)
        else:
            progress = 1.0

        motion_done = progress >= 1.0

        # Interpolate current positions
        curr_j1 = self._start_j1 + (self._target_j1 - self._start_j1) * progress
        curr_j2 = self._start_j2 + (self._target_j2 - self._start_j2) * progress
        curr_j3 = self._start_j3 + (self._target_j3 - self._start_j3) * progress
        curr_j4 = self._start_j4 + (self._target_j4 - self._start_j4) * progress

        return {
            # AUTO block
            "start_auto": False,
            "pause": False,
            "auto_mode": True,
            "j1_target": curr_j1,
            "j2_target": curr_j2,
            "j3_target": curr_j3,
            "j4_target": curr_j4,
            "grip": self._bits.get((14, 0), False),
            "auto_mode_1": False,
            # MANUAL block
            "manual_mode": False,
            "move_to_home": False,
            "gap_vat": False,
            "nha_vat": False,
            "stop_robot": False,
            "dong_hoc_thuan": False,
            "dong_hoc_nghich": False,
            # DONG_HOC_THUAN block
            "theta_j1": curr_j1,
            "theta_j2": curr_j2,
            "theta_j3": curr_j3,
            "py_j2_ht": 0.0,
            "pz_j3_ht": 0.0,
            "px_j1_ht": 0.0,
            # DONG_HOC_NGHICH block
            "px_j1": 0.0,
            "py_j2": 0.0,
            "pz_j3": 0.0,
            "theta_j2_ht": 0.0,
            "theta_j3_ht": 0.0,
            "theta_j1_ht": 0.0,
            # Status flags
            "motion_done": motion_done,
            "error_flag": False,
            # Classification
            "phan_loai_hang": self._phan_loai_hang,
            "hang_tot": self._hang_tot,
            "hang_xau": self._hang_xau,
        }

    # ── Data Block writes ────────────────────────────────────────────────

    def write_bit(self, byte_offset: int, bit_offset: int, value: bool) -> None:
        """Simulate writing a single bit to PLC."""
        self._bits[(byte_offset, bit_offset)] = value
        log.debug("write_bit(%d, %d, %s)", byte_offset, bit_offset, value)

    def write_classification(self, is_good: bool) -> None:
        """Simulate classification write."""
        self._phan_loai_hang = True
        self._hang_tot = is_good
        self._hang_xau = not is_good
        label = "HANG_TOT ✓" if is_good else "HANG_XAU ✗"
        log.info("📦 Classification → %s", label)
        if self.tx_callback:
            self.tx_callback(f"[MOCK] Classification written: {label}")

    def clear_classification(self) -> None:
        """Reset classification bits."""
        self._phan_loai_hang = False
        self._hang_tot = False
        self._hang_xau = False
        log.info("Classification bits cleared.")

    def send_pulse(self, byte_offset: int, bit_offset: int) -> None:
        """Simulate push-button pulse."""
        log.debug("send_pulse(%d, %d)", byte_offset, bit_offset)

        def _run():
            self.write_bit(byte_offset, bit_offset, True)
            time.sleep(0.15)
            self.write_bit(byte_offset, bit_offset, False)

        threading.Thread(target=_run, daemon=True).start()

    def send_command(self, cmd: int) -> None:
        """Simulate command write."""
        cmd_names = {
            0: "IDLE/PAUSE",
            1: "HOME",
            2: "MOVE/START_AUTO",
            3: "STOP",
            4: "GAP_VAT",
        }
        log.info("📡 Command → %s (cmd=%d)", cmd_names.get(cmd, "UNKNOWN"), cmd)
        if self.tx_callback:
            self.tx_callback(f"[MOCK] Command: {cmd_names.get(cmd, f'CMD_{cmd}')}")

    def send_joint_targets(
        self,
        j1: float = 0.0,
        j2: float = 0.0,
        j3: float = 0.0,
        j4: float = 0.0,
    ) -> None:
        """Simulate joint target write with motion timer."""
        # Calculate simulated travel time
        dist = max(
            abs(j1 - self._target_j1),
            abs(j2 - self._target_j2),
            abs(j3 - self._target_j3),
            abs(j4 - self._target_j4),
        )
        self._motion_duration = max(0.3, dist / 90.0)  # Faster than real for testing

        # Save current position as start for interpolation
        if self._motion_duration > 0:
            now = time.monotonic()
            elapsed = now - self._motion_start
            progress = (
                min(1.0, elapsed / self._motion_duration)
                if self._motion_duration > 0
                else 1.0
            )
            self._start_j1 = (
                self._start_j1 + (self._target_j1 - self._start_j1) * progress
            )
            self._start_j2 = (
                self._start_j2 + (self._target_j2 - self._start_j2) * progress
            )
            self._start_j3 = (
                self._start_j3 + (self._target_j3 - self._start_j3) * progress
            )
            self._start_j4 = (
                self._start_j4 + (self._target_j4 - self._start_j4) * progress
            )

        self._motion_start = time.monotonic()

        self._target_j1 = j1
        self._target_j2 = j2
        self._target_j3 = j3
        self._target_j4 = j4

        log.info(
            "🎯 Joint targets → J1=%.2f° J2=%.2f° J3=%.2f° J4=%.2f° (travel=%.2fs)",
            j1,
            j2,
            j3,
            j4,
            self._motion_duration,
        )
        if self.tx_callback:
            self.tx_callback(
                f"[MOCK] Targets: J1={j1:.2f}°, J2={j2:.2f}°, J3={j3:.2f}°, J4={j4:.2f}°"
            )

    def send_joint_targets_and_command(
        self, j1: float, j2: float, j3: float, j4: float, cmd: int
    ) -> None:
        """Joint targets + command (sorting controller uses this)."""
        self.send_joint_targets(j1, j2, j3, j4)
        if cmd != 2:
            self.send_command(cmd)

    def send_forward_kinematics_data(
        self, j1: float, j2: float, j3: float, x: float, y: float, z: float
    ) -> None:
        """Simulate FK data write."""
        log.info(
            "FK data → Theta=[%.2f, %.2f, %.2f], XYZ=[%.2f, %.2f, %.2f]",
            j1,
            j2,
            j3,
            x,
            y,
            z,
        )

    def send_inverse_kinematics_data(
        self, x: float, y: float, z: float, j1: float, j2: float, j3: float
    ) -> None:
        """Simulate IK data write."""
        log.info(
            "IK data → XYZ=[%.2f, %.2f, %.2f], Theta=[%.2f, %.2f, %.2f]",
            x,
            y,
            z,
            j1,
            j2,
            j3,
        )


# ═══════════════════════════════════════════════════════════════════════════
# Monkey-patch and launch
# ═══════════════════════════════════════════════════════════════════════════


def main() -> None:
    """Patch PLCController with MockPLCController, then launch the app."""
    import sys
    from PySide6.QtWidgets import QApplication
    from src.ui.theme import GLOBAL_QSS

    # 1. Monkey-patch PLCController BEFORE importing main
    import src.plc.plc_controller as plc_module

    plc_module.PLCController = MockPLCController  # type: ignore[misc,assignment]

    # 2. Also patch it in the main module namespace
    import main as main_module

    main_module.PLCController = MockPLCController  # type: ignore[misc,assignment]

    # 3. Create Qt Application
    qapp = QApplication.instance() or QApplication(sys.argv)
    qapp.setStyleSheet(GLOBAL_QSS)

    # 4. Load config normally
    config: RobotConfig = load_config()

    # 5. Create and run the app (it will use MockPLCController)
    from main import RobotApp

    app = RobotApp(cfg=config)
    app.setWindowTitle(config.app.title + "  [🧪 TEST MODE – No PLC]")

    log.info("=" * 60)
    log.info("  🧪  TEST MODE ACTIVE – Mock PLC (no real PLC needed)")
    log.info("  📷  Camera + YOLO are LIVE (real hardware)")
    log.info(
        "  📸  Manual Capture Mode ACTIVE (ROI: %dx%d)",
        config.yolo.roi_width,
        config.yolo.roi_height,
    )
    log.info("  ⏳  Post-sort cooldown: %.1fs", app._sort_cooldown_duration)
    log.info("=" * 60)

    app.show()
    sys.exit(qapp.exec())


if __name__ == "__main__":
    main()

