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
    """In-memory PLC with distinct target writes, command edges and sensor input."""

    def __init__(self, cfg: PLCConfig) -> None:
        self._cfg = cfg
        self._lock = threading.RLock()
        self.tx_callback = None
        self._connected = True
        self._stopped = False
        self._bits = {}
        self._start = (0.0,) * 4
        self._pending = (0.0,) * 4
        self._destination = self._start
        self._motion_start = 0.0
        self._motion_duration = 0.0
        self._phan_loai_hang = self._hang_tot = self._hang_xau = False

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def validate_motion_layout(self) -> None:
        # The mock uses separate named storage, not overlapping DB bytes.
        return

    def _pose(self) -> tuple:
        progress = min(1.0, (time.monotonic() - self._motion_start) / self._motion_duration) if self._motion_duration else 1.0
        return tuple(a + (b - a) * progress for a, b in zip(self._start, self._destination))

    def read_status(self) -> dict[str, Any]:
        with self._lock:
            if not self._connected:
                return {}
            pose = self._pose()
            done = not self._motion_duration or time.monotonic() - self._motion_start >= self._motion_duration
            data = {f"j{i+1}_target": angle for i, angle in enumerate(self._pending)}
            data.update({f"theta_j{i+1}": angle for i, angle in enumerate(pose[:3])})
            data.update(auto_mode=True, motion_done=done, error_flag=False,
                        grip=self._bits.get((14, 0), False), stop_robot=self._stopped,
                        phan_loai_hang=self._phan_loai_hang, hang_tot=self._hang_tot,
                        hang_xau=self._hang_xau)
            return data

    def write_joint_targets(self, j1=0.0, j2=0.0, j3=0.0, j4=0.0) -> bool:
        with self._lock:
            if not self._connected or self._stopped:
                return False
            self._pending = (j1, j2, j3, j4)
            if self.tx_callback:
                self.tx_callback(f"[MOCK] Targets written without motion: {self._pending}")
            return True

    def simulate_box_arrival(self) -> bool:
        """Simulate the conveyor sensor starting the PLC-owned QR sequence."""
        with self._lock:
            if not self._connected or self._stopped:
                return False
            self._start = self._pose()
            self._destination = self._pending
            distance = max(abs(a-b) for a, b in zip(self._start, self._destination))
            self._motion_duration = max(0.3, distance / 90.0)
            self._motion_start = time.monotonic()
            return True

    def send_joint_targets(self, j1=0.0, j2=0.0, j3=0.0, j4=0.0) -> bool:
        return self.write_joint_targets(j1, j2, j3, j4) and self.send_pulse(0, 0)

    def send_joint_targets_and_command(self, j1, j2, j3, j4, cmd) -> bool:
        return self.write_joint_targets(j1, j2, j3, j4) and self.send_command(cmd)

    def write_bit(self, byte_offset, bit_offset, value) -> bool:
        return self.write_bits(byte_offset, {bit_offset: value})

    def write_bits(self, byte_offset, bit_values) -> bool:
        with self._lock:
            if not self._connected:
                return False
            if self._stopped and any(value and (byte_offset, bit) != (16, 4) for bit, value in bit_values.items()):
                return False
            for bit, value in bit_values.items():
                self._bits[(byte_offset, bit)] = value
            return True

    def write_classification(self, is_good: bool) -> bool:
        with self._lock:
            if not self._connected or self._stopped:
                return False
            self._phan_loai_hang = True
            self._hang_tot, self._hang_xau = is_good, not is_good
            return True

    def clear_classification(self) -> bool:
        with self._lock:
            if not self._connected:
                return False
            self._phan_loai_hang = self._hang_tot = self._hang_xau = False
            return True

    def send_pulse(self, byte_offset, bit_offset) -> bool:
        key = (byte_offset, bit_offset)
        if key == (16, 4):
            return self.stop_motion()
        with self._lock:
            if not self._connected or self._stopped:
                return False
            if key in ((0, 0), (16, 1)):
                return self.simulate_box_arrival()
            return True

    def send_command(self, cmd) -> bool:
        commands = self._cfg.commands
        mapping = {commands.idle: (0, 1), commands.home: (16, 1),
                   commands.move: (0, 0), commands.stop: (16, 4), commands.grip: (16, 2)}
        return self.send_pulse(*mapping[cmd]) if cmd in mapping else False

    def stop_motion(self) -> bool:
        with self._lock:
            self._start = self._destination = self._pose()
            self._motion_duration = 0.0
            self._stopped = True
            self.write_bits(80, {bit: False for bit in range(6)})
            return self.write_bit(16, 4, True)

    def resume_commands(self) -> bool:
        with self._lock:
            if not self.write_bit(16, 4, False):
                return False
            self._stopped = False
            return True

    def acknowledge_error(self) -> bool:
        return self._cfg.offsets.error_ack_byte >= 0 and self.is_connected()

    def send_forward_kinematics_data(self, j1, j2, j3, x, y, z) -> bool:
        with self._lock:
            if not self._connected or self._stopped:
                return False
            log.info("Mock FK angles=%s XYZ=%s", (j1, j2, j3), (x, y, z))
            return True

    def send_inverse_kinematics_data(self, x, y, z, j1, j2, j3) -> bool:
        with self._lock:
            if not self._connected or self._stopped:
                return False
            log.info("Mock IK XYZ=%s angles=%s", (x, y, z), (j1, j2, j3))
            return True


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
    from PySide6.QtWidgets import QPushButton
    sensor_button = QPushButton("SIMULATE BOX SENSOR")
    sensor_button.clicked.connect(lambda: app.plc.simulate_box_arrival())
    app.statusBar().addPermanentWidget(sensor_button)
    app.setWindowTitle(config.app.title + "  [🧪 TEST MODE – No PLC]")

    log.info("=" * 60)
    log.info("  🧪  TEST MODE ACTIVE – Mock PLC (no real PLC needed)")
    log.info("  📷  Camera + YOLO are LIVE (real hardware)")
    log.info("  ⏳  Post-sort cooldown: %.1fs", app._sort_cooldown_duration)
    log.info("=" * 60)

    app.show()
    sys.exit(qapp.exec())


if __name__ == "__main__":
    main()

