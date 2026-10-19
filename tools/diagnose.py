"""Check dependencies and configuration without connecting a PLC or camera."""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import importlib
from importlib.metadata import version
import json
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def diagnose(config_path: Path | None = None) -> list[dict[str, str]]:
    results = []

    def record(check: str, status: str, detail: str) -> None:
        results.append(dict(check=check, status=status, detail=detail))

    record("python", "OK" if sys.version_info >= (3, 11) else "ERROR",
           f"{platform.python_version()} at {sys.executable}")
    for package, module in (
        ("PyYAML", "yaml"), ("numpy", "numpy"), ("PySide6", "PySide6.QtWidgets"),
        ("pillow", "PIL.Image"), ("opencv-python", "cv2"),
        ("ultralytics", "ultralytics"), ("python-snap7", "snap7"),
    ):
        try:
            with redirect_stdout(sys.stderr):
                importlib.import_module(module)
            record(package, "OK", version(package))
        except Exception as error:
            record(package, "ERROR", str(error))

    try:
        from src.config_loader import load_config, validate_motion_layout
        cfg = load_config(str(config_path or ROOT / "config.yaml"))
        record("configuration", "OK", f"sorting_mode={cfg.app.sorting_mode}")
        model = ROOT / cfg.yolo.model_path
        record("model", "OK" if model.is_file() else ("ERROR" if cfg.app.sorting_mode == "vision" else "WARNING"),
               str(model) + ("" if model.is_file() else " (required for Vision)"))
        if cfg.yolo.calibration_path:
            calibration = ROOT / cfg.yolo.calibration_path
            record("calibration", "OK" if calibration.is_file() else "WARNING", str(calibration))
        try:
            validate_motion_layout(cfg.plc.offsets)
            record("vision_db_layout", "OK", "No overlapping fields detected; verify the TIA mapping separately.")
        except ValueError as error:
            record("vision_db_layout", "ERROR" if cfg.app.sorting_mode == "vision" else "WARNING", str(error))
    except Exception as error:
        record("configuration", "ERROR", str(error))
    return results


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args(argv)
    results = diagnose(args.config)
    if args.as_json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        for result in results:
            print(f"[{result['status']}] {result['check']}: {result['detail']}")
    return int(any(result["status"] == "ERROR" for result in results))


if __name__ == "__main__":
    raise SystemExit(main())
