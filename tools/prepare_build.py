"""Stage external runtime files beside the EXE using paths from config.yaml."""
from __future__ import annotations

import argparse
from pathlib import Path
import shutil

import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


def prepare_icon() -> Path:
    destination = ROOT / "build" / "vaa_logo.ico"
    destination.parent.mkdir(parents=True, exist_ok=True)
    with Image.open(ROOT / "assets" / "vaa_logo.png") as image:
        image.save(destination, format="ICO", sizes=[(16, 16), (32, 32), (48, 48), (256, 256)])
    return destination


def stage_runtime(destination: Path) -> None:
    destination = destination.resolve()
    config_path = ROOT / "config.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    yolo = config["yolo"]
    # Check all inputs before changing the output directory.
    copies = []
    for key in ("model_path", "calibration_path"):
        value = yolo.get(key)
        if not value:
            continue
        relative = Path(value)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"For portable builds, yolo.{key} must be relative to the project root.")
        source = ROOT / relative
        if not source.is_file():
            raise FileNotFoundError(source)
        copies.append((source, destination / relative))
    destination.mkdir(parents=True, exist_ok=True)
    for source, target in copies:
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    shutil.copy2(config_path, destination / "config.yaml")
    shutil.copytree(ROOT / "assets", destination / "assets", dirs_exist_ok=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--icon", action="store_true")
    parser.add_argument("--destination", type=Path)
    args = parser.parse_args()
    if args.icon:
        print(prepare_icon())
    if args.destination:
        stage_runtime(args.destination)
