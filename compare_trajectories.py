#!/usr/bin/env python3
"""Compare a live ArUco trajectory against a CoppeliaSim trajectory.

This script reads:
- a live CSV exported by CoppeliaEsp/ArUco/live_v2.py
- a CoppeliaSim CSV exported by path_tracking_kuongshun.py

It then plots both trajectories on the same Cartesian plane using
normalized labels so the figure is easier to read.
"""

from __future__ import annotations

import csv
import re
import shutil
from pathlib import Path
from typing import List, Tuple
from datetime import datetime

import matplotlib.pyplot as plt


BASE_DIR = Path(r"C:\Users\jaide\Desktop\Proyecto-UnB-UDLA-IEEE-rob-tica-mobil")
LIVE_TRAJ_ROOT = BASE_DIR / "CoppeliaEsp" / "Cars_Trajectory"
COPPELIA_ROOT = BASE_DIR


def normalize_label(raw_name: str) -> str:
    """Convert a raw file name into a friendlier plot label."""
    name = raw_name.lower()
    if "carro_0" in name or "car_0" in name:
        return "Carro 0"
    if "robot_1" in name or "coppelia" in name:
        return "Coppelia"
    return Path(raw_name).stem.replace("_", " ").title()


def robot_name_from_index(robot_index: int) -> str:
    """Convert a robot index like 1, 2, 3 into Car_A, Car_B, Car_C."""
    letter_index = robot_index - 1
    if letter_index < 0:
        return f"Car_{robot_index}"
    return f"Car_{chr(ord('A') + letter_index)}"


def extract_index(path: Path, prefix: str) -> int | None:
    match = re.search(rf"{re.escape(prefix)}(\d+)", path.stem)
    if match is None:
        return None
    return int(match.group(1))


def load_live_csv(path: Path) -> Tuple[List[float], List[float], List[float]]:
    times: List[float] = []
    xs: List[float] = []
    ys: List[float] = []

    with path.open("r", newline="", encoding="utf-8") as file_handle:
        reader = csv.DictReader(file_handle)
        for row in reader:
            times.append(float(row["time"]))
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))

    return times, xs, ys


def load_coppelia_csv(path: Path) -> Tuple[List[float], List[float]]:
    xs: List[float] = []
    ys: List[float] = []

    with path.open("r", newline="", encoding="utf-8") as file_handle:
        reader = csv.DictReader(file_handle)
        for row in reader:
            xs.append(float(row["x_out"]))
            ys.append(float(row["y_out"]))

    return xs, ys


def find_latest_live_csv() -> Path:
    candidates = sorted(LIVE_TRAJ_ROOT.glob("*/*.csv"))
    if not candidates:
        raise FileNotFoundError(f"No se encontraron CSV live dentro de {LIVE_TRAJ_ROOT}")
    return max(candidates, key=lambda item: item.stat().st_mtime)


def find_latest_live_session() -> Path:
    session_dirs = [path for path in LIVE_TRAJ_ROOT.iterdir() if path.is_dir()]
    if not session_dirs:
        raise FileNotFoundError(f"No se encontraron carpetas de sesión live dentro de {LIVE_TRAJ_ROOT}")
    return max(session_dirs, key=lambda item: item.stat().st_mtime)


def find_coppelia_csvs() -> List[Path]:
    return sorted(COPPELIA_ROOT.glob("robot_*_trajectory.csv"))


def pair_live_and_coppelia(live_csv: Path, coppelia_csvs: List[Path]) -> Path:
    live_index = extract_index(live_csv, "carro_")
    if live_index is None:
        return coppelia_csvs[0]

    live_to_robot = live_index + 1
    for candidate in coppelia_csvs:
        robot_index = extract_index(candidate, "robot_")
        if robot_index == live_to_robot:
            return candidate

    return coppelia_csvs[0]


def comparison_title(live_csv: Path, coppelia_csv: Path) -> str:
    """Build a cleaner title using the robot name convention from gen_parallel."""
    live_index = extract_index(live_csv, "carro_")
    robot_index = extract_index(coppelia_csv, "robot_")

    if robot_index is not None:
        car_name = robot_name_from_index(robot_index)
    elif live_index is not None:
        car_name = robot_name_from_index(live_index + 1)
    else:
        car_name = "Carro"

    return f"{car_name}: Simulación vs Real"


def main() -> None:
    live_session = find_latest_live_session()
    live_csvs = sorted(live_session.glob("*.csv"))
    coppelia_csvs = find_coppelia_csvs()

    if not live_csvs:
        raise FileNotFoundError(f"No se encontraron CSV live dentro de {live_session}")
    if not coppelia_csvs:
        raise FileNotFoundError(f"No se encontraron CSV de Coppelia en {COPPELIA_ROOT}")

    print(f"Usando sesion live mas reciente: {live_session}")

    output_root = BASE_DIR / "Trajectory_Comparisons"
    output_root.mkdir(exist_ok=True)
    output_dir = output_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir.mkdir(exist_ok=True)

    copied_sources_dir = output_dir / "source_csv"
    copied_sources_dir.mkdir(exist_ok=True)

    for live_csv in live_csvs:
        coppelia_csv = pair_live_and_coppelia(live_csv, coppelia_csvs)

        _, live_xs, live_ys = load_live_csv(live_csv)
        coppelia_xs, coppelia_ys = load_coppelia_csv(coppelia_csv)

        live_label = "Real (Live)"
        coppelia_label = "Simulación (CoppeliaSim)"

        plt.figure(figsize=(7, 7))
        plt.plot(coppelia_xs, coppelia_ys, "b-o", markersize=3, linewidth=1.5, label=coppelia_label)
        plt.plot(live_xs, live_ys, "r-o", markersize=3, linewidth=1.5, label=live_label)

        plt.xlabel("x")
        plt.ylabel("y")
        plt.title(comparison_title(live_csv, coppelia_csv))
        plt.grid(True)
        plt.axis("equal")
        plt.legend()
        plt.tight_layout()

        output_plot = output_dir / f"{live_csv.stem}_vs_{coppelia_csv.stem}.png"
        plt.savefig(output_plot, dpi=150, bbox_inches="tight")
        plt.show()

        shutil.copy2(live_csv, copied_sources_dir / live_csv.name)
        shutil.copy2(coppelia_csv, copied_sources_dir / coppelia_csv.name)

        print(f"Grafica guardada: {output_plot}")
        print(f"CSV copiados en: {copied_sources_dir}")

    print(f"Comparaciones guardadas en: {output_dir}")


if __name__ == "__main__":
    main()