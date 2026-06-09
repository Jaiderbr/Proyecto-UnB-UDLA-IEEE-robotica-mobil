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
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt


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


def find_latest_live_csv(live_traj_root: Path) -> Path:
    candidates = sorted(live_traj_root.glob("*/*.csv"))
    if not candidates:
        raise FileNotFoundError(f"No se encontraron CSV live dentro de {live_traj_root}")
    return max(candidates, key=lambda item: item.stat().st_mtime)


def find_latest_live_session(live_traj_root: Path) -> Path:
    session_dirs = [path for path in live_traj_root.iterdir() if path.is_dir()]
    if not session_dirs:
        raise FileNotFoundError(f"No se encontraron carpetas de sesión live dentro de {live_traj_root}")
    return max(session_dirs, key=lambda item: item.stat().st_mtime)


def find_coppelia_csvs(coppelia_root: Path) -> List[Path]:
    return sorted(coppelia_root.glob("robot_*_trajectory.csv"))


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

def load_csv_auto(path: Path) -> Tuple[List[float], List[float], List[float]]:
    """
    Carga cualquier CSV de trayectoria detectando automáticamente el formato.
    Retorna: (times, xs, ys) — si no hay tiempo, genera índices.
    """
    times: List[float] = []
    xs: List[float] = []
    ys: List[float] = []
    
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"CSV vacío: {path}")
        
        fields = [f.strip().lower() for f in reader.fieldnames]
        
        # Detectar formato
        has_time = any(f in fields for f in ["time", "timestamp", "t"])
        has_x_out = "x_out" in fields
        has_y_out = "y_out" in fields
        has_x = "x" in fields
        has_y = "y" in fields
        
        for row in reader:
            # Extraer X
            if has_x_out:
                xs.append(float(row["x_out"]))
            elif has_x:
                xs.append(float(row["x"]))
            else:
                raise KeyError(f"No se encontró columna X en {path}")
            
            # Extraer Y
            if has_y_out:
                ys.append(float(row["y_out"]))
            elif has_y:
                ys.append(float(row["y"]))
            else:
                raise KeyError(f"No se encontró columna Y en {path}")
            
            # Extraer tiempo (opcional)
            if has_time:
                t = (row.get("time") or row.get("timestamp") or 
                     row.get("Time") or row.get("Timestamp") or row.get("t"))
                times.append(float(t))
            else:
                times.append(float(len(times)))  # Índice como tiempo
    
    return times, xs, ys

REAL_ROOT = Path(r"D:\Proyecto-UnB-UDLA-IEEE-robotica-mobil\20260609_144552")
COPPELIA_ROOT = Path(r"D:\Proyecto-UnB-UDLA-IEEE-robotica-mobil\CoppeliaEsp\Cars_Trajectory_V3\20260609_144552")

def main() -> None:
    real_root = REAL_ROOT
    coppelia_root = COPPELIA_ROOT
    
    # Cargar TODOS los CSV de ambas carpetas
    all_csvs = sorted(real_root.glob("*.csv")) + sorted(coppelia_root.glob("*.csv"))
    
    # Separar por formato según el nombre o contenido
    live_csvs = []
    coppelia_csvs = []
    
    for csv_path in all_csvs:
        # Detectar por nombre primero
        name_lower = csv_path.name.lower()
        if "coppelia" in name_lower or "robot_" in name_lower:
            coppelia_csvs.append(csv_path)
        elif "carro_" in name_lower or "live" in name_lower or csv_path.parent == real_root:
            live_csvs.append(csv_path)
        else:
            # Si no se sabe, detectar por contenido
            with csv_path.open("r", encoding="utf-8") as f:
                header = f.readline().strip().lower()
                if "x_out" in header:
                    coppelia_csvs.append(csv_path)
                else:
                    live_csvs.append(csv_path)
    
    if not live_csvs:
        raise FileNotFoundError(f"No se encontraron CSV live en {real_root}")
    if not coppelia_csvs:
        raise FileNotFoundError(f"No se encontraron CSV de Coppelia en {coppelia_root}")

    print(f"📁 Live ({len(live_csvs)}): {[c.name for c in live_csvs]}")
    print(f"📁 Coppelia ({len(coppelia_csvs)}): {[c.name for c in coppelia_csvs]}")

    # Crear directorio de salida
    output_root = real_root / "Trajectory_Comparisons"
    output_root.mkdir(exist_ok=True)
    output_dir = output_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir.mkdir(exist_ok=True)
    copied_sources_dir = output_dir / "source_csv"
    copied_sources_dir.mkdir(exist_ok=True)

    # Emparejar por índice de robot
    for live_csv in live_csvs:
        coppelia_csv = pair_live_and_coppelia(live_csv, coppelia_csvs)
        
        # Usar la función universal para ambos
        _, live_xs, live_ys = load_csv_auto(live_csv)
        _, coppelia_xs, coppelia_ys = load_csv_auto(coppelia_csv)

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

        print(f"✅ Gráfica guardada: {output_plot}")

    print(f"\n📂 Comparaciones guardadas en: {output_dir}")


if __name__ == "__main__":
    main()