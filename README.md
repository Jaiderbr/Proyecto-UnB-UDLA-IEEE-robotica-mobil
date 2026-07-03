# UnB-UDLA-IEEE Mobile Robotics Project

**Pose Control and Trajectory Tracking in Differential Robots via Bio-inspired Optimization and CoppeliaSim Simulation**

A collaborative research project between the University of Brasília (UnB) and Universidad de Los Andes (UDLA), focused on autonomous mobile robot control, PID controller optimization using bio-inspired algorithms, and hardware-in-the-loop validation.

---

## Overview

This platform covers the full pipeline from simulation to physical deployment:

1. **Simulation** — differential-drive robots are modeled in CoppeliaSim and controlled via a PID trajectory-tracking controller.
2. **Optimization** — PID gains (Kp, Ki, Kd) are tuned automatically using a suite of nine bio-inspired metaheuristic algorithms.
3. **Hardware** — the optimized controller is deployed to an ESP32-based physical robot.
4. **Validation** — ArUco marker vision tracks the real robot's pose; a correction module compares it against the simulation and sends adjustments to the hardware in real time.

---

## Repository Structure

```
Proyecto-UnB-UDLA-IEEE-robotica-mobil/
├── path_planning_kuongshun/
│   ├── Online_training/          # Bio-inspired PID optimization
│   │   ├── Bioinspired.py        # PSO, ARPSO, DE, ABC, MFO, GWO, WOA, BAT, AHA
│   │   └── online_training_robot.py
│   └── Path_planning/            # Trajectory tracking controllers
│       ├── path_tracking_kuongshun.py   # Main controller (kuongshun robot)
│       ├── path_tracking_Pioneer_4W.py  # Pioneer 4-wheel variant
│       ├── path_tracking_VSSS.py        # VSSS small robot variant
│       ├── gen.py                # Multi-robot sequential launcher
│       ├── gen-parallel.py       # Multi-robot parallel launcher
│       └── corrector.py          # Real-world trajectory correction
├── CoppeliaEsp/
│   ├── ArUco/                    # Real-time ArUco marker detection
│   │   ├── live_v2.py            # Pose publisher via ZMQ (main)
│   │   └── live_v3.py            # Enhanced version
│   └── sketch/                   # ESP32 Arduino firmware
├── Scenarios/                    # CoppeliaSim scene files (.ttt)
│   ├── Scenario_1/, Scenario_2/, Scenario_3/
│   └── *.ttt
├── Kuongshun_car_model/          # CAD and STL files for the physical robot
├── Data/                         # Experiment video recordings (MP4)
├── compare_trajectories.py       # Plots real vs. simulated trajectories
└── 20260609_*/                   # Timestamped experimental run results
```

---

## Supported Robots

| Robot | Wheel Base | Notes |
|-------|-----------|-------|
| kuongshun | 13 cm | Primary differential robot |
| Pioneer | 11 cm | 4-wheel differential drive |
| Corobeu | — | VSSS-style small robot |
| robot_four_wheels_UDLA | — | Custom UDLA model |

---

## Bio-inspired Optimization Algorithms

All algorithms optimize the PID gains (Kp, Ki, Kd) by minimizing trajectory error in closed-loop simulation:

| # | Algorithm | Acronym |
|---|-----------|---------|
| 0 | Particle Swarm Optimization | PSO |
| 1 | Adaptive Repulsive PSO | ARPSO |
| 2 | Differential Evolution | DE |
| 3 | Artificial Bee Colony | ABC |
| 4 | Moth-Flame Optimization | MFO |
| 5 | Grey Wolf Optimizer | GWO |
| 6 | Whale Optimization Algorithm | WOA |
| 7 | Bat Algorithm | BAT |
| 8 | Artificial Hummingbird Algorithm | AHA |

---

## Usage

### Prerequisites

- **CoppeliaSim** (with Remote API enabled)
- **Python 3** with: `numpy`, `matplotlib`, `opencv-python`, `pyzmq`
- **Arduino IDE** (optional, for ESP32 firmware flashing)

---

### Mode A — Simulation Only

```bash
# 1. Open CoppeliaSim and load a scene from Scenarios/
# 2. Start the controller
python path_planning_kuongshun/Path_planning/path_tracking_kuongshun.py
```

The controller connects to CoppeliaSim on `127.0.0.1:19999`, executes PID-based trajectory tracking, and saves results to a CSV file.

---

### Mode B — PID Optimization

```bash
python path_planning_kuongshun/Online_training/online_training_robot.py
# Select an algorithm (0–8) when prompted
```

The selected metaheuristic iterates over the simulation to find optimal Kp, Ki, Kd values. Results are saved to `variables_{ALGORITHM}.csv`.

---

### Mode C — Multi-Robot

```bash
# Sequential
python path_planning_kuongshun/Path_planning/gen.py

# Parallel
python path_planning_kuongshun/Path_planning/gen-parallel.py
```

Launches multiple robot instances (Car_A, Car_B, Car_C) in a shared CoppeliaSim scene.

---

### Mode D — Real Hardware with Visual Correction

```bash
# Terminal 1 — ArUco pose tracking
python CoppeliaEsp/ArUco/live_v2.py

# Terminal 2 — Correction loop
python path_planning_kuongshun/Path_planning/corrector.py

# Terminal 3 — CoppeliaSim controller (reference)
python path_planning_kuongshun/Path_planning/path_tracking_kuongshun.py
```

`live_v2.py` detects ArUco markers on the robot and publishes its pose over ZMQ. `corrector.py` subscribes to that stream, compares it against the CoppeliaSim ground truth, and sends corrections to the ESP32 over the network.

---

### Mode E — Trajectory Comparison

```bash
python compare_trajectories.py
```

Reads CSV files from real and simulated runs and generates normalized comparative plots (PNG).

---

## Technology Stack

| Layer | Technology |
|-------|-----------|
| Simulation | CoppeliaSim + Remote API |
| Control | Python 3, PID + differential-drive kinematics |
| Optimization | Custom bio-inspired algorithms (Bioinspired.py) |
| Vision | OpenCV, ArUco markers |
| Communication | ZMQ (pose), UDP/TCP (ESP32 commands) |
| Hardware | ESP32, differential-drive robot |
| CAD | STL models (Chassis + Wheel) |

---

## Authors

- **Jaider Bautista Rodriguez** — Universidad de la Amazonia
- **Mario Andrés Pastrana Triana** — Universidade de Brasília
- **Jesus Pinto Lopera** — Professor, Universidad de la Amazonia
- **Daniel Mauricio Muñoz Arboleda** — Professor, Universidade de Brasília
