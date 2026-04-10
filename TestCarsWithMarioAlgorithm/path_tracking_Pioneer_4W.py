#!/usr/bin/env python3
"""
@file follow_ball_refactored.py
@brief PID follower robot controller for CoppeliaSim (refactored, documented for Doxygen)(see video of the georgia tech
 course in this link: https://www.youtube.com/watch?v=Lgy92yXiyqQ&list=PLp8ijpvp8iCvFDYdcXqqYU5Ibl_aOqwjr&index=14)

Authors:
    Mario Andres Pastrana Triana (mario.pastrana@ieee.org)
    Matheus de Sousa Luiz (luiz.matheus@aluno.unb.br)
    EVA/MARIA PROJECT - University of Brasilia (FGA)

Release Date:
    JUN 19, 2024 (refactored Nov 2025)

Note:
    Finally comment:
    Querido lector por ahora, la correcta implementación de este código lo sabe Mario, Dios, la Virgen Maria y los santos
    esperemos que futuramente Mario continue sabiendo como ejecutar el código o inclusive que más personas se unan para
    que el conocimiento aqui depositado se pueda expandir de la mejor manera.
    Let's go started =)
"""
from __future__ import annotations

import csv
import math
import logging
import time
from typing import Iterable, List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
import sim  # CoppeliaSim remote API

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("FollowBall")

# ---------------------------
# Configuration / Constants
# ---------------------------
WHEEL_BASE = 11    # distance between wheels in cm (same units as in your sim)
WHEEL_RADIUS = 3.25            # radius of wheels in cm
DEFAULT_V_MAX = 10.0     # max linear speed in cm/s
DEFAULT_V_MIN = -10.0    # min linear speed in cm/s
DEFAULT_V_LINEAR = 15.0   # linear speed in cm/s

class Pioneer:
    """
    @brief Controller class that implements PID-based heading control for a differential robot
           that follows a ball in CoppeliaSim.

    The class encapsulates:
      - connection to CoppeliaSim
      - PID controller for heading (phi)
      - wheel-speed computation for differential drive
      - data logging and plotting routines
    """


    def __init__(
        self,
        v_max: float = DEFAULT_V_MAX,
        v_min: float = DEFAULT_V_MIN,
        v_linear: float = DEFAULT_V_LINEAR,
        ideal_goleiro_x: float = -0.6,
        sel_position: int = 1,
    ) -> None:
        """
        @brief Initialize controller internals.

        @param v_max Maximum wheel speed (saturation)
        @param v_min Minimum wheel speed (saturation)
        @param v_linear Linear velocity scaling factor for distance controller
        @param ideal_goleiro_x X location used when sel_position==0 (goalkeeper)
        @param sel_position 0 => "goleiro", 1 => "atacante" (affects target computation)
        """
        self.v_max = float(v_max)
        self.v_min = float(v_min)
        self.v_linear = float(v_linear)
        self.ideal_goleiro_x = float(ideal_goleiro_x)
        self.sel_position = int(sel_position)

        # runtime data
        self.x_out: List[float] = []
        self.y_out: List[float] = []
        self.pos_error: List[float] = []

        # connection handles
        self.client_id: Optional[int] = None
        self.handle_robot: Optional[int] = None
        self.handle_motor_left: Optional[int] = None
        self.handle_motor_right: Optional[int] = None
        self.handle_ball: Optional[int] = None

    # ---------------------------
    # Connection helpers
    # ---------------------------
    def connect(self, port: int = 19999, host: str = "127.0.0.1", timeout_ms: int = 2000) -> bool:
        """
        @brief Establish connection to CoppeliaSim and fetch needed object handles.

        @param port TCP port used by CoppeliaSim remote API
        @param host Host address (default 127.0.0.1)
        @param timeout_ms Connection timeout in milliseconds

        @return True if connection and handles were successfully obtained, False otherwise
        """
        sim.simxFinish(-1)  # close all opened connections first
        client_id = sim.simxStart(host, port, True, True, timeout_ms, 5)
        if client_id == -1:
            logger.error("Failed to connect to CoppeliaSim on %s:%d", host, port)
            return False

        self.client_id = client_id

        # Modificado para robot udla

        # obtain handles (blocking)
        # _, robot = sim.simxGetObjectHandle(client_id, "Pioneer3AT", sim.simx_opmode_blocking)
        # _, motorLF = sim.simxGetObjectHandle(client_id, "front_left_wheel", sim.simx_opmode_blocking)
        # _, motorRF = sim.simxGetObjectHandle(client_id, "front_right_wheel", sim.simx_opmode_blocking)
        # _, motorLR = sim.simxGetObjectHandle(client_id, "rear_left_wheel", sim.simx_opmode_blocking)
        # _, motorRR = sim.simxGetObjectHandle(client_id, "rear_right_wheel", sim.simx_opmode_blocking)
        # _, ball = sim.simxGetObjectHandle(client_id, "ball", sim.simx_opmode_blocking)


        # obtain handles (blocking)
        _, robot = sim.simxGetObjectHandle(client_id, "Chasis_carro0", sim.simx_opmode_blocking)

        _, motorLF = sim.simxGetObjectHandle(client_id, "Left_front_motor", sim.simx_opmode_blocking)
        _, motorRF = sim.simxGetObjectHandle(client_id, "Rigth_front_motor", sim.simx_opmode_blocking)
        _, motorLR = sim.simxGetObjectHandle(client_id, "Left_rear_motor", sim.simx_opmode_blocking)
        _, motorRR = sim.simxGetObjectHandle(client_id, "Rigth_rear_motor", sim.simx_opmode_blocking)

        _, ball = sim.simxGetObjectHandle(client_id, "ball", sim.simx_opmode_blocking)

        # store
        self.handle_robot = robot
        self.handle_motor_left_front = motorLF
        self.handle_motor_right_front = motorRF
        self.handle_motor_left_rear = motorLR
        self.handle_motor_right_rear = motorRR
        self.handle_ball = ball

        logger.info("Connected to CoppeliaSim (client id: %d)", client_id)
        return True

    # ---------------------------
    # Control primitives
    # ---------------------------
    def compute_wheel_speeds(self, linear_speed: float, angular_speed: float, stop_if_close: bool, error_phi: float) -> Tuple[float, float, int]:
        """
        @brief Compute left and right wheel speeds for a differential drive.

        @param linear_speed Desired linear speed (U)
        @param angular_speed Desired angular speed (omega)
        @param stop_if_close If True and error_phi small then return zeros
        @param error_phi Absolute phi error (used to decide stop)

        @return (vl, vr, running_flag) where running_flag==0 means stop condition
        """
        L = WHEEL_BASE
        R = WHEEL_RADIUS

        # kinematic mapping
        vr = ((2 * linear_speed) + angular_speed * L) / (2 * R)
        vl = ((2 * linear_speed) - angular_speed * L) / (2 * R)

        # saturate
        vr = min(max(vr, self.v_min), self.v_max)
        vl = min(max(vl, self.v_min), self.v_max)

        running = 1
        if stop_if_close and error_phi <= 0.08:
            running = 0
            vr = 0.0
            vl = 0.0

        return float(vl), float(vr), running

    def pid_phi(
        self,
        kp: float,
        ki: float,
        kd: float,
        delta_t: float,
        error: float,
        integral_error: float,
        prev_filtered: float,
        integral_part: float,
    ) -> Tuple[float, float, float, float]:
        """
        @brief PID controller for heading (phi) with a small derivative filter.

        @param kp Proportional gain
        @param ki Integral gain
        @param kd Derivative gain
        @param delta_t Sample time (s)
        @param error Current phi error (target - actual)
        @param integral_error Accumulated error (for next call)
        @param prev_filtered Previous filtered value used to compute derivative
        @param integral_part Previous integral part value (for anti-windup logic)

        @return (pid_output, filtered_value, updated_integral_error, updated_integral_part)
        """
        # anti-windup / integral saturation
        INTEGRAL_SAT = 10.0

        # derive a small filter constant based on polynomial roots (as in original)
        # If kd, kp, ki are all zeros this will produce a degenerate case; protect against it.
        coeffs = np.array([kd, kp, ki], dtype=float)
        if np.allclose(coeffs, 0.0):
            filter_e = 1.0
        else:
            try:
                roots = np.roots(coeffs)
                mag = np.max(np.abs(roots))
                filter_e = 1.0 / max(1e-6, mag * 10.0)
            except Exception:
                filter_e = 1.0

        # discrete exponential filter for derivative
        exp_term = math.exp(-(delta_t / filter_e))
        alpha = 1.0 - exp_term

        # update integral error
        integral_error += error * delta_t

        # filtered value for derivative calculation
        filtered = exp_term * prev_filtered + alpha * error

        # derivative (safe)
        derivative = (filtered - prev_filtered) / max(delta_t, 1e-9)

        # compute integral part with saturation
        integral_part = ki * integral_error
        integral_part = max(min(integral_part, INTEGRAL_SAT), -INTEGRAL_SAT)

        # PID output
        pid_out = kp * error + integral_part + kd * derivative

        return float(pid_out), float(filtered), float(integral_error), float(integral_part)

    # ---------------------------
    # High-level runner
    # ---------------------------
    def run(
        self,
        pid_gains: Iterable[float],
        delta_t: float,
        csv_filename: Optional[str] = None,
        max_iterations: int = 1200,
        port: int = 19999,
    ) -> None:
        """
        @brief Main loop to run the follower robot.

        @param pid_gains Iterable of three floats [kp, ki, kd]
        @param delta_t Control sample time (s)
        @param csv_filename If provided, saved x,y trajectory will be written here
        @param max_iterations Maximum loop iterations to avoid infinite loops
        @param port Port for CoppeliaSim remote API connection
        @return None
        """
        kpi, kii, kdi = map(float, pid_gains)

        if not self.connect(port=port):
            raise RuntimeError("Cannot connect to CoppeliaSim. Aborting run().")

        # Prepare initial streaming for positions/orientation
        client = self.client_id
        assert client is not None and self.handle_robot is not None and self.handle_ball is not None
        robot = self.handle_robot
        ball = self.handle_ball
        motorLF = self.handle_motor_left_front
        motorRF = self.handle_motor_right_front
        motorLR = self.handle_motor_left_rear
        motorRR = self.handle_motor_right_rear

        # initialize PID internals
        integral_error_phi = 0.0
        prev_filtered_phi = 0.0
        integral_part_phi = 0.0
        prev_omega = 0.0

        # small safety: set motors to zero initially
        if motorLF is not None and motorRF is not None and motorLR is not None and motorRR is not None:
            sim.simxSetJointTargetVelocity(client, motorLF, 0.0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client, motorRF, 0.0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client, motorLR, 0.0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client, motorRR, 0.0, sim.simx_opmode_blocking)

        iteration = 0

        # start streaming positions
        sim.simxGetObjectPosition(client, robot, -1, sim.simx_opmode_streaming)
        sim.simxGetObjectPosition(client, ball, -1, sim.simx_opmode_streaming)
        sim.simxGetObjectOrientation(client, robot, -1, sim.simx_opmode_streaming)

        running = True
        while running and iteration < max_iterations:
            iteration += 1
            # read states (blocking where appropriate to ensure valid data)
            _, robot_pos = sim.simxGetObjectPosition(client, robot, -1, sim.simx_opmode_buffer)
            _, ball_pos = sim.simxGetObjectPosition(client, ball, -1, sim.simx_opmode_buffer)
            _, orientation = sim.simxGetObjectOrientation(client, robot, -1, sim.simx_opmode_blocking)

            # print(f'Orientation robot ==> {orientation}')

            if robot_pos is None or ball_pos is None or orientation is None:
                logger.debug("Waiting for valid streaming data (iter=%d)", iteration)
                time.sleep(delta_t)
                continue

            phi_robot = float(orientation[2])
            # compute planar distance to target

            err_dist = math.hypot(ball_pos[1] - robot_pos[1], ball_pos[0] - robot_pos[0])
            phid = math.atan2(ball_pos[1] - robot_pos[1], ball_pos[0] - robot_pos[0])

            # stopping criterion based on distance
            if err_dist >= 0.2:
                linear_controller = self.v_linear * err_dist
                lock_stop_simulation = False
            else:
                linear_controller = 0.0
                lock_stop_simulation = True

            # adjust phid by +90deg as in original code
            phid = phid - 1.5708*2

            # wrap angles to avoid jumps
            # produce smallest signed error phid - phi_robot
            error_phi = phid - phi_robot
            error_phi = math.atan2(math.sin(error_phi), math.cos(error_phi))  # wrap to [-pi, pi]

            logger.debug("iter=%d phid=%.4f phi=%.4f err_phi=%.4f err_dist=%.4f", iteration, phid, phi_robot, error_phi, err_dist)

            # accumulate error magnitude (for logging / diagnostics)
            # original used acumulate_error; we keep a pos_error list
            self.pos_error.append(abs(error_phi))

            # print(f'error_phi ==> {error_phi}')

            # PID compute
            omega, prev_filtered_phi, integral_error_phi, integral_part_phi = self.pid_phi(
                kpi, kii, kdi, delta_t, error_phi, integral_error_phi, prev_filtered_phi, integral_part_phi
            )

            # basic safety for omega (to avoid spikes)
            if abs(omega) >= 100.0:
                omega = prev_omega
            else:
                prev_omega = omega

            # compute wheel speeds
            vl, vr, run_flag = self.compute_wheel_speeds(linear_controller, omega, lock_stop_simulation, abs(error_phi))
            
            # vlAux,vrAux = 0.0, 0.0
            vvel = 0.5
            if iteration  < 15 :
                vlAux = vvel
                vrAux = vvel
            elif iteration >= 15 and iteration < 30:
                vlAux = -vvel
                vrAux = -vvel
                if iteration == 29:
                    iteration = 0
            
                
            print(f'vl ==> {vlAux} vr ==> {vrAux}')
            print(f'iteration ==> {iteration}')

            # send speeds to motors
            if motorLF is not None and motorRF is not None and motorLR is not None and motorRR is not None:

                # sim.simxSetJointTargetVelocity(client, motorLF, vl, sim.simx_opmode_blocking)
                # sim.simxSetJointTargetVelocity(client, motorRF, vr, sim.simx_opmode_blocking)
                # sim.simxSetJointTargetVelocity(client, motorLR, vl, sim.simx_opmode_blocking)
                # sim.simxSetJointTargetVelocity(client, motorRR, vr, sim.simx_opmode_blocking)

                sim.simxSetJointTargetVelocity(client, motorLF, vlAux, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client, motorRF, vrAux, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client, motorLR, vlAux, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client, motorRR, vrAux, sim.simx_opmode_blocking)




            # save position trace
            self.x_out.append(float(robot_pos[0]))
            self.y_out.append(float(robot_pos[1]))

            # stop if compute_wheel_speeds returned running==0
            if run_flag == 0:
                logger.info("Stop condition reached (run_flag==0). Breaking main loop.")
                running = False

            # small sleep to respect sample time
            time.sleep(delta_t)

        logger.info("Main loop finished after %d iterations", iteration)

        # stop simulation explicitly when finishing controller execution
        sim.simxStopSimulation(client, sim.simx_opmode_oneshot)
        time.sleep(0.5)

        # if CSV specified, save results
        if csv_filename:
            self.save_trajectory(csv_filename)

        # plot results (exclude first sample as requested)
        self.plot_trajectory(skip_first=True)

    # ---------------------------
    # I/O utilities
    # ---------------------------
    def save_trajectory(self, filename: str, skip_first: bool = True) -> None:
        """
        @brief Save x,y trajectory to CSV.

        @param filename Output CSV file path
        @param skip_first If True, omit the first recorded sample
        @return None
        """
        xs = self.x_out[1:] if skip_first else self.x_out[:]
        ys = self.y_out[1:] if skip_first else self.y_out[:]

        if len(xs) != len(ys):
            raise ValueError("x_out and y_out must have same length before saving.")

        with open(filename, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x_out", "y_out"])
            for x, y in zip(xs, ys):
                writer.writerow([x, y])
        logger.info("Trajectory saved to %s (%d points)", filename, len(xs))

    def plot_trajectory(self, skip_first: bool = True) -> None:
        """
        @brief Plot x vs y trajectory using matplotlib.

        @param skip_first If True, omit the first recorded sample
        @return None
        """
        xs = self.x_out[1:] if skip_first else self.x_out[:]
        ys = self.y_out[1:] if skip_first else self.y_out[:]

        if not xs or not ys:
            logger.warning("No trajectory data to plot.")
            return

        if len(xs) != len(ys):
            logger.error("Inconsistent trajectory lengths: xs=%d ys=%d", len(xs), len(ys))
            return

        plt.figure(figsize=(6, 5))
        plt.plot(xs, ys, marker="o", linestyle="-")
        plt.xlabel("x_out")
        plt.ylabel("y_out")
        plt.title("Robot trajectory (first sample omitted)" if skip_first else "Robot trajectory")
        plt.grid(True)
        plt.show()


# ---------------------------
# CLI-style entry point (for quick testing)
# ---------------------------
if __name__ == "__main__":
    # Example gains from your original script (MFO best)
    kpi_MFO = [0.3902, 0.3504, 0.3201, 0.3278, 0.3413]
    kii_MFO = [0.3468, 0.2910, 0.0001, 0.0774, 0.1230]
    kdi_MFO = [0.0001, 0.0050, 0.0001, 0.0002, 0.0049]

    gains = (kpi_MFO[4], kii_MFO[4], kdi_MFO[4])
    delta_t = 0.05
    filename = "Pioneer_experiment.csv"

    controller = Pioneer()
    try:
        controller.run(pid_gains=gains, delta_t=delta_t, csv_filename=filename, max_iterations=1200, port=19999)
    except Exception as e:
        logger.exception("Execution failed: %s", e)
