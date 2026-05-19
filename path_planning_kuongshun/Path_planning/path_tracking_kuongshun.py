#!/usr/bin/env python3
"""
@file path_tracking_VSSS.py
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
WHEEL_BASE = 13           # distance between wheels in cm (same units as in your sim)
WHEEL_RADIUS = 2.25          # radius of wheels in cm
DEFAULT_V_MAX = 15.0        # max linear speed in cm/s
DEFAULT_V_MIN = -15.0       # min linear speed in cm/s
DEFAULT_V_LINEAR = 10       # linear speed in cm/s

class kuongshun:
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
        robot_name: str = "Car_A",
        motor_left_name: str = "left_motor_A",
        motor_right_name: str = "rigth_motor_A",
        ball_name: str = "ball",
    ) -> None:
        """
        @brief Initialize controller internals.

        @param v_max Maximum wheel speed (saturation)
        @param v_min Minimum wheel speed (saturation)
        @param v_linear Linear velocity scaling factor for distance controller
        @param ideal_goleiro_x X location used when sel_position==0 (goalkeeper)
        @param sel_position 0 => "goleiro", 1 => "atacante" (affects target computation)
        @param robot_name Name of the robot object in CoppeliaSim
        @param motor_left_name Name of the left motor object in CoppeliaSim
        @param motor_right_name Name of the right motor object in CoppeliaSim
        @param ball_name Name of the ball object in CoppeliaSim
        """
        self.v_max = float(v_max)
        self.v_min = float(v_min)
        self.v_linear = float(v_linear)
        self.ideal_goleiro_x = float(ideal_goleiro_x)
        self.sel_position = int(sel_position)
        self.last_raw_phi = 0.0
        self.accumulated_offset = 0.0
        
        # object names
        self.robot_name = robot_name
        self.motor_left_name = motor_left_name
        self.motor_right_name = motor_right_name
        self.ball_name = ball_name

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

    def get_continuous(self, current_phi):
            """
            Calcula el ángulo continuo basado en el valor actual y el anterior.
            Evita saltos de pi a -pi.
            """
            # 1. Calculamos la diferencia bruta entre el dato nuevo y el anterior
            delta = current_phi - self.last_raw_phi

            # 2. Si la diferencia es mayor a PI, significa que saltó de PI a -PI
            #    entonces restamos 2*PI para compensar.
            if delta > math.pi:
                self.accumulated_offset -= 2 * math.pi
            
            # 3. Si la diferencia es menor a -PI, saltó de -PI a PI
            #    entonces sumamos 2*PI para compensar.
            elif delta < -math.pi:
                self.accumulated_offset += 2 * math.pi

            # 4. Actualizamos el último valor crudo para la siguiente iteración
            self.last_raw_phi = current_phi

            # 5. El ángulo continuo es el valor actual más todo lo acumulado
            return current_phi + self.accumulated_offset

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

        # obtain handles (blocking) using the object names provided in __init__
        _, robot = sim.simxGetObjectHandle(client_id, self.robot_name, sim.simx_opmode_blocking)
        _, motorL = sim.simxGetObjectHandle(client_id, self.motor_left_name, sim.simx_opmode_blocking)
        _, motorR = sim.simxGetObjectHandle(client_id, self.motor_right_name, sim.simx_opmode_blocking)
        _, ball = sim.simxGetObjectHandle(client_id, self.ball_name, sim.simx_opmode_blocking)

        # store
        self.handle_robot = robot
        self.handle_motor_left = motorL
        self.handle_motor_right = motorR
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

    def quaternion_to_euler(self, q):
        # Extraer los componentes del cuaternión
        x, y, z, w = q

        # Convertir a ángulos de Euler (en radianes)
        # Roll (X), Pitch (Y), Yaw (Z)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return [roll_x, pitch_y, yaw_z]  # Devolver en radianes

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
        motorL = self.handle_motor_left
        motorR = self.handle_motor_right

        # initialize PID internals
        integral_error_phi = 0.0
        prev_filtered_phi = 0.0
        integral_part_phi = 0.0
        prev_omega = 0.0

        # small safety: set motors to zero initially
        if motorL is not None and motorR is not None:
            sim.simxSetJointTargetVelocity(client, motorL, 0.0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(client, motorR, 0.0, sim.simx_opmode_blocking)

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
            # _, orientation = sim.simxGetObjectOrientation(client, robot, -1, sim.simx_opmode_blocking)
            _, quaternion = sim.simxGetObjectQuaternion(client, robot, -1, sim.simx_opmode_blocking)
            orientation = self.quaternion_to_euler(quaternion)

            if robot_pos is None or ball_pos is None or orientation is None:
                logger.debug("Waiting for valid streaming data (iter=%d)", iteration)
                time.sleep(delta_t)
                continue

            phi_robot = float(orientation[2])
            # compute planar distance to target

            # attacker: follow ball position
            err_dist = math.hypot(ball_pos[1] - robot_pos[1], ball_pos[0] - robot_pos[0])
            phid = math.atan2(ball_pos[1] - robot_pos[1], ball_pos[0] - robot_pos[0])

            # print("phi_robot: ", phi_robot)

          

            # stopping criterion based on distance
            if err_dist >= 0.2:
                linear_controller = self.v_linear * err_dist
                lock_stop_simulation = False
            else:
                linear_controller = 0.0
                lock_stop_simulation = True

            # adjust phid by +90deg as in original code
            phid = phid - 3.1416

            # wrap angles to avoid jumps
            # produce smallest signed error phid - phi_robot
            # phi_robot_smooth = self.get_continuous(phi_robot)
            # error_phi = self.get_continuous(phid - phi_robot)
            error_phi = phid - phi_robot
            error_phi = math.atan2(math.sin(error_phi), math.cos(error_phi))  # wrap to [-pi, pi]

            print("phi_robot: ", phi_robot)
            print("phid: ", phid)
            print("error_phi: ", error_phi)

            logger.debug("iter=%d phid=%.4f phi=%.4f err_phi=%.4f err_dist=%.4f", iteration, phid, phi_robot, error_phi, err_dist)

            # accumulate error magnitude (for logging / diagnostics)
            # original used acumulate_error; we keep a pos_error list
            self.pos_error.append(abs(error_phi))

            # PID compute
            omega, prev_filtered_phi, integral_error_phi, integral_part_phi = self.pid_phi(
                kpi, kii, kdi, delta_t, error_phi, integral_error_phi, prev_filtered_phi, integral_part_phi
            )


            # compute wheel speeds
            vl, vr, run_flag = self.compute_wheel_speeds(linear_controller, omega, lock_stop_simulation, abs(error_phi))

            # send speeds to motors
            if motorL is not None and motorR is not None:
                
                sim.simxSetJointTargetVelocity(client, motorL, vl, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(client, motorR, vr, sim.simx_opmode_blocking)
                
            # print("omega: ", omega)
            # print("speed_vr: ", vr)
            # print("speed_vl: ", vl)
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

    gains = (0.3432, 0.0001 , 0.0001)
    delta_t = 0.05
    filename = "kuongshun_experiment.csv"

    controller = kuongshun()
    try:
        controller.run(pid_gains=gains, delta_t=delta_t, csv_filename=filename, max_iterations=1200, port=19999)
    except Exception as e:
        logger.exception("Execution failed: %s", e)
