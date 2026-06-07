#!/usr/bin/env python3
"""
@file gen-parallel.py
@brief Generator/Base file to create and manage multiple kuongshun path tracking instances
        with different robot configurations - PARALLEL VERSION.

This file serves as a dispatcher that instantiates multiple kuongshun controllers,
each configured with different object names from the CoppeliaSim simulation.
This version runs all robots in parallel using ThreadPoolExecutor.

Authors:
    Generated from path_tracking_kuongshun.py
"""


from path_tracking_kuongshun import kuongshun
import logging
from multiprocessing import Process  

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("Generator")

def run_single_robot(robot_id, robot_name, motor_left_name, motor_right_name, ball_name, pid_gains, delta_t, csv_filename, max_iterations, port):

    logging.basicConfig(level=logging.INFO)
    log = logging.getLogger(f"Robot_{robot_id}")
    
    try:
        log.info(f"Iniciando robot {robot_id} en puerto {port}")
        controller = kuongshun(
            v_max=15.0,
            v_min=-15.0,
            v_linear=10.0,
            ideal_goleiro_x=-0.6,
            sel_position=1,
            robot_name=robot_name,
            motor_left_name=motor_left_name,
            motor_right_name=motor_right_name,
            ball_name=ball_name,
        )
        controller.run(
            pid_gains=pid_gains,
            delta_t=delta_t,
            csv_filename=csv_filename,
            max_iterations=max_iterations,
            port=port,
        )
        log.info(f"Robot {robot_id} terminó")
    except Exception as e:
        log.error(f"Robot {robot_id} falló: {e}")

def main():
    robots_config = [
        #   {
        #     "robot_id": 1,
        #     "robot_name": "Turtlebot",
        #     "motor_left_name": "left_motor",
        #     "motor_right_name": "rigth_motor",
        #     "ball_name": "ball",
        #     "csv_filename": "robot_1_trajectory.csv",
        #     "port": 19999,
        # },
        {
            "robot_id": 1,
            "robot_name": "Car_A",
            "motor_left_name": "left_motor_A",
            "motor_right_name": "rigth_motor_A",
            "ball_name": "ball_A",
            "csv_filename": "robot_1_trajectory.csv",
            "port": 19999,
        },
        # {
        #     "robot_id": 2,
        #     "robot_name": "Car_B",
        #     "motor_left_name": "left_motor_B",
        #     "motor_right_name": "rigth_motor_B",
        #     "ball_name": "ball_B",
        #     "csv_filename": "robot_2_trajectory.csv",
        #     "port": 20000,
        # },
        # {
        #     "robot_id": 3,
        #     "robot_name": "Car_C",
        #     "motor_left_name": "left_motor_C",
        #     "motor_right_name": "rigth_motor_C",
        #     "ball_name": "ball_C", 
        #     "csv_filename": "robot_3_trajectory.csv",
        #     "port": 20001,
        # },
    ]

    pid_gains = (0.3432, 0.0001, 0.0001)
    delta_t = 0.05
    max_iterations = 1200

    
    processes = []
    for config in robots_config:
        p = Process(
            target=run_single_robot,
            args=(
                config["robot_id"],
                config["robot_name"],
                config["motor_left_name"],
                config["motor_right_name"],
                config["ball_name"],
                pid_gains,
                delta_t,
                config["csv_filename"],
                max_iterations,
                config["port"],
            )
        )
        processes.append(p)

    
    logger.info("Lanzando 3 robots en paralelo REAL...")
    for p in processes:
        p.start()

    
    for p in processes:
        p.join()

    logger.info("Todos los robots terminaron")

if __name__ == "__main__":
    main()