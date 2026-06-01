#!/usr/bin/env python3
"""
@file gen.py
@brief Generator/Base file to create and manage multiple kuongshun path tracking instances
        with different robot configurations.

This file serves as a dispatcher that instantiates multiple kuongshun controllers,
each configured with different object names from the CoppeliaSim simulation.

Authors:
    Generated from path_tracking_kuongshun.py
"""

from path_tracking_kuongshun import kuongshun
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("Generator")


def create_robot_instance(
    robot_id: int,
    robot_name: str,
    motor_left_name: str,
    motor_right_name: str,
    ball_name: str,
    pid_gains: tuple = (0.3432, 0.0001, 0.0001),
    delta_t: float = 0.05,
    csv_filename: str = None,
    max_iterations: int = 1200,
    port: int = 19999,
) -> kuongshun:
    """
    @brief Create and configure a single kuongshun robot instance.

    @param robot_id Identifier for this robot instance
    @param robot_name Name of the robot object in CoppeliaSim (e.g., "Car_A")
    @param motor_left_name Name of the left motor object (e.g., "left_motor_A")
    @param motor_right_name Name of the right motor object (e.g., "rigth_motor_A")
    @param ball_name Name of the ball object (e.g., "ball")
    @param pid_gains Tuple of (kp, ki, kd) PID parameters
    @param delta_t Control sample time in seconds
    @param csv_filename Optional filename to save trajectory
    @param max_iterations Maximum number of control iterations
    @param port CoppeliaSim remote API port

    @return kuongshun instance configured and ready to run
    """
    logger.info(f"Creating robot instance {robot_id}: {robot_name}")
    
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
    
    return controller


def run_single_robot(controller: kuongshun, robot_id: int, pid_gains: tuple, delta_t: float, csv_filename: str = None, max_iterations: int = 1200, port: int = 19999) -> None:
    """
    @brief Execute a single robot controller.

    @param controller kuongshun instance to run
    @param robot_id Identifier for logging
    @param pid_gains PID parameters
    @param delta_t Sample time
    @param csv_filename Optional trajectory output file
    @param max_iterations Max control loop iterations
    @param port CoppeliaSim API port
    """
    try:
        logger.info(f"Starting robot {robot_id}")
        controller.run(
            pid_gains=pid_gains,
            delta_t=delta_t,
            csv_filename=csv_filename,
            max_iterations=max_iterations,
            port=port,
        )
        logger.info(f"Robot {robot_id} finished successfully")
    except Exception as e:
        logger.error(f"Robot {robot_id} failed: {e}")


def main():
    """
    @brief Main entry point: create and run multiple robot instances.
    """
    # Example configuration for multiple robots
    robots_config = [
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
        #     "ball_name": "ball",
        #     "csv_filename": "robot_3_trajectory.csv",
        #     "port": 20001,
        # },
    ]

    # PID gains
    pid_gains = (0.3432, 0.0001, 0.0001)
    delta_t = 0.05
    max_iterations = 1200

    logger.info("Generator: Starting multiple robot instances")

    # Create instances for all robots
    controllers = {}
    for config in robots_config:
        robot_id = config["robot_id"]
        controller = create_robot_instance(
            robot_id=robot_id,
            robot_name=config["robot_name"],
            motor_left_name=config["motor_left_name"],
            motor_right_name=config["motor_right_name"],
            ball_name=config["ball_name"],
            pid_gains=pid_gains,
            delta_t=delta_t,
            csv_filename=config["csv_filename"],
            max_iterations=max_iterations,
            port=config["port"],
        )
        controllers[robot_id] = controller

    # Run all robots sequentially
    logger.info(f"Configured {len(controllers)} robot instances. Starting sequential execution...")
    
    for robot_id in sorted(controllers.keys()):
        controller = controllers[robot_id]
        run_single_robot(
            controller=controller,
            robot_id=robot_id,
            pid_gains=pid_gains,
            delta_t=delta_t,
            csv_filename=robots_config[robot_id - 1]["csv_filename"],
            max_iterations=max_iterations,
            port=robots_config[robot_id - 1]["port"],
        )

    logger.info("Generator: All robots finished")


if __name__ == "__main__":
    main()
