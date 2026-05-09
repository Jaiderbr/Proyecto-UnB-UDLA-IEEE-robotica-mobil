""""
follow_ball.py

This class present the PID controller implementation on the mobile robot follower ball, this class present five functions
the first function called init, make the initialization of the variables, the connect_CRB present the connection with
the coppeliaSim, Speed_CRB calculated the wheels speeds based on the robot topology (in this case is a differential robot),
PID_Controller_phi implement the PID based on the phi error (see video of the georgia tech course in this link:
https://www.youtube.com/watch?v=Lgy92yXiyqQ&list=PLp8ijpvp8iCvFDYdcXqqYU5Ibl_aOqwjr&index=14) finally, Robot_CRB is the 
principal function in this class.


Author:

    Mario Andres  Pastrana Triana (mario.pastrana@ieee.org)
    Matheus de Sousa Luiz (luiz.matheus@aluno.unb.br)
    EVA/MARIA PROJECT - University of Brasilia-(FGA)
    

Release Date:
    JUN 19, 2024

Finally comment:
    Querido lector por ahora, la correcta implementación de este código lo sabe Mario, Dios, la Virgen Maria y los santos
    esperemos que futuramente Mario continue sabiendo como ejecutar el código o inclusive que más personas se unan para
    que el conocimiento aqui depositado se pueda expandir de la mejor manera.
    Let's go started =)
"""""


import math
import sim
import numpy as np
import math as mat
import time
import csv
import sys
from pathlib import Path

# Import ArUco communicator if available
try:
    # Add path to ArUco module
    aruco_path = str(Path(__file__).parent.parent / "CoppeliaEsp" / "ArUco")
    if aruco_path not in sys.path:
        sys.path.insert(0, aruco_path)
    
    from aruco_communicator import ArucoDataSubscriber, create_default_aruco_data
    ARUCO_AVAILABLE = True
except ImportError:
    print("[Warning] ArUco communicator not available. Running without ArUco integration.")
    ARUCO_AVAILABLE = False
    
    # Create dummy functions
    class ArucoDataSubscriber:
        def __init__(self, *args, **kwargs):
            pass
        def subscribe(self, car_id):
            return None
    
    def create_default_aruco_data(detected=False):
        return {
            'x': 0.0, 'y': 0.0, 'angle': 0.0, 
            'detected': detected, 'distance_to_target': 0.0,
            'timestamp': time.time(), 'stale': False
        }

class robot_four_wheels_UDLA():
    """"
    robot_four_wheels_UDLA class is the principal class to controller the EVA positions robots.
        __init___               = Function to inicializate the global variables
        connect_CRB             = Present the connection with the coppeliaSim
        Speed_CRB               = Calculated the wheels speeds based on the robot topology (in this case is a differential robot)
        PID_Controller_phi      = Implement the PID based on the phi error
        Robot_four_wheels_UDLA  = The principal function in this class.
    """""

    def __init__(self):

        """"
            Initializations global variables
            self.y_out    (float list) = Out position robot in the y axis
            self.x_out    (Float list) = Out position robot in the x axis
            self.phi      (Float)   = phi robot value
            self.v_max    (Integer) = max speed for the robot
            self.v_min    (Integer) = min speed for the robot
            self.posError (Float list) = phi error
            self.sel_position (Integer) = 0 goleiro, 1 atacante
        """""

        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.v_max = 25
        self.v_min = -25
        self.v_linear = 10
        self.posError = []
        self.ideal_goleiro_x = -0.6
        self.sel_position = 1
        self.fusion_alpha = 0.3  # Sensor fusion weight: 30% ArUco, 70% Simulation
        
        # Initialize ArUco subscriber for sensor fusion
        if ARUCO_AVAILABLE:
            aruco_dir = str(Path(__file__).parent.parent / "CoppeliaEsp" / "ArUco")
            self.aruco_subscriber = ArucoDataSubscriber(data_dir=aruco_dir, timeout=2.0)
        else:
            self.aruco_subscriber = None

    def connect_robot(self, port):
        """""
        Function used to communicate with CoppeliaSim
            argument :
                Port (Integer) = used to CoppeliaSim (same CoppeliaSim)
                
            outputs : 
                clientID (Integer)  = Client number
                robot    (Integer)  = objecto robot
                MotorE   (Integer)  = Object motor left
                MotorD   (Integer)  = Object motor right
                ball     (Integer)  = Object ball on the scene
        """""

        ### Connect to coppeliaSim ###

        sim.simxFinish(-1)
        clientID_start = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

        if clientID_start != -1:
            # print("Conexion exitosa a CoppeliaSim")

            # Iniciar la simulación
            res = sim.simxStartSimulation(clientID_start, sim.simx_opmode_oneshot)
            time.sleep(2)

        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

        # Iniciar la simulación
        res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

        time.sleep(5)

        ### Return the objects ###

        _, robot = sim.simxGetObjectHandle(clientID, "Chasis_carro0", sim.simx_opmode_blocking)
        _, motorLF = sim.simxGetObjectHandle(clientID, "Left_front_motor", sim.simx_opmode_blocking)
        _, motorRF = sim.simxGetObjectHandle(clientID, "Rigth_front_motor", sim.simx_opmode_blocking)
        _, motorLR = sim.simxGetObjectHandle(clientID, "Left_rear_motor", sim.simx_opmode_blocking)
        _, motorRR = sim.simxGetObjectHandle(clientID, "Rigth_rear_motor", sim.simx_opmode_blocking)

        _, ball = sim.simxGetObjectHandle(clientID, "ball", sim.simx_opmode_blocking)
               
        return clientID, robot, motorLF, motorLR, motorRF, motorRR, ball, clientID
    
    def Speed_robot(self, U, omega, lock_stop_simulation, error_phi):

        """""
        Function used to calculate the speed for each wheel based on the topology robot
            argument :
                U              (Integer) = Max linear speed
                omega          (Float)   = Angular speed
                error_distance (Float)   = Error between the robot and the final point

            outputs : 
                vl (Float)   = Left speed
                vd (Float)   = right speed
                a  (Integer) = condition to simulate
        """""

        ### Calculate the speed based on the topology robot ###

        L = 11  # Distance between the wheels
        R = 3.25  # Wheel radio

        ### Calculate the speed based on the topology robot ###

        vd = ((2 * (U) + omega * L) / (2 * R))
        vl = ((2 * (U) - omega * L) / (2 * R))
        a = 1
        # print(f'vl === {vl} vd == {vd}')
        # print(f' omega == {omega}')
        ### the first time #####

        Max_Speed = self.v_max
        Min_Speed = self.v_min

        # Max_Speed = self.v_max
        # Min_Speed = self.v_min

        ### Saturation speed upper ###

        if vd >= Max_Speed:
            vd = Max_Speed
        if vd <= Min_Speed:
            vd = Min_Speed

        ### Saturation speed Lower ###

        if vl >= Max_Speed:
            vl = Max_Speed
        if vl <= Min_Speed:
            vl = Min_Speed

        ### When arrive to the goal ###

        if lock_stop_simulation == 1 and error_phi <= 0.08:
            a = 0
            vl = 0
            vd = 0

        ### Return values ###

        return vl, vd, a
    
    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):

        """""
        Function used to calculate the omega value (output PID) used on the speed function
            argument :
                kp              (Float) = Proportional constant used on the PID controller
                ki              (Float) = Integral constant used on the PID controller
                kd              (Float) = Derivative constant used on the PID controller
                deltaT          (Float) = Sample time
                error           (Float) = Phi error between the robot and the goal point
                interror        (Float) = Error Integral
                fant            (Float  = Used on the derivative part
                Integral_part   (Float) = Integral part

            outputs : 
               PID              (Float) = Omega value used on the speed function
               f                (Float) = f value use on the derivative part
               interror         (Float) = Error Integral
               Integral_part    (Float) = Integral part
        """""

        ### Max value of saturation in the integral part ###

        Integral_saturation = 10

        ### Find the filter e ####

        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        # print(f'mayor == {mayor}')
        Filter_e = 1 / (mayor * 10)

        ### Calculate the derivative part ###

        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror = interror + error
        f = unomenosalfaana * fant + alfaana * error
        if fant == 0:
            deerror = (f/deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        ### Calculate the integral part ###

        if Integral_part > Integral_saturation:
            Integral_part = Integral_saturation
        elif Integral_part < -Integral_saturation:
            Integral_part = -Integral_saturation
        else:
            Integral_part = ki * interror * deltaT

        ### Calculate the omega value (PID output) ###

        PID = kp * error + Integral_part + deerror * kd

        ### Return the principal values ###
        # print(f'Integral_part == {Integral_part}')
        return PID, f, interror, Integral_part

    def Robot_four_wheels_UDLA(self, x, deltaT, aruco_data=None):

        """""
        Principal function to simulate the follower ball robot with ArUco integration
            argument :
                x               (list)  = [kpi, kii, kdi] PID parameters
                deltaT          (Float) = Sample time
                aruco_data      (dict)  = ArUco detection data from live.py
                                         {
                                            'x': float (meters),
                                            'y': float (meters),
                                            'angle': float (degrees),
                                            'detected': bool,
                                            'distance_to_target': float
                                         }

            outputs : 
               fitness_function (float): Control fitness value
        """""
        kpi = x[0]
        kii = x[1]
        kdi = x[2]
        y_position_out = []
        x_position_out = []
        ### Get the objects within coppeliaSim using the connect_CRB function ###

        (clientID, robot, motorLF, motorLR, motorRF, motorRR, ball, clientID_start) = self.connect_robot(19999)

        ### Criterio to simulation ###
        # Constantes del PID
        raizes = np.roots([kdi, kpi, kii])
        acumulate_error = 0
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)
        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        omega_ant = 0
        a = 1
        angulo_anterior_phid = 0
        angulo_anterior_phi_robot = 0

        ### Init the principal values ###

        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        fant_phi = 0
        Integral_part_phi = 0


        ### Make the communication with coppeliaSim ###

        if (sim.simxGetConnectionId(clientID) != -1):

            ### Criterio to simulation ###

            while (a == 1):

                ### important to get valid values and init the phi value ###

                ### important to get valid values and init the phi value ###

                if Number_Iterations <= 1:

                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                    s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # self.phi = angle_robot[2] - 1.47
                    self.phi = 0
                    sim.simxSetJointTargetVelocity(clientID, motorLF, 0.0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorRF, 0.0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorLR, 0.0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorRR, 0.0, sim.simx_opmode_blocking)
                else:

                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                    # s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # s, angle_ball = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # self.phi = angle_robot[2] - 1.47
                    # Obtener la orientación del robot relativa al sistema de coordenadas global
                    returnCode, orientation = sim.simxGetObjectOrientation(clientID, robot, -1,
                                                                           sim.simx_opmode_blocking)

                    phi_robot = orientation[2]
                    self.phi = phi_robot
                    
                    # INTEGRACIÓN DIRECTA DE DATOS DE ARUCO
                    # Si recibimos datos de ArUco como parámetro, fusionar directamente
                    if aruco_data is not None and aruco_data.get('detected', False):
                        # Convertir ángulo de grados a radianes
                        aruco_angle_rad = math.radians(aruco_data['angle'])
                        
                        # Fusión de sensores: 0.3*ArUco + 0.7*Simulación
                        positiona_fused_x = self.fusion_alpha * aruco_data['x'] + \
                                           (1 - self.fusion_alpha) * positiona[0]
                        positiona_fused_y = self.fusion_alpha * aruco_data['y'] + \
                                           (1 - self.fusion_alpha) * positiona[1]
                        
                        # Manejo de wraparound para ángulos (radianes)
                        angle_diff = aruco_angle_rad - phi_robot
                        while angle_diff > math.pi:
                            angle_diff -= 2 * math.pi
                        while angle_diff < -math.pi:
                            angle_diff += 2 * math.pi
                        
                        phi_robot_fused = phi_robot + self.fusion_alpha * angle_diff
                        self.phi = phi_robot_fused
                        
                        # Log de fusión
                        print(f"[ArUco Fusion] ArUco: ({aruco_data['x']:.3f}, {aruco_data['y']:.3f}), "
                              f"Sim: ({positiona[0]:.3f}, {positiona[1]:.3f}), "
                              f"Fused: ({positiona_fused_x:.3f}, {positiona_fused_y:.3f})")
                    else:
                        # Si no hay datos de ArUco, usar solo simulación
                        positiona_fused_x = positiona[0]
                        positiona_fused_y = positiona[1]
                        phi_robot_fused = phi_robot

                    if self.sel_position == 1:
                        error_distance = math.sqrt((ballPos[1] - positiona_fused_y) ** 2 + (ballPos[0] - positiona_fused_x) ** 2)
                    elif self.sel_position == 0:
                        error_distance = math.sqrt(
                            (ballPos[1] - positiona_fused_y) ** 2 + ((self.ideal_goleiro_x) - positiona_fused_x) ** 2)

                    # print(f'Angle robot ==> {angle_robot}')

                    if error_distance >= 0.15:
                        ### Calculate the phid (see georgia tech course) ###
                        phid = math.atan2(ballPos[1] - positiona_fused_y, ballPos[0] - positiona_fused_x)
                        controller_Linear = self.v_linear * error_distance
                        lock_stop_simulation = 0

                    else:
                        # phid = 1.570   ## 90
                        phid = 0  ## 90
                        controller_Linear = 0
                        lock_stop_simulation = 1
                    ### Phi error to send the PID controller
                    phid = phid + 3.1516  # sum 180

                    # Calcula la diferencia entre el ángulo actual y el anterior
                    diferencia_phid = phid - angulo_anterior_phid
                    diferencia_phi = self.phi - angulo_anterior_phi_robot
                    # Si la diferencia es mayor que π, ajusta restando 2π
                    if diferencia_phid > math.pi:
                        phid -= 2 * math.pi
                    # Si la diferencia es menor que -π, ajusta sumando 2π
                    elif diferencia_phid < -math.pi:
                        phid += 2 * math.pi

                    # Si la diferencia es mayor que π, ajusta restando 2π
                    if diferencia_phi > math.pi:
                        self.phi -= 2 * math.pi
                    # Si la diferencia es menor que -π, ajusta sumando 2π
                    elif diferencia_phi < -math.pi:
                        self.phi += 2 * math.pi

                    # Actualiza el ángulo anterior
                    angulo_anterior_phid = phid
                    angulo_anterior_phi_robot = self.phi

                    print(f'phid == > {phid}, self.phi ==> {self.phi}, error ==> {phid - self.phi}')
                    error_phi = phid - self.phi
                    # print(f'Position robot ==> {error_phi}')
                    acumulate_error = acumulate_error + abs(phid - self.phi)
                    omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kpi, kii, kdi, deltaT,
                                                                                               error_phi, interror_phi,
                                                                                               fant_phi,
                                                                                               Integral_part_phi)

                    ### Calculate the distance error, the robot stop when arrive the ball ###

                    if omega >= 100 or omega <= -100:
                        omega = omega_ant
                    else:
                        omega_ant = omega

                    ### Acumulative distance error ###

                    self.posError.append(error_distance)

                    ### Implement the PID controller ###

                    ### Update the phi robot value ###

                    # self.phi = self.phi + omega * deltaT

                    ### Calculate the speed right and left based on the topology robot ###

                    #### Proportional controller of liner velocity ####

                    # controller_Linear = 1 / (self.v_linear * error_distance) ## for VSSS more speed in the end

                    vl, vd, a = self.Speed_robot(controller_Linear, omega, lock_stop_simulation,
                                               abs(phid - self.phi))

                    ### Send the speed values to coppeliasim simulato ###
                    sim.simxSetJointTargetVelocity(clientID, motorLF, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorRF, vd, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorLR, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorRR, vd, sim.simx_opmode_blocking)
                    # print(f'vl == {vl}, vd == {vd}, error_distance == {error_distance}, error_phi == {error_phi}')
                    ### update the time simulation and the simulation iteration

                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)
                y_position_out.append(positiona[1])
                x_position_out.append(positiona[0])
                if Number_Iterations == 50:
                    a = 0
                # time.sleep(0.5)
        ### Save the robot position ###ç
        # Detener la simulación
        res = sim.simxStopSimulation(clientID_start, sim.simx_opmode_oneshot)
        time.sleep(2)
        fitness_function = ((acumulate_error/Number_Iterations) + error_distance)
        return fitness_function

if __name__ == "__main__":

    crb01 = robot_four_wheels_UDLA()
    filename = "Corobeu_experiment.csv"
    ### DE  Experiment best 0 0.4172###
    # kpi_DE = [0.3629, 0.3609, 0.8000, 0.3746, 0.3432]
    # kii_DE = [0.1891, 0.3841, 0.0479, 0.0001, 0.0001]
    # kdi_DE = [0.0001, 0.0039, 0.0001, 0.0001, 0.0001]
    ### MFO  Experiment best 3 0.3736###
    # kpi_MFO = [0.3902, 0.3504, 0.3201, 0.3278, 0.3413]
    # kii_MFO = [0.3468, 0.2910, 0.0001, 0.0774, 0.1230]
    # kdi_MFO = [0.0001, 0.0050, 0.0001, 0.0002, 0.0049]
    kp_MFO = 0.7973
    ki_MFO = 0.2436
    kd_MFO = 0.0234
    x = [kp_MFO, ki_MFO, kd_MFO]
    deltaT = 0.05
    
    # Read ArUco data for sensor fusion (car_id = 3)
    car_id = 3
    aruco_data = None
    if crb01.aruco_subscriber is not None:
        aruco_data = crb01.aruco_subscriber.subscribe(car_id)
        if aruco_data is None or aruco_data.get('stale', True):
            # No ArUco data available or data is stale, use default
            aruco_data = create_default_aruco_data(detected=False)
            if ARUCO_AVAILABLE:
                print("[Warning] ArUco data not available or stale, using simulation-only mode")
    
    # Call Robot_four_wheels_UDLA with ArUco data
    crb01.Robot_four_wheels_UDLA(x, deltaT, aruco_data)