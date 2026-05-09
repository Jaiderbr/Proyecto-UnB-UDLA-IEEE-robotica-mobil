"""
online_training_robot.py

This script defines a class representing the mobile robot and uses various bio-inspired optimization algorithms to automatically tune the PID controller gains (Kp, Ki, Kd).
Depending on the algorithm selected (PSO, ARPSO, DE, ABC, MFO, GWO, etc.), the code trains the PID parameters, computes additional filter values, and stores the results.
When executed as the main program, it runs multiple training experiments and saves all results to a CSV file.

Author:
    Mario Pastrana (mariopastrana403@gmail.com)
    EVA PROJECT - Universidade de Brasília

Version:
    0.0.1 (beta)

Release Date:
    Jan 23, 2023
"""

import numpy as np
import math as mat
import Bioinspired
import csv

class robot():
    """""
    Represents a robot controlled by a PID controller and capable of being trained with bio-inspired optimization algorithms.
    """

    def __init__(self):

        """""
        Initializes the robot object by setting default values for:

        PID gains (Kp, Ki, Kd)

        Sampling time

        Filter parameters

        Minimum fitness value obtained

        Convergence history vector
        """

        self.Ki = 0
        self.Kd = 0
        self.Kp = 0
        self.Sample_Time = 0.05
        self.unomenosalfa = 0
        self.alfa = 0
        self.Min_value = 0
        self.Conv_array = []

    def Trainig_PID(self, selAlg):

        """""
        Runs PID training using a selected bio-inspired optimization algorithm.
        Main responsibilities:

        * Creates an instance of the optimization class

        * Detects which algorithm the user requested

        * Sends algorithm-specific parameters (population size, bounds, iterations, etc.)
        to the corresponding function in Bioinspired

        * Obtains optimized PID gains from the algorithm result

        * Computes filter parameters based on controller poles

        * Stores best fitness and convergence profile for later analysis
        """

        ctf = Bioinspired.Bioinspired()

        if 'PSO_' in selAlg:
            ##### Parametros PSO ####
            S = 7
            N = 3
            maxIter = 20
            w0 = 0.9
            wf = 0.1
            c1 = 2.05
            c2 = 2.05
            vMax = 3
            vIni = vMax / 10
            xMax = 3
            xMin = 0.001
            ##### Parametros Funciones de transferencia ####
            Sample_time = 0.05
            ctf.PSO(S, N, maxIter, Sample_time, w0, wf, c1, c2, vMax, vIni, xMax, xMin)

        elif 'ARPSO' in selAlg:

            ##### Parametros ARPSO ####
            S = 7
            N = 3
            maxIter = 20
            w0 = 0.9
            wf = 0.1
            c1 = 2.05
            c2 = 2.05
            vMax = 3
            vIni = vMax / 10
            xMax = 2
            xMin = 0.001
            para_div = 4
            para_iter = 50  # em porcentagem
            Sample_time = 0.05
            ctf.ARPSO(S, N, maxIter, Sample_time, w0, wf, c1, c2, vMax, vIni, xMax, xMin, para_div, para_iter)

        elif 'ABC' in selAlg:
            ##### Parametros ABC ####
            S = 7
            N = 3
            Pm = 0.6
            Pl = 0.1
            maxIter = 20
            xMax = 2
            xMin = 0.001
            Sample_time = 0.05
            ctf.ABC(S, N, maxIter, Sample_time, xMax, xMin, Pm, Pl)

        elif 'DE' in selAlg:
            ##### Parametros ABC ####
            S = 7
            N = 3
            maxIter = 20
            xMax = 2
            xMin = 0.001
            Sample_time = 0.05
            F_mutation = 1.25
            crossover = 0.75
            ctf.DE(S, maxIter, N, F_mutation, crossover, xMin, xMax, Sample_time)

        elif 'MFO' in selAlg:
            ##### Parametros ABC ####
            S = 7
            N = 3
            maxIter = 20
            xMax = 2
            xMin = 0.001
            Sample_time = 0.05
            ctf.MFO(S, maxIter, N, Sample_time, xMin, xMax)

        elif 'GWO' in selAlg:
            ##### Parametros ABC ####
            S = 7
            N = 3
            maxIter = 20
            xMax = 2
            xMin = 0.001
            Sample_time = 0.05
            ctf.GWO(S, maxIter, N, Sample_time, xMin, xMax)

        elif 'WOA' in selAlg:
            ##### Parametros ABC ####
            S = 7
            N = 3
            maxIter = 20
            xMax = 2
            xMin = 0.001
            Sample_time = 0.05
            ctf.WOA(S, maxIter, N, Sample_time, xMin, xMax)

        elif 'BAT' in selAlg:
            ##### Parametros ABC ####
            S = 7
            N = 3
            maxIter = 20
            xMax = 2
            xMin = 0.001
            Sample_time = 0.05
            ctf.BAT(S, maxIter, N, Sample_time, xMin, xMax)

        elif 'AHA' in selAlg:
            ##### Parametros ABC ####
            S = 7
            N = 3
            maxIter = 20
            xMax = [2]
            xMin = [0.001]
            Sample_time = 0.05
            ctf.AHA(S, maxIter, N, Sample_time, xMin, xMax)



        self.Kp = ctf.ys[0]
        self.Ki = ctf.ys[1]
        self.Kd = ctf.ys[2]
        raizes = np.roots([self.Kd, self.Kp, self.Ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)
        self.unomenosalfa = mat.exp(-(self.Sample_Time / Filter_e))
        self.alfa = 1 - self.unomenosalfa
        self.Min_value = ctf.fitVector[-1]
        self.Conv_array = ctf.fitVector


if __name__ == "__main__":

    """""
    Executes only when the file runs directly.

    Responsibilities:
    
    * Selects one optimization algorithm to test
    * Creates a CSV file for results
    * Runs several independent training experiments
    * Records PID parameters, filtering values, and performance metrics
    * Tracks the best experiment across all runs
    * Writes each experiment’s results to the CSV file
    
    """

    algorithms = [
        "PSO_",
        "DE",
        "ABC",
        "ARPSO",
        "MFO",
        "GWO",
        "WOA",
        "BAT",
        "AHA"
    ]

    print("Select algorithm:")
    for i, alg in enumerate(algorithms):
        print(f"{i}: {alg}")

    choice = int(input("Enter number: "))
    alg_implemented = algorithms[choice]

    with open(f'variables_{alg_implemented}.csv', 'w', newline='') as csvfile:

        number_of_exp = 3
        best_min = 100000
        EVA_object = robot()
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(
            [f'Kp_{alg_implemented}', f'Ki_{alg_implemented}', f'Kd_{alg_implemented}', '1 - alfa', 'alfa', 'Num_exp', 'Min_Global_exp', 'Best_exp', 'Best_exp_min'])

        for i in range(number_of_exp):

            print(f'Experiment_{i}')
            EVA_object.Trainig_PID(alg_implemented)
            kp_BIO = EVA_object.Kp
            ki_BIO = EVA_object.Ki
            kd_BIO = EVA_object.Kd
            unomenosalfa_BIO = EVA_object.unomenosalfa
            alfa = EVA_object.alfa
            Min_global = EVA_object.Min_value
            if Min_global < best_min:
                best_exp = i
                best_min = Min_global

            csv_writer.writerow([kp_BIO, ki_BIO, kd_BIO, unomenosalfa_BIO, alfa, i, Min_global, best_exp, best_min])



