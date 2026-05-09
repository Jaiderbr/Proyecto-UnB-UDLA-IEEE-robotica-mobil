"""
Bioinspired.py

This code have a several bioisnpired algorithms

Author:
    Mario Pastrana (mariopastrana403@gmail.com)
    EVA/MARIA PROJECT - Universidade de Brasília

Version:
    0.0.1 (beta)

Release Date:
    MAY 25, 2023
"""

import numpy as np
import math as mt
from random import *
import follow_ball
import random
import numpy
import math
from solution import solution
import time
from torch import randperm



class Bioinspired:

    def __init__(self):
        self.fitVector = []
        self.ys = []
        self.interror = 0
        self.fant = 0
        self.infrared_sensor_bio = []


    def PSO(self, S, N, maxIter, Sample_time, w0, wf, c1, c2, vMax, vIni, xMax, xMin):

        '''
            Unpack the configuration parameters from key arguments.
        '''
        fitVector = []

        '''
            PSO Initializations
        '''
        w, dw = w0, (wf - w0) / maxIter
        x = xMin + (xMax - xMin) * np.random.rand(S, N)
        y, v = 1e10 * np.ones((S, N)), vIni * np.ones((S, N))
        fInd, k = 1e10 * np.ones(S), 1
        corobeu_class = follow_ball.Corobeu()
        '''
            PSO Main Loop
        '''
        print("PSO")
        while k <= maxIter:
            print(f'Iteration ==> {k}')
            '''
                Loop to find the best individual particle
            '''
            for i in range(S):

                #### Function for online training ####
                fx = corobeu_class.Robot_CRB(x[i, :], Sample_time)

                if fx < fInd[i]:
                    y[i, :] = x[i, :]
                    fInd[i] = fx

            '''
                Find the best overall particle from the swarm
            '''
            bestFit = min(fInd)
            self.fitVector.append(bestFit)
            p = np.where(fInd == bestFit)[0][0]
            self.ys = y[p, :]
            # print(self.ys)
            # print(bestFit)

            '''
                Particles' speed update using inertia factor.
            '''
            for j in range(N):
                for i in range(S):
                    u1, u2 = np.random.rand(), np.random.rand()
                    v[i, j] = w * v[i, j] + c1 * u1 * (y[i, j] - x[i, j]) + c2 * u2 * (self.ys[j] - x[i, j])
                    x[i, j] += v[i, j]

                    if x[i, j] > xMax: x[i, j] = xMax - (np.random.rand() * (xMax - xMin))
                    if x[i, j] < xMin: x[i, j] = xMin + (np.random.rand() * (xMax - xMin))
                    if v[i, j] > vMax: v[i, j] = vMax - (np.random.rand() * vMax)
                    if v[i, j] < -vMax: v[i, j] = -vMax + (np.random.rand() * vMax)
            k += 1
            w += dw

    def ARPSO(self, S, N, maxIter, Sample_time, w0, wf, c1, c2, vMax, vIni, xMax, xMin, para_div, para_iter):

        '''
            Unpack the configuration parameters from key arguments.
        '''
        fitVector = []

        '''
            PSO Initializations
        '''
        w, dw = w0, (wf - w0) / maxIter
        x = xMin + (xMax - xMin) * np.random.rand(S, N)
        y, v = 1e10 * np.ones((S, N)), vIni * np.ones((S, N))
        fInd, k = 1e10 * np.ones(S), 1
        corobeu_class = follow_ball.Corobeu()

        '''
            ARPSO Main Loop
        '''
        print("ARPSO")

        while k <= maxIter:
            # print(k)
            '''
                Loop to find the best individual particle
            '''
            for i in range(S):

                #### Function for online training ####
                fx = corobeu_class.Robot_CRB(x[i, :], Sample_time)

                # print("___________________")
                if fx < fInd[i]:
                    y[i, :] = x[i, :]
                    fInd[i] = fx

            '''
                Find the best overall particle from the swarm
            '''
            bestFit = min(fInd)
            self.fitVector.append(bestFit)
            p = np.where(fInd == bestFit)[0][0]
            self.ys = y[p, :]
            # print(self.ys)
            # print(bestFit)

            sum_media = 0
            sum_media_ant = 0
            media = []
            Num_div_ant = 0
            L = mt.sqrt(N * ((xMax - xMin) * (xMax - xMin)))
            ##### Div Artificial #####
            for i in range(N):
                for j in range(S):
                    sum_media = x[j][i]+sum_media_ant
                    sum_media_ant = sum_media

                media.append(sum_media/N)
                sum_media_ant = 0

            for i in range(S):
                for j in range(N):
                    Vale_mean = (x[i][j]-media[j]) * (x[i][j]-media[j])
                Num_div = mt.sqrt(Vale_mean)+Num_div_ant
                Num_div_ant = Num_div
            Den_div = S * L
            diversidade = ((Num_div) / (Den_div))

            if (diversidade <= para_div) and (k <= ((maxIter*para_iter)/100)):
                sig = -1
            else:
                sig = 1

            ##########################
            '''
                Particles' speed update using inertia factor.
            '''
            for j in range(N):
                for i in range(S):
                    u1, u2 = np.random.rand(), np.random.rand()
                    v[i, j] = w * v[i, j] + sig * (c1 * u1 * (y[i, j] - x[i, j]) + c2 * u2 * (self.ys[j] - x[i, j]))
                    v[i, j] = min(vMax, max(-vMax, v[i, j]))

                    if x[i, j] > xMax: x[i, j] = xMax - (np.random.rand() * (xMax - xMin))
                    if x[i, j] < xMin: x[i, j] = xMin + (np.random.rand() * (xMax - xMin))
                    if v[i, j] > vMax: v[i, j] = vMax - (np.random.rand() * vMax)
                    if v[i, j] < -vMax: v[i, j] = -vMax + (np.random.rand() * vMax)

            k += 1
            w += dw

    def ABC(self, S, N, maxIter, Sample_time, xMax, xMin, Pm, Pl):

        print("Algorithm ABC")
        foodNumber = int(S/2)
        Range = np.tile(xMax - xMin, (foodNumber, 1))
        lower = np.tile(xMin, (foodNumber, 1))
        foods = np.random.rand(foodNumber, N) * Range + lower
        fitVector = []
        trial = np.zeros(foodNumber)
        _iter = 1
        bas = np.ones(foodNumber)
        globalMin = 1e50
        corobeu_class = follow_ball.Corobeu()
        objVal = np.array([corobeu_class.Robot_CRB(foods[i, :], Sample_time) for i in range(foodNumber)])

        for k in range(len(objVal)):
            if objVal[k] < globalMin:
                globalMin = objVal[k]
                globalParams = foods[k, :].copy()

        while _iter <= maxIter:

            '''
                Employed Bee phase
            '''
            for i in range(foodNumber):
                param = int(np.fix(N * np.random.rand()))

                while True:
                    neighbour = int(np.fix(foodNumber * np.random.rand()))
                    if neighbour != i: break

                sol = foods[i, :].copy()
                # print('Prev',sol)
                sol[param] = foods[i, param] + (foods[i, param] - foods[neighbour, param]) * np.random.uniform(-1,
                                                                                                               1.00000001)
                if sol[param] < xMin:
                    sol[param] = xMin * randint(0, 1)
                if sol[param] > xMax:
                    sol[param] = xMax * randint(0, 1)

                ### Only training ###
                objValSol = corobeu_class.Robot_CRB(sol, Sample_time)
                # print(sol,objValSol)
                if objValSol < objVal[i]:
                    foods[i, :] = sol.copy()
                    objVal[i] = objValSol
                    trial[i] = 0
                else:
                    trial[i] = trial[i] + 1

            # Probability calculator
            Pr = Pm * (objVal / np.max(objVal)) + Pl

            '''
                Onlooker Bee phase
            '''
            # i, t = 0, 0
            # while t < foodNumber:
            for i in range(foodNumber):
                if np.random.rand() > Pr[i]:
                    # t += 1

                    param = int(np.fix(N * np.random.rand()))

                    while True:
                        neighbour = int(np.fix(foodNumber * np.random.rand()))
                        if neighbour != i: break

                    sol = foods[i, :].copy()
                    sol[param] = foods[i, param] + (foods[i, param] - foods[neighbour, param]) * np.random.uniform(
                        -1, 1.00000001)

                    if sol[param] < xMin:
                        sol[param] = xMin * randint(0, 1)
                    if sol[param] > xMax:
                        sol[param] = xMax * randint(0, 1)

                    objValSol = corobeu_class.Robot_CRB(sol, Sample_time)

                    if objValSol < objVal[i]:
                        foods[i, :] = sol.copy()
                        objVal[i] = objValSol
                        trial[i] = 0
                    else:
                        trial[i] = trial[i] + 1

            for k in range(len(objVal)):
                # print(globalParams,target(globalParams),globalMin)
                if objVal[k] < globalMin:
                    globalMin = objVal[k]
                    globalParams = foods[k, :].copy()


            ind = np.where(trial == max(trial))[0][-1]
            fitVector.append(globalMin)

            _iter += 1
            print(_iter)

        self.ys = globalParams
        self.fitVector = fitVector

    def DE(self, S, max_iter, N, F_mutation, crossover, xmin, xmax, Sample_time):

        print("DE algorithm")

        def mutation(x, S, F, i):
            dir = 1
            R = np.random.permutation(S)
            j, k, p, u, v = R[:5]
            if j == i:
                j = R[5]
            elif k == i:
                k = R[5]
            elif p == i:
                p = R[5]
            elif u == i:
                u = R[5]
            elif v == i:
                v = R[5]

            return x[j, :] + dir * F * (x[k, :] - x[p, :])

        maxGens = max_iter
        F = F_mutation
        C = crossover

        assert (S > 6), "Population is too small."

        xmin = xmin
        xmax = xmax

        fitVector = []

        Range = np.tile(xmax - xmin, (S, 1))
        lower = np.tile(xmin, (S, 1))
        x = np.random.rand(S, N) * Range + lower
        curIter = 1
        U, y = np.zeros(N), np.zeros(S)
        corobeu_class = follow_ball.Corobeu()

        while curIter <= maxGens:

            for s in range(S):
                '''
                    Mutation phase
                '''
                V = mutation(x, S, F, s)
                V = np.maximum(xmin, np.minimum(xmax, V))

                '''
                    Crossover phase
                '''
                selec = int(np.floor(np.random.rand() * N))
                for n in range(N):
                    if np.random.rand() < C or n == selec:
                        U[n] = V[n]
                    else:
                        U[n] = x[s, n]

                '''
                    Selection phase
                '''
                #### Function for online training ####


                # Kinemactics TF
                # fx = SISOFunct.EVA_Kinematics_PID(x[i, :], time_simulation, Sample_time, SP)
                # Dynamic TF
                fx_U = corobeu_class.Robot_CRB(U[:], Sample_time)
                fx_X = corobeu_class.Robot_CRB(x[s, :], Sample_time)

                if fx_U < fx_X:
                    Tr = U[:].copy()
                else:
                    Tr = x[s, :].copy()

                x[s, :] = Tr[:].copy()
                y[s] = corobeu_class.Robot_CRB(x[s, :], Sample_time)

            bestFit = np.min(y)
            indF = np.where(y == bestFit)[0][0]

            fitVector.append(bestFit)
            xo = x[indF, :].copy()
            curIter += 1
            print(f'Iteration {curIter}')
            self.ys = xo
            self.fitVector =fitVector

    def initialization(self, SearchAgents_no, dim, ub, lb):

        Boundary_no = 1  # number of boundaries

        # If the boundaries of all variables are equal and user enters a single number for both ub and lb
        if Boundary_no == 1:
            X = np.random.rand(SearchAgents_no, dim) * (ub - lb) + lb

        # If each variable has a different lb and ub
        if Boundary_no > 1:
            X = np.zeros((SearchAgents_no, dim))
            for i in range(dim):
                ub_i = ub[i]
                lb_i = lb[i]
                X[:, i] = np.random.rand(SearchAgents_no) * (ub_i - lb_i) + lb_i

        return X

    def MFO(self, N, Max_iteration, dim, Sample_time, lb, ub):
        print("MFO algorithm")
        Moth_pos = self.initialization(N, dim, ub, lb)
        stopMOF = 0  # Condição de parada em estagnação
        lastBestScore = 0
        Convergence_curve = np.zeros(Max_iteration)

        Iteration = 1
        Moth_fitness = np.zeros(Moth_pos.shape[0])
        corobeu_class = follow_ball.Corobeu()

        # Main loop
        while Iteration < Max_iteration + 1:
            # Number of flames Eq. (3.14) in the paper
            Flame_no = round(N - Iteration * ((N - 1) / Max_iteration))

            for i in range(Moth_pos.shape[0]):
                # Check if moths go out of the search space and bring it back
                Flag4ub = Moth_pos[i, :] > ub
                Flag4lb = Moth_pos[i, :] < lb
                Moth_pos[i, :] = (
                        Moth_pos[i, :] * (~(Flag4ub + Flag4lb)) + ub * Flag4ub + lb * Flag4lb
                )

                #### Online training robot ####
                Moth_fitness[i] = corobeu_class.Robot_CRB(Moth_pos[i, :], Sample_time)

            if Iteration == 1:
                # Sort the first population of moths
                fitness_sorted, I = np.sort(Moth_fitness), np.argsort(Moth_fitness)
                sorted_population = Moth_pos[I, :]

                # Update the flames
                best_flames = sorted_population
                best_flame_fitness = fitness_sorted
            else:
                # Sort the moths
                double_population = np.concatenate((previous_population, best_flames), axis=0)
                double_fitness = np.concatenate((previous_fitness, best_flame_fitness))
                I = np.argsort(double_fitness)
                double_sorted_population = double_population[I, :]

                fitness_sorted = double_fitness[I[:N]]
                sorted_population = double_sorted_population[:N, :]

                # Update the flames
                best_flames = sorted_population
                best_flame_fitness = fitness_sorted

            # Update the position best flame obtained so far
            Best_flame_score = fitness_sorted[0]
            Best_flame_pos = sorted_population[0, :]

            previous_population = Moth_pos
            previous_fitness = Moth_fitness

            # A linearly dicreases from -1 to -2 to calculate t in Eq. (3.12)
            a = -1 + Iteration * ((-1) / Max_iteration)

            for i in range(Moth_pos.shape[0]):
                for j in range(Moth_pos.shape[1]):
                    if i <= Flame_no:  # Update the position of the moth with respect to its corresponsing flame
                        # D in Eq. (3.13)
                        distance_to_flame = abs(sorted_population[i, j] - Moth_pos[i, j])
                        b = 1
                        t = (a - 1) * np.random.rand() + 1

                        # Eq. (3.12)
                        Moth_pos[i, j] = (
                                distance_to_flame * np.exp(b * t) * np.cos(t * 2 * np.pi)
                                + sorted_population[i, j]
                        )
                    if i > Flame_no:  # Update the position of the moth with respct to one flame
                        # Eq. (3.13)
                        distance_to_flame = abs(sorted_population[i, j] - Moth_pos[i, j])
                        b = 1
                        t = (a - 1) * np.random.rand() + 1

                        # Eq. (3.12)
                        Moth_pos[i, j] = (
                                distance_to_flame * np.exp(b * t) * np.cos(t * 2 * np.pi)
                                + sorted_population[Flame_no, j]
                        )

            Convergence_curve[Iteration - 1] = Best_flame_score

            # Display the iteration and best optimum obtained so far
            if Iteration % 50 == 0:
                print(f"At iteration {Iteration}, the best fitness is {Best_flame_score}")

            Iteration += 1
            # print(f"Iteration ==> {Iteration}")
            self.ys = Best_flame_pos
            self.fitVector = Convergence_curve

        # return Best_flame_score, Best_flame_pos

    def GWO(self, SearchAgents_no, Max_iter, dim, Sample_time, lb, ub):
        print("GWO algorithm")
        # Max_iter=1000
        # lb=-100
        # ub=100
        # dim=30
        # SearchAgents_no=5

        # initialize alpha, beta, and delta_pos
        Alpha_pos = numpy.zeros(dim)
        Alpha_score = float("inf")

        Beta_pos = numpy.zeros(dim)
        Beta_score = float("inf")

        Delta_pos = numpy.zeros(dim)
        Delta_score = float("inf")
        corobeu_class = follow_ball.Corobeu()

        if not isinstance(lb, list):
            lb = [lb] * dim
        if not isinstance(ub, list):
            ub = [ub] * dim

        # Initialize the positions of search agents
        Positions = numpy.zeros((SearchAgents_no, dim))
        for i in range(dim):
            Positions[:, i] = (
                numpy.random.uniform(0, 1, SearchAgents_no) * (ub[i] - lb[i]) + lb[i]
            )

        Convergence_curve = numpy.zeros(Max_iter)
        s = solution()

        # Loop counter
        # print('GWO is optimizing  "' + objf.__name__ + '"')

        timerStart = time.time()
        s.startTime = time.strftime("%Y-%m-%d-%H-%M-%S")
        # Main loop
        for l in range(0, Max_iter):
            for i in range(0, SearchAgents_no):

                # Return back the search agents that go beyond the boundaries of the search space
                for j in range(dim):
                    Positions[i, j] = numpy.clip(Positions[i, j], lb[j], ub[j])

                #### Online training robot ####
                fitness= corobeu_class.Robot_CRB(Positions[i, :], Sample_time)
                # print(fx)
                # fitness = objf(Positions[i, :])

                # Update Alpha, Beta, and Delta
                if fitness < Alpha_score:
                    Delta_score = Beta_score  # Update delte
                    Delta_pos = Beta_pos.copy()
                    Beta_score = Alpha_score  # Update beta
                    Beta_pos = Alpha_pos.copy()
                    Alpha_score = fitness
                    # Update alpha
                    Alpha_pos = Positions[i, :].copy()

                if fitness > Alpha_score and fitness < Beta_score:
                    Delta_score = Beta_score  # Update delte
                    Delta_pos = Beta_pos.copy()
                    Beta_score = fitness  # Update beta
                    Beta_pos = Positions[i, :].copy()

                if fitness > Alpha_score and fitness > Beta_score and fitness < Delta_score:
                    Delta_score = fitness  # Update delta
                    Delta_pos = Positions[i, :].copy()

            a = 2 - l * ((2) / Max_iter)
            # a decreases linearly fron 2 to 0

            # Update the Position of search agents including omegas
            for i in range(0, SearchAgents_no):
                for j in range(0, dim):

                    r1 = random.random()  # r1 is a random number in [0,1]
                    r2 = random.random()  # r2 is a random number in [0,1]

                    A1 = 2 * a * r1 - a
                    # Equation (3.3)
                    C1 = 2 * r2
                    # Equation (3.4)

                    D_alpha = abs(C1 * Alpha_pos[j] - Positions[i, j])
                    # Equation (3.5)-part 1
                    X1 = Alpha_pos[j] - A1 * D_alpha
                    # Equation (3.6)-part 1

                    r1 = random.random()
                    r2 = random.random()

                    A2 = 2 * a * r1 - a
                    # Equation (3.3)
                    C2 = 2 * r2
                    # Equation (3.4)

                    D_beta = abs(C2 * Beta_pos[j] - Positions[i, j])
                    # Equation (3.5)-part 2
                    X2 = Beta_pos[j] - A2 * D_beta
                    # Equation (3.6)-part 2

                    r1 = random.random()
                    r2 = random.random()

                    A3 = 2 * a * r1 - a
                    # Equation (3.3)
                    C3 = 2 * r2
                    # Equation (3.4)

                    D_delta = abs(C3 * Delta_pos[j] - Positions[i, j])
                    # Equation (3.5)-part 3
                    X3 = Delta_pos[j] - A3 * D_delta
                    # Equation (3.5)-part 3

                    Positions[i, j] = (X1 + X2 + X3) / 3  # Equation (3.7)

            Convergence_curve[l] = Alpha_score

            # if l % 1 == 0:
                # print(["At iteration " + str(l) + " the best fitness is " + str(Alpha_score)])


        self.fitVector = Convergence_curve
        self.ys = Alpha_pos

    def WOA(self, SearchAgents_no, Max_iter, dim, Sample_time, lb, ub):
        print("WOA algorithm")
        if not isinstance(lb, list):
            lb = [lb] * dim
        if not isinstance(ub, list):
            ub = [ub] * dim

        # initialize position vector and score for the leader
        Leader_pos = numpy.zeros(dim)
        Leader_score = float("inf")  # change this to -inf for maximization problems
        corobeu_class = follow_ball.Corobeu()
        # Initialize the positions of search agents
        Positions = numpy.zeros((SearchAgents_no, dim))
        for i in range(dim):
            Positions[:, i] = (
                    numpy.random.uniform(0, 1, SearchAgents_no) * (ub[i] - lb[i]) + lb[i]
            )

        # Initialize convergence
        convergence_curve = numpy.zeros(Max_iter)

        ############################
        s = solution()

        timerStart = time.time()
        s.startTime = time.strftime("%Y-%m-%d-%H-%M-%S")
        ############################

        t = 0  # Loop counter

        # Main loop
        while t < Max_iter:
            for i in range(0, SearchAgents_no):

                # Return back the search agents that go beyond the boundaries of the search space

                # Positions[i,:]=checkBounds(Positions[i,:],lb,ub)
                for j in range(dim):
                    Positions[i, j] = numpy.clip(Positions[i, j], lb[j], ub[j])

                #### Online training robot ####
                fitness = corobeu_class.Robot_CRB(Positions[i, :], Sample_time)
                # print(fx)
                # fitness = objf(Positions[i, :])

                # Update the leader
                if fitness < Leader_score:  # Change this to > for maximization problem
                    Leader_score = fitness
                    # Update alpha
                    Leader_pos = Positions[
                                 i, :
                                 ].copy()  # copy current whale position into the leader position

            a = 2 - t * ((2) / Max_iter)
            # a decreases linearly fron 2 to 0 in Eq. (2.3)

            # a2 linearly decreases from -1 to -2 to calculate t in Eq. (3.12)
            a2 = -1 + t * ((-1) / Max_iter)

            # Update the Position of search agents
            for i in range(0, SearchAgents_no):
                r1 = random.random()  # r1 is a random number in [0,1]
                r2 = random.random()  # r2 is a random number in [0,1]

                A = 2 * a * r1 - a  # Eq. (2.3) in the paper
                C = 2 * r2  # Eq. (2.4) in the paper

                b = 1
                #  parameters in Eq. (2.5)
                l = (a2 - 1) * random.random() + 1  # parameters in Eq. (2.5)

                p = random.random()  # p in Eq. (2.6)

                for j in range(0, dim):

                    if p < 0.5:
                        if abs(A) >= 1:
                            rand_leader_index = math.floor(
                                SearchAgents_no * random.random()
                            )
                            X_rand = Positions[rand_leader_index, :]
                            D_X_rand = abs(C * X_rand[j] - Positions[i, j])
                            Positions[i, j] = X_rand[j] - A * D_X_rand

                        elif abs(A) < 1:
                            D_Leader = abs(C * Leader_pos[j] - Positions[i, j])
                            Positions[i, j] = Leader_pos[j] - A * D_Leader

                    elif p >= 0.5:

                        distance2Leader = abs(Leader_pos[j] - Positions[i, j])
                        # Eq. (2.5)
                        Positions[i, j] = (
                                distance2Leader * math.exp(b * l) * math.cos(l * 2 * math.pi)
                                + Leader_pos[j]
                        )

            convergence_curve[t] = Leader_score
            # if t % 1 == 0:
                # print(["At iteration " + str(t) + " the best fitness is " + str(Leader_score)])
            t = t + 1

        self.fitVector = convergence_curve
        self.ys = Leader_pos

    def BAT(self, N, Max_iteration, dim, Sample_time, lb, ub):
        print("BAT algorithm")
        n = N
        # Population size
        corobeu_class = follow_ball.Corobeu()
        if not isinstance(lb, list):
            lb = [lb] * dim
        if not isinstance(ub, list):
            ub = [ub] * dim
        N_gen = Max_iteration  # Number of generations

        A = 0.5
        # Loudness  (constant or decreasing)
        r = 0.5
        # Pulse rate (constant or decreasing)

        Qmin = 0  # Frequency minimum
        Qmax = 2  # Frequency maximum

        d = dim  # Number of dimensions

        # Initializing arrays
        Q = numpy.zeros(n)  # Frequency
        v = numpy.zeros((n, d))  # Velocities
        Convergence_curve = []

        # Initialize the population/solutions
        Sol = numpy.zeros((n, d))
        for i in range(dim):
            Sol[:, i] = numpy.random.rand(n) * (ub[i] - lb[i]) + lb[i]

        S = numpy.zeros((n, d))
        S = numpy.copy(Sol)
        Fitness = numpy.zeros(n)

        # initialize solution for the final results
        s = solution()

        # Initialize timer for the experiment
        timerStart = time.time()
        s.startTime = time.strftime("%Y-%m-%d-%H-%M-%S")

        # Evaluate initial random solutions
        for i in range(0, n):

            #### Online training robot ###
            Fitness[i] = corobeu_class.Robot_CRB(Sol[i, :], Sample_time)
            # print(fx)
            # Fitness[i] = objf(Sol[i, :])

        # Find the initial best solution and minimum fitness
        I = numpy.argmin(Fitness)
        best = Sol[I, :]
        fmin = min(Fitness)

        # Main loop
        for t in range(0, N_gen):

            # Loop over all bats(solutions)
            for i in range(0, n):
                Q[i] = Qmin + (Qmin - Qmax) * random.random()
                v[i, :] = v[i, :] + (Sol[i, :] - best) * Q[i]
                S[i, :] = Sol[i, :] + v[i, :]

                # Check boundaries
                for j in range(d):
                    Sol[i, j] = numpy.clip(Sol[i, j], lb[j], ub[j])

                # Pulse rate
                if random.random() > r:
                    S[i, :] = best + 0.001 * numpy.random.randn(d)

                #### Online training robot ####
                Fnew = corobeu_class.Robot_CRB(S[i, :], Sample_time)
                # Fnew = objf(S[i, :])

                # Update if the solution improves
                if (Fnew <= Fitness[i]) and (random.random() < A):
                    Sol[i, :] = numpy.copy(S[i, :])
                    Fitness[i] = Fnew

                # Update the current best solution
                if Fnew <= fmin:
                    best = numpy.copy(S[i, :])
                    fmin = Fnew

            # update convergence curve
            Convergence_curve.append(fmin)

            if t % 1 == 0:
                print(["At iteration " + str(t) + " the best fitness is " + str(fmin)])

        self.fitVector = Convergence_curve
        self.ys = best

    def space_bound(self, X, Up, Low):
        dim = len(X)
        S = (X > Up) + (X < Low)
        res = (np.random.rand(dim) * (np.array(Up) - np.array(Low)) + np.array(Low)) * S + X * (~S)
        return res

    def AHA(self, npop, max_it, dim, Sample_time, lb, ub):

        if len(lb) == 1:
            lb = lb * dim
            ub = ub * dim
        pop_pos = np.zeros((npop, dim))
        for i in range(dim):
            pop_pos[:, i] = np.random.rand(npop) * (ub[i] - lb[i]) + lb[i]
        pop_fit = np.zeros(npop)

        corobeu_class = follow_ball.Corobeu()

        for i in range(npop):
            #### Online training robot ###
            pop_fit[i] = corobeu_class.Robot_CRB(pop_pos[i, :], Sample_time)

        best_f = float('inf')
        best_x = []
        for i in range(npop):
            if pop_fit[i] <= best_f:
                best_f = pop_fit[i]
                best_x = pop_pos[i, :]
        his_best_fit = np.zeros(max_it)
        visit_table = np.zeros((npop, npop))
        diag_ind = np.diag_indices(npop)
        visit_table[diag_ind] = float('nan')
        for it in range(max_it):
            # Direction
            visit_table[diag_ind] = float('-inf')
            for i in range(npop):
                direct_vector = np.zeros((npop, dim))
                r = np.random.rand()
                # Diagonal flight
                if r < 1 / 3:
                    rand_dim = randperm(dim)
                    if dim >= 3:
                        rand_num = np.ceil(np.random.rand() * (dim - 2))
                    else:
                        rand_num = np.ceil(np.random.rand() * (dim - 1))

                    direct_vector[i, rand_dim[:int(rand_num)]] = 1
                # Omnidirectional flight
                elif r > 2 / 3:
                    direct_vector[i, :] = 1
                else:
                    # Axial flight
                    rand_num = np.ceil(np.random.rand() * (dim - 1))
                    direct_vector[i, int(rand_num)] = 1
                # Guided foraging
                if np.random.rand() < 0.5:
                    MaxUnvisitedTime = max(visit_table[i, :])
                    TargetFoodIndex = visit_table[i, :].argmax()
                    MUT_Index = np.where(visit_table[i, :] == MaxUnvisitedTime)
                    if len(MUT_Index[0]) > 1:
                        Ind = pop_fit[MUT_Index].argmin()
                        TargetFoodIndex = MUT_Index[0][Ind]
                    newPopPos = pop_pos[TargetFoodIndex, :] + np.random.randn() * direct_vector[i, :] * (
                            pop_pos[i, :] - pop_pos[TargetFoodIndex, :])
                    newPopPos = self.space_bound(newPopPos, ub, lb)

                    #### Online training robot ####

                    newPopFit = corobeu_class.Robot_CRB(newPopPos, Sample_time)
                    # newPopFit = ben_functions(newPopPos, fun_index)
                    if newPopFit < pop_fit[i]:
                        pop_fit[i] = newPopFit
                        pop_pos[i, :] = newPopPos
                        visit_table[i, :] += 1
                        visit_table[i, TargetFoodIndex] = 0
                        visit_table[:, i] = np.max(visit_table, axis=1) + 1
                        visit_table[i, i] = float('-inf')
                    else:
                        visit_table[i, :] += 1
                        visit_table[i, TargetFoodIndex] = 0
                else:
                    # Territorial foraging
                    newPopPos = pop_pos[i, :] + np.random.randn() * direct_vector[i, :] * pop_pos[i, :]
                    newPopPos = self.space_bound(newPopPos, ub, lb)
                    # Kinemactics TF
                    # newPopFit = SISOFunct.EVA_Kinematics_PID(newPopPos, time_simulation, Sample_time, SP)
                    # Dynamic TF
                    newPopFit = corobeu_class.Robot_CRB(newPopPos, Sample_time)
                    # newPopFit = ben_functions(newPopPos, fun_index)
                    if newPopFit < pop_fit[i]:
                        pop_fit[i] = newPopFit
                        pop_pos[i, :] = newPopPos
                        visit_table[i, :] += 1
                        visit_table[:, i] = np.max(visit_table, axis=1) + 1
                        visit_table[i, i] = float('-inf')
                    else:
                        visit_table[i, :] += 1
            visit_table[diag_ind] = float('nan')
            # Migration foraging
            if np.mod(it, 2 * npop) == 0:
                visit_table[diag_ind] = float('-inf')
                MigrationIndex = pop_fit.argmax()
                pop_pos[MigrationIndex, :] = np.random.rand(dim) * (np.array(ub) - np.array(lb)) + np.array(lb)
                visit_table[MigrationIndex, :] += 1
                visit_table[:, MigrationIndex] = np.max(visit_table, axis=1) + 1
                visit_table[MigrationIndex, MigrationIndex] = float('-inf')
                # Kinemactics TF
                # newPopFit = SISOFunct.EVA_Kinematics_PID(newPopPos, time_simulation, Sample_time, SP)
                # Dynamic TF
                pop_fit[MigrationIndex] = corobeu_class.Robot_CRB(pop_pos[MigrationIndex, :], Sample_time)
                # pop_fit[MigrationIndex] = ben_functions(pop_pos[MigrationIndex, :], fun_index)
                visit_table[diag_ind] = float('nan')
            for i in range(npop):
                if pop_fit[i] < best_f:
                    best_f = pop_fit[i]
                    best_x = pop_pos[i, :]
            his_best_fit[it] = best_f
        self.ys = best_x
        self.fitVector = his_best_fit
        # return best_x, best_f, his_best_fit