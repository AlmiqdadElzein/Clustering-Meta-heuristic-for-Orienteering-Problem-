#! /usr/bin/env python

import random
from random import randint
import math
import matplotlib.pyplot as plt
from math import sin, cos, pi, sqrt, ceil
import numpy as np
from copy import deepcopy
import itertools
from gurobipy import *
import matplotlib.pyplot as plt
from scipy.spatial import distance
import os
import time
import sys
from pyconcorde import solve_TSP
from linkern import solve_2opt
from pyEAOP import solve_EA_OP


def sumOverj(x,i,taskCount):
    return quicksum([x[i,j] for j in range(taskCount+1)])

def sumOveri(x,j,taskCount):
    return quicksum([x[i,j] for i in range(taskCount+1)])

# Get median of a list
def get_median(L):
    if L == []:
        return 0
    L.sort()
    n = len(L)
    if len(L) % 2 == 0:
        median1 = L[n//2] 
        median2 = L[n//2 - 1] 
        median = (median1 + median2)/2.0
    else: 
        median = L[n//2]
    return median

# Returns True if centers didn't change significantly
def reached_threshold(centers,centers_prev,threshold):
    if centers_prev[0] == 0:
        return False
    for j in range(len(centers)):
        if dist2(centers[j],centers_prev[j]) > threshold:
            return False
    return True 

# Divide tasks with unknown distributions in an nxm region
# into rectangular sub-regions such that each region has
# a given task limit
def divide_tasks_nonuniform(Tasks,task_nums,task_limit):
    threshold = 0.1
    # number of clusters in the environment
    if len(Tasks) % task_limit == 0:
        k = len(Tasks) / task_limit # 0 < k <= len(Tasks)
    else:
        k = len(Tasks) / task_limit + 1

    # pick k random points to be the centers of the clusters
    centers = [Tasks[randint(0,len(Tasks)-1)] for i in range(k)]
    task_regions = [[] for i in range(k)]
    centers_prev = [0 for i in range(k)]
    start_time = time.time()
    while not reached_threshold(centers,centers_prev,threshold) and (
                                    time.time() - start_time < 10):
        task_regions = [[] for i in range(k)]
        # for each point p in the environmnet
        for j in range(len(Tasks)):
            p = Tasks[j]
            num = task_nums[j]
            # put p in the non-full cluster with the nearest center
            min_dist = None
            chosen_region = None
            for z in range(len(centers)):
                if len(task_regions[z]) < task_limit:
                    if min_dist == None:
                        min_dist = dist2(p, centers[z])
                        chosen_region = z
                    else:
                        if dist2(p,centers[z]) < min_dist:
                            min_dist = dist2(p,centers[z])
                            chosen_region = z
            if chosen_region != None:
                task_regions[chosen_region].append(num)
        for c in range(k):
                centers_prev[c] = [centers[c][0],centers[c][1]]
        # Recalculate cluster centers if this is not the first iteration
        for l in range(len(task_regions)):
            # calculate median
            x_median = []
            y_median = []
            for T in task_regions[l]:
                x_median.append(Tasks[T][0])
                y_median.append(Tasks[T][1])
            x_m  = get_median(x_median)
            y_m = get_median(y_median)
            centers[l] = [x_m,y_m]
    return [task_regions]

# Returns a list of the total utilities of given
# regions
def get_region_med_utils(Task_regions,utilities):

    U_med = [0 for i in range(len(Task_regions))]
    S = [0 for j in range(len(Task_regions))]
    U = []
    for i in range(len(Task_regions)):
        for j in range(len(Task_regions[i])):
            if Task_regions[i][j] > 0:
                U.append(utilities[Task_regions[i][j] - 1])
        U_med[i] = get_median(U) * len(Task_regions[i])

    for i in range(len(Task_regions)):
        S[i] = U_med[i] * 1.0 / sum(U_med)

    return S

def get_task_to_add(solution, cost_M, utilities, timeBudget, T):
    Scores = [0 for i in range(len(utilities))]
    for i in range(len(utilities)):
        if i+1 in solution:
            Scores[i] = None
        else:
            Scores[i] = utilities[i] - cost_M[solution[-1]][i+1]
    task = None
    max_score = -2**30
    for j in range(len(Scores)):
        if Scores[j] != None and Scores[j] > max_score:
            task = j+1
            max_score = Scores[j]
    return task

def get_initial_solution(cost_M, utilities, timeBudget):
    solution = [0]
    T = 0
    while T < timeBudget and len(solution) < len(utilities) + 1:
        to_add = get_task_to_add(solution, cost_M, utilities, timeBudget, T)
        T += cost_M[solution[-1]][to_add]
        solution.append(to_add)
    if T > timeBudget:
        T -= cost_M[solution[-2]][solution[-1]]
        del solution[-1]

    assert(T <= timeBudget)
    t = 0
    for i in range(len(solution) - 1):
        pi = solution[i]
        pj = solution[i+1]
        t += cost_M[pi][pj]
    
    assert(abs(t - T) < 0.1)
    return solution

def find_opt_path(sp,taskCount,utilities,timeBudget,taskLocations, time_limit, H):
    assert(sp == 0)
    assert(taskCount == len(utilities))
    assert(taskCount + 1 == len(taskLocations))
    cost_M = [[0 for i in range(taskCount+1)] for j in range(taskCount+1)]
    for i in range(taskCount+1):
        for j in range(taskCount+1):
            p1 = taskLocations[i]
            p2 = taskLocations[j]
            if p1 == [-1, -1] or p2 == [-1, -1]:
                cost_M[i][j] = 0.0
            else:
                cost_M[i][j] = dist2(p1,p2)
    if H == 'ea4op':
        uts = utilities[:]
        uts.insert(0,0)
        path, total_time = solve_EA_OP(cost_M, uts, timeBudget, taskLocations)
        U = 0
        for i in range(len(path)):
            U += uts[path[i]]
        
        return [path, total_time]
    
    m = Model("UAV optimal path")
    m.params.OutputFlag = 0
    m.params.MIPGap = 0.05
    #m.params.NodefileStart = 0.000001
    #m.params.NodefileDir = '/home/elmiqdad/project_ws/src/uav/src/Nodes'
    m.params.TimeLimit = time_limit
    #m.params.Threads = 1
    #m.params.SolFiles = 'model_solutions/solution' 
    x = m.addVars(taskCount+1,taskCount+1,lb=0.0,ub=1.0,vtype=GRB.BINARY, name='x')
    u = m.addVars(taskCount+1,lb=2.0,ub=float(taskCount+1),vtype=GRB.INTEGER, name='u')
    #Define the objective and cost function
    Cost = 0
    for i in range(taskCount+1):
        for j in range(taskCount+1):
            p1 = taskLocations[i]
            p2 = taskLocations[j]
            Cost += x[i, j] * cost_M[i][j]

    initial_solution = get_initial_solution(cost_M, utilities, timeBudget)
    for i in range(taskCount+1):
        for j in range(taskCount+1):
            i_index = None
            j_index = None
            if i in initial_solution:
                i_index = initial_solution.index(i)
            if j in initial_solution:
                j_index = initial_solution.index(j)
            if i_index == None or j_index == None:
                x[i, j].Start = 0.0
            elif j_index == i_index + 1:
                x[i, j].Start = 1.0
            else:
                x[i, j].Start = 0.0
    for i in range(len(initial_solution) - 1):
        pi = initial_solution[i]
        pj = initial_solution[i+1]
        x[pi, pj].Start = 1.0
    obj = []
    for j in range(taskCount+1):
        if j != sp:
            obj.append(utilities[j-1] * sumOveri(x,j,taskCount))
    U = quicksum(obj)
    m.setObjective(U,GRB.MAXIMIZE)
    # Guarantee that starting location is left exactly once and is
    # never visited
    m.addConstr(sumOverj(x,sp,taskCount),sense=GRB.EQUAL,rhs=1)
    m.addConstr(sumOveri(x,sp,taskCount),sense=GRB.EQUAL,rhs=0)

    # Guarantee that every location is visited at most once and is left
    # at most once
    m.addConstrs((sumOverj(x,i,taskCount) <= 1 
                                                for i in range(taskCount+1)))

    m.addConstrs((sumOveri(x,j,taskCount) <= 1 
                                                for j in range(taskCount+1)))
    # Guarantee that every task is either never left or left after it was
    # visited

    m.addConstrs((2 * sumOveri(x,task,taskCount) - 
    sumOverj(x,task,taskCount) >= 0 for task in range(taskCount+1)
                                             if task != sp ))

    # subtour eliminating constraints
    n = taskCount 
    m.addConstrs((u[i] - u[j] + 1 <= (n-1)*(1-x[i,j]) 
                                        for i in range(1, taskCount+1)
                                        for j in range(1, taskCount+1)))
    # Guarantee that the path will finish according to the time budget
    m.addConstr(Cost,sense=GRB.LESS_EQUAL,rhs=timeBudget)
    m.optimize()
    if m.status != GRB.OPTIMAL:
        return [[], 0]
    i = 0
    values = []
    for v in m.getVars():
        if i < ((taskCount+1)**2): 
            values.append(round(v.X))
            if round(v.X) == 1:
                pass
        i += 1
    # extract a sequence of locations from the solution
    sequence = extract_seq(sp,values,taskCount,timeBudget,0)
    sequence[0][0].insert(0,0)
    total_time = 0
    for i in range(len(sequence[0][0]) - 1):
        pi = sequence[0][0][i]
        pj = sequence[0][0][i + 1]
        total_time += cost_M[pi][pj]

    assert(len(sequence[0]) == 1)
    return [sequence[0][0], total_time]

def extract_seq(sp,values,taskCount,timeBudget,totalTime):
    taskPairs = []
    for i in range(taskCount+1):
        for j in range(taskCount+1):
            if j != sp:
                taskPairs.append([i,j])

    total_time = totalTime
    nexti = sp
    list_index = 0
    sequence = [[]]
    onesCount = values.count(1.0)
    visitedCount = 0
    found = False
    cyclic = False
    end = False
    z = 0
    cycle_finished = False
    # untill all tasks are visite
    while visitedCount < onesCount:
        # if there's a variable that's equal to 1 and has i = nexti
        for k in range(len(taskPairs)):
            i = taskPairs[k][0]
            j = taskPairs[k][1]
            index = i * (taskCount+1) + j
            if i == nexti and values[index] == 1:
                found = True
                break

        assert(found)
        if found and not cycle_finished:
            # add the j of that variable to the sequence of index list index
            if j in sequence[list_index]:
                cycle_finished = True
            else:
                sequence[list_index].append(j)
                visitedCount += 1
                # make nexti equal to the j of the variable found
                nexti = taskPairs[k][1]

        found = False
    return [sequence,total_time]

def get_best_path(tsp_sol, cost_M):
    reverse = False
    if cost_M[tsp_sol[-1]][tsp_sol[0]] < cost_M[tsp_sol[0]][tsp_sol[1]]:
        reverse = True

    if reverse:
        better_path = [0]
        for i in range(len(tsp_sol) - 1):
            better_path.append(tsp_sol[len(tsp_sol) - 1 - i])

        c = 0
        for j in range(len(better_path) - 1):
            c += cost_M[better_path[j]][better_path[j+1]]
        return better_path, c

    c = 0
    for j in range(len(tsp_sol) - 1):
            c += cost_M[tsp_sol[j]][tsp_sol[j+1]]
    return tsp_sol, c

def reverse_array(A):
    A_r = []
    for i in range(len(A)):
        A_r.append(A[len(A) - i - 1])
    return A_r

# This heuristic is for solving the Rural Postman Problem
# The approach is to convert the Arc Routing Problem into
# a TSP by assigning random directions to the arc in each
# iteration
#
# The heuristic looks for solutions until a time limit has been exceede
def get_TSP_matrix_H2(region_solutions, region_costs, Tasks, time_limit):
    k = 0
    while k < len(region_costs):
        if region_costs[k] == 0:
            del region_costs[k]
            del region_solutions[k]
            k -= 1
        k += 1
    s = time.time()
    arc_directions = [1 for i in range(len(region_solutions))]
    best_solution = []
    best_directions = []
    tried_directions = []
    best_cost = 2**30
    comb_count = 0
    possible_arc_directions = 2 ** (len(region_solutions) - 1)
    while (time.time() - s < time_limit) and (len(tried_directions) < possible_arc_directions):
        # generate random directions for each arc
        # 0 if left-to-right, 1 of right-to-left
        arc_directions = [randint(0,1) for i in range(len(region_solutions))]
        comb_count += 1
        arc_directions[0] = 0
        # generate cost matrix
        cost_M = [[0 for i in range(len(region_solutions))]
                                                for j in range(len(region_solutions))]
        for i in range(len(cost_M)):
            for j in range(len(cost_M)):
                if i != j:
                    # 4 possibilities
                    # ------>------ and ------>----- 00
                    # ----->------ and ------<----- 01
                    # -----<----- and ------>----- 10
                    # ----<----- and ------<-----  11
                    arc1_dir = arc_directions[i]
                    arc2_dir = arc_directions[j]
                    if arc1_dir == 0 and arc2_dir == 0:
                        pi = Tasks[region_solutions[i][-1]]
                        pj = Tasks[region_solutions[j][0]]
                    elif arc1_dir == 0 and arc2_dir == 1:
                        pi = Tasks[region_solutions[i][-1]]
                        pj = Tasks[region_solutions[j][-1]]
                    elif arc1_dir == 1 and arc2_dir == 0:
                        pi = Tasks[region_solutions[i][0]]
                        pj = Tasks[region_solutions[j][0]]
                    else:
                        pi = Tasks[region_solutions[i][0]]
                        pj = Tasks[region_solutions[j][-1]]

                    cost = dist2(pi, pj)
                    cost_M[i][j] = cost

        tsp_sol = solve_TSP(cost_M)
        sol, cost = get_best_path(tsp_sol, cost_M)
        if cost < best_cost:
            best_cost = cost
            best_solution = sol[:]
            best_directions = arc_directions[:]

    optimal_path = []
    for i in range(len(best_solution)):
        sub_path = region_solutions[best_solution[i]]
        if best_directions[best_solution[i]] == 1:
            sub_path = reverse_array(sub_path)
        for s in sub_path:
            optimal_path.append(s)

    return optimal_path

# This heurisic is for solving the Rural Postman problem
# The endpoints of each arc are the nodes in a TSP
# the cost between the endpoints of the same arc is 1
# the cost berween endpoints of different arcs is the distance
# between those endpoints multiplied by some large factor K
def get_TSP_matrix_H1(region_solutions, region_costs, Tasks):
    K_factor = 9999
    k = 0
    while k < len(region_costs):
        if region_costs[k] == 0:
            del region_costs[k]
            del region_solutions[k]
            k -= 1
        k += 1

    M = [[0 for i in range(2*len(region_solutions))]
                                        for j in range(2*len(region_solutions))]
        
    for i in range(2*len(region_solutions)):
        for j in range(2*len(region_solutions)):
            if i != j:
                if i % 2 == 0 and j == i + 1:
                    r = i / 2
                    M[i][j] = 1
                elif j % 2 == 0 and i == j + 1:
                    r = j / 2
                    M[i][j] = 1
                else:
                    region_i = i / 2
                    region_j = j / 2
                    if i % 2 == 0:
                        pi = Tasks[region_solutions[region_i][0]]
                    else:
                        pi = Tasks[region_solutions[region_i][-1]]
                    if j % 2 == 0:
                        pj = Tasks[region_solutions[region_j][0]]
                    else:
                        pj = Tasks[region_solutions[region_j][-1]]

                    M[i][j] = dist2(pi, pj)*K_factor
    for i in range(len(M)):
        for j in range(len(M[i])):
            assert(M[i][j] == M[j][i])
    return M

def get_path_cost(Tasks, path):
    t = 0
    for i in range(len(path) - 1):
        pi = Tasks[path[i]]
        pj = Tasks[path[i+1]]
        t += dist2(pi, pj)
    return t

def node_to_remove(S):
    min_score = 2**30
    node = None
    for i in range(len(S)):
        if S[i][2] < min_score:
            min_score = S[i][2]
            node = S[i][1]
    return node

def repair(Tasks, optimal_path, self_timeBudget, utilities):
    total_cost = get_path_cost(Tasks, optimal_path)
    delta_U_list = []
    delta_C_list = []
    while total_cost > self_timeBudget:
        S = [0 for i in range(len(optimal_path) - 1)]
        for i in range(1,len(optimal_path)):
            if i == len(optimal_path) - 1:
                delta_C = dist2(Tasks[optimal_path[i-1]], 
                                                        Tasks[optimal_path[i]])
                delta_U = -utilities[optimal_path[i]-1]
            else:
                # for cost:
                #  A delete (i-1) -> (i) link
                A = dist2(Tasks[optimal_path[i-1]], 
                                                        Tasks[optimal_path[i]])
                #  B delete (i) -> (i+1) link
                B = dist2(Tasks[optimal_path[i]], 
                                                    Tasks[optimal_path[i+1]])
                #  C add (i-1) -> (i+1) link
                C = dist2(Tasks[optimal_path[i-1]], 
                                                    Tasks[optimal_path[i+1]])
                # delta_C = C - A - B
                delta_C = C - (A + B)
                # for utility:
                # delta_U = -utility of (i)
                delta_U = -utilities[optimal_path[i]-1]

            # S = delta_C - delta_U
            S[i-1] = [delta_C + delta_U, optimal_path[i], delta_C]
            
        to_remove = node_to_remove(S)
        del optimal_path[optimal_path.index(to_remove)]
        total_cost = get_path_cost(Tasks, optimal_path)
    
    return optimal_path, total_cost

def get_opt_path(self_taskLocations, self_utilities, self_timeBudget, task_limit, H):
    Tasks = taskLocations
    self_taskCount = len(taskLocations) - 1
    assert(self_taskCount == len(self_utilities))
    if self_taskCount > task_limit:
        startPoint = taskLocations[0]
        # Divide the set of tasks into smaller subsets of tasks
        print "Clustering tasks..."
        D = divide_tasks_nonuniform(Tasks,
                            [i for i in range(self_taskCount+1)], 
                            task_limit)
        print "Done clustering"
        Task_regions = D[0]
        # Find the region with the starting point
        startRegion = 0
        for i in range(len(Task_regions)):
            if 0 in Task_regions[i]:
                startRegion = i
                break

        region_order = [i for i in range(len(Task_regions))]
        # Compute for start region first
        # Divide the time budget across the subsets
        Budgets = [0 for i in range(len(Task_regions))]
        U = get_region_med_utils(Task_regions, self_utilities)
        execTimes = []
        for i in range(len(Budgets)):
            Budgets[i] = self_timeBudget * U[i]
        region_order[0] = startRegion
        region_order[startRegion] = 0
        T = Budgets[0]
        Budgets[0] = Budgets[startRegion]
        Budgets[startRegion] = T
        t = 0
        total_path = [0]
        savings = 0
        region_costs = [0 for i in range(len(Task_regions))]
        region_solutions = [0 for i in range(len(Task_regions))]
        region_times = []
        print "There are",len(Task_regions),"clusters"
        r = 0
        print "region order:",region_order
        for R_current in region_order:
            print "Computing path for cluster",R_current
            # Find path in current region accrding to the time budget left
            task_set = Task_regions[R_current]
            Locations = [0 for i in range(len(task_set))]
            for i in range(len(task_set)):
                Locations[i] = self_taskLocations[task_set[i]]

            if R_current != startRegion:
                Locations.insert(0, [-1, -1])
                
            taskCount = len(Locations) - 1
            utilities = []
            for i in range(len(task_set)):
                if task_set[i] > 0:
                    utilities.append(self_utilities[task_set[i] - 1])

            B = Budgets[R_current] + savings
            if B < 0:
                B = 0
            start = time.time()
            p  = find_opt_path(0,taskCount,utilities,B,Locations, 40, H)
            e = time.time() - start
            region_times.append(e)
            next_region = R_current
            # If the starting cluster has no feasible solution, 
            # Add the budget of another cluster until a solution is feasible
            if p[0] == [] or p[1] == 0.0:
                j = r + 1
                while p[1] == 0.0 and j < len(region_order):

                    B += Budgets[region_order[j]]
                    if Budgets[region_order[j]] > 0:
                        Budgets[region_order[j]] = 0
                        p  = find_opt_path(0,taskCount,utilities, B, Locations, 40, H)
                    j += 1
            
            path = []
            z = 0
            if R_current == startRegion:
                z = 0
            else:
                z = 1
            path_time = p[1]
            region_costs[R_current] = path_time
            savings = B - path_time
            for i in range(z,len(p[0])):
                path.append(task_set[p[0][i] - z])
            if len(path) == 1:
                path = []
            region_solutions[R_current] = path

            r += 1

        print "solved all clusters"
        if startRegion != 0:
            T = region_costs[startRegion]
            region_costs[startRegion] = region_costs[0]
            region_costs[0] = T

            T = region_solutions[startRegion][:]
            region_solutions[startRegion] = region_solutions[0][:]
            region_solutions[0] = T
        
	print "solving CPP to connect clusters..."
        t1 = time.time()
        optimal_path = get_TSP_matrix_H2(region_solutions, region_costs, Tasks, 2)
        time_H2 = time.time() - t1
        cost_M = [[0 for i in range(len(optimal_path))] 
                                            for j in range(len(optimal_path))]
        for i in range(len(optimal_path)):
            for j in range(len(optimal_path)):
                pi = Tasks[optimal_path[i]]
                pj = Tasks[optimal_path[j]]
                cost_M[i][j] = dist2(pi, pj)
	print "solving 2-opt search to optimize path"
        R = solve_2opt(cost_M, optimal_path)
	print "solved 2-opt"
        optimal_path = R[0]
        total_time = R[1]
        optimal_path = R[0] 
        total_time = R[1]
        t = 0
        for i in range(len(optimal_path) - 1):
            pi = Tasks[optimal_path[i]]
            pj = Tasks[optimal_path[i+1]]
            t += dist2(pi, pj)
        assert(abs(t - total_time) < 0.1)
        
        if total_time > self_timeBudget:
	    print "path cost exceeds budget, fixing..."
            optimal_path, total_time = repair(Tasks, optimal_path, 
                                            self_timeBudget, self_utilities)
            print "fixed path"

        U = 0
        for p in optimal_path:
            if p > 0:
                U += self_utilities[p-1]

        return optimal_path, U

    else:
        # define an array corresponding to x that define which task pair
        # each of the random variables refer to. in the form [[i,j],....]
        if self_taskCount <= 0:
            return []    
        sp = 0
        op, U = find_opt_path(sp,self_taskCount,self_utilities,self_timeBudget,
                                        self_taskLocations, 7200, H)
        U = 0
        for p in op:
            if p > 0:
                U += self_utilities[p-1]
        return op, U
        opt_path = op[0]
        optimal_path = []
        total_U = 0
        for i in range(len(opt_path)):
            total_U += self_utilities[opt_path[i] - 1]
            new_task = [self_taskLocations[opt_path[i]][0],
                            self_taskLocations[opt_path[i]][1]]
            index = taskLocations.index(new_task)
            optimal_path.append(new_task)

        if optimal_path[0] == self_taskLocations[0]:
            execT = [0 for i in range(len(optimal_path))]
            U = [0 for i in range(len(optimal_path))]
        else:
            execT = [0 for i in range(len(optimal_path) - 1)]
            U = [0 for i in range(len(optimal_path) - 1)]
        for i in range(len(execT)):
            task = optimal_path[i+1]
            index = self_taskLocations.index(task)
            if index != 0:
                execT[i] = self_execTimes[self_taskLocations.index(task) - 
                                                                            1]
                U[i] = self_utilities[self_taskLocations.index(task) -
                                                                            1]
        optimal_path.insert(0, self_taskLocations[0])

        return optimal_path, total_U

        if 2*i + 1 >= array_length:
            return None
        return 2*i + 1

def dist2(p1,p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def read_dataset(benchmark):
    f = open('benchmarks/'+benchmark+'.txt')
    f.readline()
    budget = float(f.readline().split()[1])
    line = f.readline()
    taskLocations = []
    while line != "":
        L = line.split()
        x = L[1]
        y = L[2]
        taskLocations.append([float(x), float(y)])
        line = f.readline()

    return taskLocations, budget

def dist(p1, p2):
    xd = p1[0] - p2[0]
    yd = p1[1] - p2[1]
    return np.sqrt(xd**2 + yd**2)

if __name__ == '__main__':
    
    #os.nice(-20) # give highest priority to process
    # Get user-inputs
    args = sys.argv
    benchmark = args[1]
    Gen = int(args[2])
    task_limit = int(args[3])
    H = args[4]
    taskLocations, Budget = read_dataset(benchmark)

    if Gen == 1:
        utilities = [1 for i in range(len(taskLocations) - 1)]
    else:
        utilities = [1 + (7141 * (i) + 73) % 100 for i in 
                                                range(1,len(taskLocations))]
    start_time = time.time()
    path, total_U = get_opt_path(taskLocations, utilities, Budget, task_limit, H)
    end_time = time.time() - start_time
    
    # add utility of the first task (starting location)
    if Gen == 1:
        total_U += 1
    else:
        total_U += 74
    
    print "found solution in",round(end_time,1),"seconds"
    print "solution has utility",total_U
    print "\n\nsolution:"
    print path
