import os
from random import randint
from itertools import permutations
import time
import numpy as np

def get_cost(solution, cost_M):
    c = 0
    for i in range(len(solution) - 1):
        c += cost_M[solution[i]][solution[i+1]]
    return c

def get_max_weight(M):
    max_weight = 0
    for i in range(len(M)):
        for j in range(len(M)):
            if M[i][j] > max_weight:
                max_weight = M[i][j]
    return max_weight

def is_symmetric(M):
    for i in range(len(M)):
        for j in range(len(M)):
            if M[i][j] != M[j][i]:
                return False
    return True

def fix_region_sequence(path):
    for i in range(0,len(path) - 1,2):
            j = i + 1
            i_index = path.index(i)
            j_index = path.index(j)
            if (i_index != j_index + 1) and (j_index != i_index + 1):
                if i == 0 and j == 1:
                    del path[j_index]
                    path.insert(1,1)
                else:
                    del path[min(i_index, j_index)]
                    if min(i_index, j_index) == i_index:
                        path.insert(max(i_index, j_index)-1, i)
                    else:
                        path.insert(max(i_index, j_index)-1, j)

    return path

def get_dmin(M):
    dmin = 2**31 - 1
    for i in range(len(M)):
        for j in range(len(M)):
            if M[i][j] < dmin:
                dmin = M[i][j]
    return dmin

def get_dmax(M):
    dmax = 0
    for i in range(len(M)):
        for j in range(len(M)):
            if M[i][j] > dmax:
                dmax = M[i][j]
    return dmax

def make_symmetric(M):
    e = 0.01
    INT_MAX = 2**10
    dmin = get_dmin(M)
    dmax = get_dmax(M)
    D = [[0 for i in range(len(M))] for i in range(len(M))]
    for i in range(len(D)):
        for j in range(len(D)):
            if i == j:
                D[i][j] = 0
            elif 4*dmin - 3*dmax > 0:
                D[i][j] = M[i][j]
            else:
                D[i][j] = M[i][j] + 3*dmax - 4*dmin + e

    D_T = np.array(D[:])
    D_T = D_T.transpose()

    D_symm = [[0 for i in range(2*len(M))] for j in range(2*len(M))]

    n = len(M)

    for i in range(n):
        for j in range(n):
            D_symm[i][j] = INT_MAX
    for i in range(n):
        for j in range(n, 2*n):
            D_symm[i][j] = D_T[i][j-n]
    for i in range(n, 2*n):
        for j in range(n):
            D_symm[i][j] = D[i-n][j]
    for i in range(n, 2*n):
        for j in range(n, 2*n):
            D_symm[i][j] = INT_MAX
    
    assert(is_symmetric(D_symm))
    return D_symm

def read_TSP_solution(filename):
    f = open('TSP_problem.sol', 'r')
    f.readline()
    line = f.readline()
    path = []
    while line != '':
        L = line.split()
        for i in range(len(L)):
            path.append(int(L[i]))
        line = f.readline()

    return path


def solve_TSP(cost_M):
    n = len(cost_M)
    if not is_symmetric(cost_M):
        cost_M = make_symmetric(cost_M)
    
    max_weight = get_max_weight(cost_M)
    f = open('TSP_problem', 'w')
    f.write('NAME : TSP_problem\n')
    f.write('TYPE : TSP\n')
    f.write('COMMENT : A new TSP problem\n')
    f.write('DIMENSION : '+str(len(cost_M))+'\n')
    f.write('EDGE_WEIGHT_TYPE : EXPLICIT\n')
    f.write('EDGE_WEIGHT_FORMAT : FULL_MATRIX\n')
    f.write('EDGE_WEIGHT_SECTION:\n')
    for i in range(len(cost_M)):
        for j in range(len(cost_M)):
            if j != len(cost_M) - 1:
                f.write(str(int(cost_M[i][j]))+' ')
            else:
                f.write(str(int(cost_M[i][j]))+'\n')
    
    f.close()
    finished = False
    z = 0
    s = time.time()
    solutions = []
    start_time = time.time()
    last_sol_time = 0
    time_since_last_sol = 0
    os.system('./concorde/TSP/concorde TSP_problem > output')
    path = read_TSP_solution('TSP_problem.sol')
    os.system("rm TSP_problem.sol")
    os.system('rm TSP_problem.mas')
    i = 0
    while i < len(path):
        if path[i] >= n:
            del path[i]
            i -= 1
        i += 1

    return path
