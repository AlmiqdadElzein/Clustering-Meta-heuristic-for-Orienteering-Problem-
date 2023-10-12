import os
from random import randint
from itertools import permutations

def get_max_weight(M):
    max_weight = 0
    for i in range(len(M)):
        for j in range(len(M)):
            if M[i][j] > max_weight:
                max_weight = M[i][j]
    return max_weight

def get_cost(solution, cost_M):
    c = 0
    for i in range(len(solution) - 1):
        c += cost_M[solution[i]][solution[i+1]]
    return c

def solve_2opt(cost_M, starting_path):

    assert(len(cost_M) >= 10)
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
                if i != j:
                    f.write(str(int(cost_M[i][j]))+' ')
                else:
                    f.write(str(99999)+' ')
            else:
                if i != j:
                    f.write(str(int(cost_M[i][j]))+'\n')
                else:
                    f.write(str(99999))

    f.close()
    os.system('./concorde/LINKERN/linkern -o solution TSP_problem > output')
    f = open('solution', 'r')
    L = f.readline().split()
    L = f.readline().split()
    path = [0]
    while L[1] != '0':
        path.append(int(L[1]))
        L = f.readline().split()
    f.close()
    os.system('rm TSP_problem')
    os.system('rm output')
    os.system('rm solution')
    actual_path = [0 for i in range(len(path))]
    for i in range(len(path)):
        actual_path[i] = starting_path[path[i]]

    t = 0
    for i in range(len(path) - 1):
        t += cost_M[path[i]][path[i+1]]

    return [actual_path, t]
