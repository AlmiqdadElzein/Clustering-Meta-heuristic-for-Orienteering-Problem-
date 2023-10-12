import os
from random import randint
from math import ceil
import subprocess

def is_symmetric(M):
    for i in range(len(M)):
        for j in range(len(M)):
            if M[i][j] != M[j][i]:
                return False
    return True


def dist(pi, pj):
    xd = pi[0] - pj[0]
    yd = pi[1] - pj[1]
    return (xd**2 + yd**2) ** 0.5

def is_triangular_indequality_satisfied(M, taskLocations):
    result = True
    for i in range(1,len(M)):
        for j in range(1,len(M)):
            for k in range(1,len(M)):
                if i != j and j != k and i != k:
                    c_ij = M[i][j]
                    c_jk = M[j][k]
                    c_ik =  M[i][k]
                    if c_ik - c_ij - c_jk > 0.01:
                        return False

    return True

def zeros(x):
    n = 0
    k = x
    while k > 1:
        k /= 10
        n += 1
    return n


def write_OP_file(cost_M, filename, budget, utilities):
    for i in range(len(cost_M)):
        cost_M[i][i] = 999999
    f = open(filename, 'w')
    city_count = len(cost_M)
    f.write('NAME : OP_problem\n')
    f.write('COMMENT : solving an OP using evolutionary algorithm\n')
    f.write('TYPE : OP\n')
    f.write('DIMENSION: '+str(city_count)+'\n')
    f.write('COST_LIMIT : '+str(int(budget * 100))+'\n')
    f.write('EDGE_WEIGHT_TYPE : EXPLICIT\n')
    f.write('EDGE_WEIGHT_FORMAT : FULL_MATRIX\n')
    f.write('EDGE_WEIGHT_SECTION\n')
    for i in range(len(cost_M)):
        for j in range(len(cost_M)):
            if j != len(cost_M) - 1:
                f.write(str(int(cost_M[i][j] * 100))+' ')
            else:
                f.write(str(int(cost_M[i][j] * 100))+'\n')

    f.write('NODE_SCORE_SECTION\n')
    for i in range(city_count):
        f.write(str(i+1)+' '+str(utilities[i])+'\n')
    f.write('DEPOT_SECTION\n')
    f.write('1\n')
    f.write('-1\n')
    f.write('EOF')

    f.close()
    
def write_OP_file2(cost_M, filename, budget, utilities):
    INT_MAX = 2**25 - 1
    for i in range(len(cost_M)):
        cost_M[i][i] = INT_MAX

    f = open(filename, 'w')
    city_count = len(cost_M)
    f.write('NAME : OP_problem\n')
    f.write('COMMENT : solving an OP using evolutionary algorithm\n')
    f.write('TYPE : OP\n')
    f.write('DIMENSION: '+str(city_count)+'\n')
    assert(int(budget * 100)  < INT_MAX)
    f.write('COST_LIMIT : '+str(int(budget))+'\n')
    f.write('EDGE_WEIGHT_TYPE : EXPLICIT\n')
    f.write('EDGE_WEIGHT_FORMAT : FULL_MATRIX\n')
    f.write('EDGE_WEIGHT_SECTION\n')
    for i in range(len(cost_M)):
        for j in range(len(cost_M)):
            if j != len(cost_M) - 1:
                if cost_M[i][j] < INT_MAX:
                    f.write(str(int(cost_M[i][j]))+' ')
                else:
                    f.write(str(int(cost_M[i][j]))+' ')
            else:
                if cost_M[i][j] < INT_MAX:
                    f.write(str(int(cost_M[i][j]))+'\n')
                else:
                     f.write(str(int(cost_M[i][j]))+'\n')

    f.write('NODE_SCORE_SECTION\n')
    for i in range(city_count):
        f.write(str(i+1)+' '+str(utilities[i])+'\n')
    f.write('DEPOT_SECTION\n')
    f.write('1\n')
    f.write('-1\n')
    f.write('EOF')

    f.close()

def write_OP_file3(cost_M, filename, budget, utilities, taskLocations):
    INT_MAX = 2**30 - 1
    for i in range(len(cost_M)):
        cost_M[i][i] = INT_MAX

    f = open(filename, 'w')
    city_count = len(cost_M)
    f.write('NAME : OP_problem\n')
    f.write('COMMENT : solving an OP using evolutionary algorithm\n')
    f.write('TYPE : OP\n')
    f.write('DIMENSION: '+str(city_count)+'\n')
    f.write('COST_LIMIT : '+str(int(round(budget)))+'\n')
    f.write('EDGE_WEIGHT_TYPE : EUC_2D\n')
    f.write('NODE_COORD_SECTION\n')
    for i in range(len(taskLocations)):
        x = int(round(taskLocations[i][0]))
        y = int(round(taskLocations[i][1]))
        f.write(str(i+1)+' '+str(x)+' '+str(y)+'\n')
    
    f.write('NODE_SCORE_SECTION\n')

    for i in range(len(taskLocations)):
        f.write(str(i+1)+' '+str(utilities[i])+'\n')

    f.write('DEPOT_SECTION\n')
    f.write('1\n')
    f.write('-1\n')
    f.write('EOF')

    f.close()


def read_matrix_from_file(filename, taskLocations):
    f = open(filename, 'r')
    line = f.readline()
    while "EDGE_WEIGHT_SECTION" not in line:
        line = f.readline()

    matrix_lines = []
    line = f.readline()
    while "NODE_SCORE_SECTION" not in line:
        matrix_lines.append(line[:-1])
        line = f.readline()
    
    matrix = [matrix_lines[i].split() for i in range(len(matrix_lines))]

    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            matrix[i][j] = int(matrix[i][j])

    f.close()

    while not is_triangular_indequality_satisfied(matrix, taskLocations):
        for i in range(len(matrix)):
            for j in range(len(matrix)):
                for k in range(len(matrix)):
                    c_ij = matrix[i][j]
                    c_jk = matrix[j][k]
                    c_ik = matrix[i][k]
                    if (c_ij + c_jk < c_ik) and (i != j) and (i != k) and (j != k):
                        diff = c_ik - c_ij - c_jk
                        matrix[i][j] = int(matrix[i][j] + ceil(diff / 2.0))
                        matrix[j][k] = int(matrix[j][k] + ceil(diff / 2.0))
                        assert(matrix[i][j] + matrix[j][k] >= matrix[i][k])

    return matrix


def command_timeout(command, timeout):
    os.system('./exec_timeout.py '+str(timeout)+' '+command)

def solve_EA_OP(cost_M, utilities, budget, taskLocations):

    assert(is_symmetric(cost_M))
    assert(is_triangular_indequality_satisfied(cost_M, taskLocations))
    assert(len(cost_M) == len(utilities))
    filename = "test.oplib"
    write_OP_file(cost_M, filename, budget, utilities)
    #os.system('./compass/compass --op --op-ea4op test.oplib -o out > output_here')
    command_timeout('./compass/compass --op --op-ea4op test.oplib -o out > output_here', 1)
    try:
        f = open('out', 'r')
    except:
        os.system('rm out')
        return [], 0
    line = f.readline()
    os.system("rm output_here")
    while line[:10] != 'ROUTE_COST':
        line = f.readline()
    total_time = float(line[13:-1])
    f.readline()
    line = f.readline()
    path = []
    while line != '-1\n':
        path.append(int(line[:-1]) - 1)
        line = f.readline()
    f.close()
    os.system('rm out')
    os.system('rm test.oplib')
    t = 0
    for i in range(len(path) - 1):
        t += cost_M[path[i]][path[i+1]]
    return path, t

