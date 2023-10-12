The Python 3.6 code in this folder implements the clustering metaheuristic for Orienteering Problems (OP) described in the paper:
"A clustering metaheuristic for large orienteering problems", A. Elzein, G. A. Di Caro, PLOS ONE, 2022.

The clustering meta-heuristic tackles an OP by grouping the nodes into clusters, solving an OP inside each cluster, and connect the solutions of the clusters.


Prerequisites:

Inside the 'Clustering-Meta-Heuristic-for-the-Orienteering-Problem' directory, install and configure 'concorde' (https://github.com/matthelb/concorde) and 'compass' (https://github.com/bcamath-ds/compass). 
Concorde is. well-know solver for TSP, while compass installs the EA4OP evolutionary heuristic described in the paper: 
An efficient evolutionary algorithm for the orienteering problem. Kobeaga G, Merino M, Lozano J., Computers & Operations Research. 2017:90.
In addition, if you want to use Gurobi instead of EA4OP as a base solver, you shall install Gurobi https://www.gurobi.com/



How to Use:

./solve_CMH.py Benchmark Gen N_max Heuristic


Benchmark: One of the files in the 'benchmarks' directory. This parameter should be used without the '.txt' extension (for example, EIL101).

Gen: if Gen = 1, the utilities of all nodes will be set to 1. If Gen = 2, the utilities of the nodes are assigned psudo-randomly.

N_max: The maximum number of nodes per cluster

Heuristic: If Heuristic = 'ea4op', the ea4op evolutionary algorithm is used to find a path for each cluster. If Heuristic = 'gur', the Gurobi exact solver is used to find a path for each cluster
