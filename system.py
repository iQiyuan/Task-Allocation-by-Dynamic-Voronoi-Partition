import numpy as np
from scipy.integrate import odeint


# Consensus function xdot = -Lx
def consensus(x, t, L):
    return -np.dot(L, x)

# Function to dynamically create the adjacency matrix based on user input
def create_custom_adjacency_matrix(num_robots, connection_matrix):
    L = np.zeros((num_robots, num_robots))
    for i in range(num_robots):
        for j in range(num_robots):
            if connection_matrix[i][j] == 1:    # Unidirectional from i to j
                L[i][j] = -1
                L[j][j] += 1
            elif connection_matrix[i][j] == 2:  # Bidirectional
                L[i][j] = -1
                L[j][i] = -1
                L[i][i] += 1
                L[j][j] += 1
    return L

# Function to simulate consensus dynamics
def simulate_consensus(num_robots, connection_matrix):
    # Initial conditions
    x0 = np.random.rand(num_robots)
    
    # Time points
    t = np.linspace(0, 10, 500)

    # Get Laplacian matrix based on custom connections
    L = create_custom_adjacency_matrix(num_robots, connection_matrix)

    # Solve ODE
    solution = odeint(consensus, x0, t, args=(L,))
    return t, solution