import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define the parameters
n_robots = 10  # Number of robots
dim = 2  # 2D space

# Initialize random positions for each robot
np.random.seed(0)
initial_positions = np.random.rand(n_robots, dim) * 10  # Random positions in 2D
consensus_point = initial_positions.mean(axis=0)  # Target consensus point

# Define adjacency matrix for network topology (nearest neighbors)
adjacency_matrix = np.zeros((n_robots, n_robots))
for i in range(n_robots - 1):
    adjacency_matrix[i, i + 1] = 1
    adjacency_matrix[i + 1, i] = 1  # Undirected graph

# Consensus dynamics function for the ODE solver
def consensus_dynamics(t, positions):
    positions = positions.reshape((n_robots, dim))
    velocity = np.zeros((n_robots, dim))
    
    for i in range(n_robots):
        for j in range(n_robots):
            if adjacency_matrix[i, j] == 1:
                velocity[i] += (positions[j] - positions[i])
    
    return velocity.flatten()

# Run the ODE solver to simulate the system over time
time_span = (0, 10)  # Simulate from t=0 to t=10
initial_conditions = initial_positions.flatten()  # Flatten for the ODE solver

solution = solve_ivp(consensus_dynamics, time_span, initial_conditions, t_eval=np.linspace(0, 10, 200))

# Setting up the animation
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_title("2D Consensus of Robots Over Time (Increasing Time Speed)")
ax.set_xlabel("X position")
ax.set_ylabel("Y position")

# Initial plot setup
robots, = ax.plot([], [], 'bo', label="Robots")
trajectory_lines = [ax.plot([], [], 'b-', alpha=0.3)[0] for _ in range(n_robots)]
consensus_marker, = ax.plot(consensus_point[0], consensus_point[1], 'rx', markersize=10, label="Consensus Point")

# Initialize the plot elements
def init():
    robots.set_data([], [])
    for line in trajectory_lines:
        line.set_data([], [])
    return [robots, consensus_marker] + trajectory_lines

# Update function with dynamic time acceleration
def update(frame):
    # Dynamically increase the index based on the frame count to speed up over time
    accelerated_frame = int(frame ** 1.5)  # Using a power function for non-linear acceleration
    
    # Ensure we don't exceed the solution's time length
    if accelerated_frame >= len(solution.t):
        accelerated_frame = len(solution.t) - 1

    # Extract current positions of robots
    current_positions = solution.y[:, accelerated_frame].reshape((n_robots, dim))
    robots.set_data(current_positions[:, 0], current_positions[:, 1])
    
    # Update each robot's trajectory up to the current accelerated frame
    for i, line in enumerate(trajectory_lines):
        line.set_data(solution.y[i * dim, :accelerated_frame], solution.y[i * dim + 1, :accelerated_frame])
    
    return [robots, consensus_marker] + trajectory_lines

# Creating the animation
ani = FuncAnimation(fig, update, frames=range(100), init_func=init, blit=True, interval=50)

# Show the animation
plt.legend()
plt.show()
