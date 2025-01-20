import numpy as np
import matplotlib.pyplot as plt

# Number of robots
num_robots = 20

# Time step
dt = 0.001

# Circle formation parameters
radius = 3.0  # Radius of the circular formation
center = np.array([5, 5])  # Center of the circle

# Create random initial positions for the robots
positions = np.random.rand(num_robots, 2) * 10  # Random positions in a 10x10 space

# Generate target positions in a circular pattern
angles = np.linspace(0, 2 * np.pi, num_robots, endpoint=False)
target_positions = np.array([center + radius * np.array([np.cos(angle), np.sin(angle)]) for angle in angles])

# Initialize velocities
velocities = np.zeros_like(positions)

# Define adjacency matrix
adjacency_matrix = np.ones((num_robots, num_robots)) - np.eye(num_robots)

# Control loop
def control_law(positions, target_positions):
    velocities = np.zeros_like(positions)
    num_targets = len(target_positions)
    
    for i in range(len(positions)):
        for j in range(len(positions)):
            if adjacency_matrix[i, j] == 1:
                target_i = i % num_targets  # Wrap the target index using modulo
                target_j = j % num_targets  # Wrap the target index using modulo
                velocities[i] += (positions[j] - positions[i]) - (target_positions[target_i] - target_positions[target_j])
    
    return velocities

# Simulation loop
# keyframe_steps = [0, 50, 100, 150, 200, 250, 300, 350, 400, 450]  # Keyframes
# keyframe_count = 0  # Counter to track keyframes

for step in range(500):  # Run for 500 steps
    velocities = control_law(positions, target_positions)
    positions += velocities * dt  # Update positions based on velocities

    # Clear the plot
    plt.clf()
    
    # Plot the robots' positions and desired positions
    plt.scatter(positions[:, 0], positions[:, 1], c='blue', label='Robots')
    plt.scatter(target_positions[:, 0], target_positions[:, 1], c='green', marker='x', label='Desired Positions (Circle)')
    
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.legend()
    plt.pause(0.01)  # Pause to create animation effect

    # Save keyframes at specified steps
    # if step in keyframe_steps:
    #     plt.savefig(f'keyframe_{keyframe_count}.png')  # Save the figure as a PNG file
    #     keyframe_count += 1

plt.show()
