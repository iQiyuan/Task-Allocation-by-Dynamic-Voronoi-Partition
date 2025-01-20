import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# Number of robots
num_robots = 20

# Time step
dt = 0.01

# Circle formation parameters
radius = 3.0  # Radius of the circular formation
center = np.array([5, 5])  # Center of the circle

# Create random initial positions for the robots
positions = np.random.rand(num_robots, 2) * 10  # Random positions in a 10x10 space

# Generate target distances between each pair of robots in a circular pattern
angles = np.linspace(0, 2 * np.pi, num_robots, endpoint=False)
target_positions = np.array([center + radius * np.array([np.cos(angle), np.sin(angle)]) for angle in angles])
target_distances = np.linalg.norm(target_positions[:, np.newaxis, :] - target_positions[np.newaxis, :, :], axis=-1)

# Create an adjacency matrix based on connectivity percentage
def create_adjacency_matrix(connectivity):
    adjacency_matrix = np.zeros((num_robots, num_robots))
    for i in range(num_robots):
        neighbors = np.random.choice(num_robots, connectivity, replace=False)
        for j in neighbors:
            if i != j:
                adjacency_matrix[i, j] = 1
                adjacency_matrix[j, i] = 1  # Ensure undirected connectivity
    return adjacency_matrix

# Control loop for distance-based control
def control_law_distance_based(positions, target_distances, adjacency_matrix):
    velocities = np.zeros_like(positions)
    for i in range(len(positions)):
        for j in range(len(positions)):
            if adjacency_matrix[i, j] == 1:
                current_distance = np.linalg.norm(positions[j] - positions[i])
                if current_distance != 0:
                    # Direction vector from i to j
                    direction = (positions[j] - positions[i]) / current_distance
                    # Distance error
                    distance_error = current_distance - target_distances[i, j]
                    # Apply the control law
                    velocities[i] += distance_error * direction
    return velocities

# Initialize the plot and slider
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.3)  # Adjust plot to make room for the slider and button

# Plot initial positions
robots_scatter = plt.scatter(positions[:, 0], positions[:, 1], c='blue', label='Robots')
target_scatter = plt.scatter(target_positions[:, 0], target_positions[:, 1], c='green', marker='x', label='Desired Positions (Circle)')
plt.xlim(0, 10)
plt.ylim(0, 10)
plt.legend()

# Add a slider for connectivity
ax_connectivity = plt.axes([0.2, 0.15, 0.65, 0.03], facecolor='lightgoldenrodyellow')
connectivity_slider = Slider(ax_connectivity, 'Connectivity', 0, num_robots - 1, valinit=num_robots - 1, valstep=1)

# Add a button to start the simulation
ax_button = plt.axes([0.45, 0.05, 0.1, 0.04])
start_button = Button(ax_button, 'Start Simulation')

# Define a function to start the simulation with animation
def start_simulation(event):
    connectivity = int(connectivity_slider.val)
    adjacency_matrix = create_adjacency_matrix(connectivity)
    
    # Run the simulation with animation
    global positions
    for step in range(5000):  # Run for 500 steps
        velocities = control_law_distance_based(positions, target_distances, adjacency_matrix)
        positions += velocities * dt
        
        # Update the scatter plot data
        robots_scatter.set_offsets(positions)
        
        # Redraw the plot to show animation
        plt.pause(0.01)  # Pause to create animation effect

# Attach the start function to the button
start_button.on_clicked(start_simulation)

plt.show()
