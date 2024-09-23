import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
from system import *

# Update connection matrix based on dropdown selections
def on_connection_change(event, i, j, conn_type, direction, connection_matrix):
    if conn_type.get() == "Unidirectional":
        direction.grid()  # Show direction dropdown
        if direction.get() == f"{i+1} → {j+1}":
            connection_matrix[i][j] = 1
        elif direction.get() == f"{j+1} → {i+1}":
            connection_matrix[j][i] = 1
    elif conn_type.get() == "Bidirectional":
        connection_matrix[i][j] = 2
        connection_matrix[j][i] = 2
        direction.grid_remove()  # Hide direction dropdown
    else:
        connection_matrix[i][j] = 0
        connection_matrix[j][i] = 0
        direction.grid_remove()  # Hide direction dropdown

# Update robot controls dynamically in the GUI
def update_robot_controls(robot_slider, connection_frame, connection_matrix):
    # Clear previous controls if they exist
    for widget in connection_frame.winfo_children():
        widget.destroy()

    num_robots = int(robot_slider.get())
    connection_matrix[:] = [[0 for _ in range(num_robots)] for _ in range(num_robots)]

    # Dynamically create connection controls for each pair of robots
    for i in range(num_robots):
        for j in range(i + 1, num_robots):
            tk.Label(connection_frame, text=f"Robot {i+1} ↔ Robot {j+1}:").grid(row=i*num_robots + j, column=0)

            # Dropdown to select connection type
            connection_type = ttk.Combobox(connection_frame, values=["No Connection", "Unidirectional", "Bidirectional"])
            connection_type.set("No Connection")
            connection_type.grid(row=i*num_robots + j, column=1)

            # Direction dropdown (appears if Unidirectional is selected)
            direction = ttk.Combobox(connection_frame, values=[f"{i+1} → {j+1}", f"{j+1} → {i+1}"])
            direction.grid(row=i*num_robots + j, column=2)
            direction.grid_remove()  # Initially hidden

            # Bind connection type changes to update connection matrix
            connection_type.bind("<<ComboboxSelected>>", lambda event, i=i, j=j, conn_type=connection_type, direction=direction:
                on_connection_change(event, i, j, conn_type, direction, connection_matrix))
            direction.bind("<<ComboboxSelected>>", lambda event, i=i, j=j, conn_type=connection_type, direction=direction:
                on_connection_change(event, i, j, conn_type, direction, connection_matrix))

# Update the plot based on the current configuration
def update_plot(robot_slider, connection_matrix, ax, canvas):
    num_robots = int(robot_slider.get())
    t, solution = simulate_consensus(num_robots, connection_matrix)
    
    ax.clear()
    for i in range(num_robots):
        ax.plot(t, solution[:, i], label=f'Robot {i+1}')
    ax.legend(loc='best')
    ax.set_title(f'Simulation with {num_robots} Robots')
    ax.set_xlabel('Time')
    ax.set_ylabel('State')
    canvas.draw()

# Create the GUI window
def create_gui():
    window = tk.Tk()
    window.title("Consensus Simulation")

    connection_matrix = []

    # Robot number slider
    robot_slider = tk.Scale(window, from_=2, to=10, label="Number of Robots", orient=tk.HORIZONTAL)
    robot_slider.pack()

    # Frame for connection controls
    connection_frame = tk.Frame(window)
    connection_frame.pack()

    # Button to update plot
    fig, ax = plt.subplots()
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.get_tk_widget().pack()

    # Update robot controls when slider changes
    robot_slider.config(command=lambda val: update_robot_controls(robot_slider, connection_frame, connection_matrix))

    # Button to update plot
    update_button = tk.Button(window, text="Simulate", command=lambda: update_plot(robot_slider, connection_matrix, ax, canvas))
    update_button.pack()

    window.mainloop()

# Run the GUI
create_gui()
