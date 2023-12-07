import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import atexit

# Assume vx and vy are initially zero and continuously updated
vx = 0.0
vy = 0.0

# Create a figure and axes
fig, (ax_vx) = plt.subplots(1, 1)
ax_vx.set_xlim(0, 100)  # Adjust x-axis limits based on your data


# Initialize velocity arrays
vx_values = np.array([0.0])


# Initialize time array
time = np.array([0])

# Update function called for each animation frame
def update(frame):
    global vx
    global vx_values, time

    # Update velocity components (replace with your actual updating logic)
    vx += 0.01 * np.random.randn()

    # Update time
    time = np.append(time, frame)

    # Update velocity arrays
    vx_values = np.append(vx_values, vx)

    # Update plots
    ax_vx.clear()
    ax_vx.plot(time, vx_values, label='vx')

    # Add labels and legend
    ax_vx.set_xlabel('Time')
    ax_vx.set_ylabel('vx')
    ax_vx.legend()

# Register function to save plot on program exit
@atexit.register
def save_plot_on_exit():
    plt.savefig('/home/mawais/catkin_ws/src/sphero_simulation/sphero_stage/resources/figures/velocity_plot.png')

# Create animation
ani = FuncAnimation(fig, update, frames=None, interval=100, save_count=1000)

# Show the plot
plt.tight_layout()
plt.show()

# Save the plot before exiting
save_plot_on_exit()
