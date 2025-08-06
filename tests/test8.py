import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Create a figure and polar axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='polar')

# Set up initial plot
r = np.arange(0, 2, 0.01)
theta = 2 * np.pi * r
line, = ax.plot(theta, r)

# Animation update function
def update(frame):
    theta = 2 * np.pi * r + frame / 10
    line.set_data(theta, r)
    return line,

# Create animation
ani = FuncAnimation(fig, update, frames=200, interval=20, blit=True)

# Show animation
plt.show()