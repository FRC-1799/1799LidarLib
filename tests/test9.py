import matplotlib.pyplot as plt
import numpy as np

# Generating sample data
theta = np.linspace(0, 2*np.pi, 100)
r = np.random.rand(100)  # Random radius values
colors = np.random.rand(100)  # Random colors

# Creating the polar scatter plot
plt.figure(figsize=(8, 8))
ax = plt.subplot(111, polar=True)
ax.scatter(theta, r, c=colors, s=100, cmap='hsv', alpha=0.75)

plt.title('Scatter Plot on Polar Axis', fontsize=15)
plt.show()