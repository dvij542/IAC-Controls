import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
import numpy as np

path = np.loadtxt('recorded_path.txt', delimiter=',')

normal_force = path[:,5] + path[:,6] + path[:,7] + path[:,8]
friction_x = path[:,9] + path[:,10] + path[:,11] + path[:,12]
gears = path[:,-1]
speeds = path[:,4]

plt.plot(friction_x)
plt.show()
