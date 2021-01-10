import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
import numpy as np

path = np.loadtxt('recorded_path.txt', delimiter=',')
acc_x = path[:,2]
normal_force = path[:,5] + path[:,6] + path[:,7] + path[:,8]
friction_x = path[:,9] + path[:,10] + path[:,11] + path[:,12]
gears = path[:,-1]
speeds = path[:,4]
#plt.plot(normal_force)
#plt.plot((friction_x))
plt.plot(speeds)
#plt.plot(0.43*speeds*speeds)
#plt.plot(750*acc_x)
plt.plot(10*gears)

plt.show()
