import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
import numpy as np

path = np.loadtxt('recorded_path.txt', delimiter=',')
<<<<<<< HEAD
acc_x = path[:,2]
=======

>>>>>>> 422d40fec66a3b9fa2bc413a9bbb13d5e21d7b15
normal_force = path[:,5] + path[:,6] + path[:,7] + path[:,8]
friction_x = path[:,9] + path[:,10] + path[:,11] + path[:,12]
gears = path[:,-1]
speeds = path[:,4]
<<<<<<< HEAD
#plt.plot(normal_force)
#plt.plot((friction_x))
plt.plot(speeds)
#plt.plot(0.43*speeds*speeds)
#plt.plot(750*acc_x)
plt.plot(10*gears)
=======
>>>>>>> 422d40fec66a3b9fa2bc413a9bbb13d5e21d7b15

plt.plot(friction_x)
plt.show()
