import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib import colors as mcolors
import numpy as np

#####################################################################
# which data

TRACK_NAME = 'exp1'
SAVE_RESULTS = True
no_of_trajs = 0
line_width = 1
till = 450
#####################################################################
# plot best trajectory
filepath = 'laptraj-{}.png'.format(TRACK_NAME)

coordinates_center = np.loadtxt('coordinates_c.txt', delimiter=',')[:,:-2]
coordinates_left = np.loadtxt('coordinates_l.txt', delimiter=',')[:,:-1]
coordinates_right = np.loadtxt('coordinates_r.txt', delimiter=',')[:,:-1]

fig = plt.figure()
ax = plt.gca()
ax.axis('equal')
plt.plot(coordinates_center[0], coordinates_center[1], 'k', lw=0.5, alpha=0.5)
plt.plot(coordinates_left[0], coordinates_left[1], 'k', lw=0.5, alpha=0.5)
plt.plot(coordinates_right[0], coordinates_right[1], 'k', lw=0.5, alpha=0.5)
colors = [mcolors.to_rgba(c)
          for c in plt.rcParams['axes.prop_cycle'].by_key()['color']]
print(colors)
# best trajectory
#plt.plot(wx_nei[:-1], wy_nei[:-1], linestyle='', marker='D', ms=5)
labels = [f'line {i}' for i in range(len(colors))]
for i in range(no_of_trajs):
	trajectory = np.loadtxt("coordinates_nc{}.txt".format(i), delimiter=',')
	x = np.array(trajectory[0][:till])
	y = np.array(trajectory[1][:till])
	speed = np.array(trajectory[2])
	points = np.array([x, y]).T.reshape(-1, 1, 2)
	segments = np.concatenate([points[:-1], points[1:]], axis=1)
	norm = plt.Normalize(speed.min(), speed.max())
	lc = LineCollection(segments, linestyle = 'solid', color = colors[i], label=labels[i])
	lc.set_linewidth(line_width)
	line = ax.add_collection(lc)
	#fig.colorbar(line, ax=ax)
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.legend()

if SAVE_RESULTS:
	plt.savefig(filepath, dpi=600, bbox_inches='tight')

#####################################################################

plt.show()
