import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

#####################################################################
# which data

TRACK_NAME = 'exp1'
#####################################################################
# plot best trajectory
filepath = 'results/laptraj-{}.png'.format(TRACK_NAME)

coordinates_center = np.readtxt('coordinates_center.txt', delimiter=',')
coordinates_left = np.readtxt('coordinates_left.txt', delimiter=',')
coordinates_right = np.readtxt('coordinates_right.txt', delimiter=',')

fig = plt.figure()
ax = plt.gca()
ax.axis('equal')
plt.plot(coordinates_center[0], coordinates_center[1], '--k', lw=0.5, alpha=0.5)
plt.plot(coordinates_left[0], coordinates_left[1], 'k', lw=0.5, alpha=0.5)
plt.plot(coordinates_right[0], coordinates_right[1], 'k', lw=0.5, alpha=0.5)

# best trajectory
#plt.plot(wx_nei[:-1], wy_nei[:-1], linestyle='', marker='D', ms=5)
trajectory = np.readtxt("traj.txt", delimiter=',')
x = np.array(trajectory[0])
y = np.array(trajectory[1])
speed = np.array(trajectory[2])
points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = plt.Normalize(speed.min(), speed.max())
lc = LineCollection(segments, cmap='viridis', norm=norm)
lc.set_array(speed)
lc.set_linewidth(2)
line = ax.add_collection(lc)
fig.colorbar(line, ax=ax)
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')

if SAVE_RESULTS:
	plt.savefig(filepath, dpi=600, bbox_inches='tight')

#####################################################################

plt.show()
