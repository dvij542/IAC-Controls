import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib import colors as mcolors
import numpy as np
# import pandas as pd
# X;Y;Yaw;speed

#####################################################################
# which data

TRACK_NAME = 'exp1'
SAVE_RESULTS = True
no_of_trajs = 2
line_width = 1
till = 292
#####################################################################
# plot best trajectory
filepath = 'laptraj-{}.png'.format(TRACK_NAME)

# coordinates_center = np.loadtxt('coordinates_c.txt', delimiter=',')
# coordinates_left = np.loadtxt('coordinates_l.txt', delimiter=',')
# coordinates_right = np.loadtxt('coordinates_r.txt', delimiter=',')

fig = plt.figure()
ax = plt.gca()
# ax.axis('equal')
# plt.plot(coordinates_center[0,:-1]*100, coordinates_center[1,:-1]*100, 'k', lw=0.5, alpha=0.5)
# plt.plot(coordinates_left[0,:-1]*100, coordinates_left[1,:-1]*100, 'k', lw=0.5, alpha=0.5)
# plt.plot(coordinates_right[0,:-1]*100, coordinates_right[1,:-1]*100, 'k', lw=0.5, alpha=0.5)
# coords = np.ones((4,coordinates_center.shape[1]-1))*6.5
# coords[:2,:] = coordinates_center[:,:-1]*100
# np.savetxt('track.csv',coords.T,delimiter=',')
colors = [mcolors.to_rgba(c)
		  for c in plt.rcParams['axes.prop_cycle'].by_key()['color']]
# print(colors)
# best trajectory
#plt.plot(wx_nei[:-1], wy_nei[:-1], linestyle='', marker='D', ms=5)
labels = [f'line {i}' for i in range(len(colors))]

glo_speed_min=None
glo_speed_max=None
speed_glo=[]
for i in range(no_of_trajs):
	trajectory = np.loadtxt("coordinates_nc{}.txt".format(i), delimiter=';')
	trajectory=trajectory.T
	speeds = np.array(trajectory[3])
	speed_glo+=list(speeds)
	if(glo_speed_max):
		glo_speed_max=max(glo_speed_max,max(speeds))
	else: 
		glo_speed_max=max(speeds)
	if(glo_speed_min):
		glo_speed_min=min(glo_speed_min,min(speeds))
	else: 
		glo_speed_min=min(speeds)
speed_glo=list(set(speed_glo))
speed_glo.sort()
speed_glo=np.array(speed_glo)
# print(speed_glo)	
cmap = plt.get_cmap('jet')
norm = plt.Normalize(glo_speed_min, glo_speed_max)
import matplotlib.cm as cm
mappable = cm.ScalarMappable(norm, cmap)
mappable.set_array(speeds)
# create the colorbar
cb = plt.colorbar(mappable)    
cb.set_label('meter/s')
for i in range(no_of_trajs):
	trajectory = np.loadtxt("coordinates_nc{}.txt".format(i), delimiter=';')
	trajectory=trajectory.T
	x = np.array(trajectory[0])
	y = np.array(trajectory[1])
	speeds = np.array(trajectory[3])
	print(x,y,speeds)

	# points = np.array([x, y]).T.reshape(-1, 1, 2)
	# segments = np.concatenate([points[:-1], points[1:]], axis=1)
	# norm = plt.Normalize(speed.min(), speed.max())
	# lc = LineCollection(segments, linestyle = 'solid', color = colors[i], label=labels[i])
	# lc.set_linewidth(line_width)
	# line = ax.add_collection(lc)
	# fig.colorbar(line, ax=ax)

	for j in range(1,till):
		speed = speeds[j-1]
		x0, y0 = x[j-1], y[j-1]
		x1, y1 = x[j], y[j]
		ax.plot([x0, x1], [y0, y1], '-', color=cmap(norm(speed)))
	# put points for each observation (no colouring)
	# ax.scatter(x, y)
	# create a mappable suitable for creation of a colorbar
	print(f'{i} done')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.legend()

if SAVE_RESULTS:
	plt.savefig(filepath, dpi=600, bbox_inches='tight')

#####################################################################

plt.show()
