import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
import numpy as np
from math import *

def dist(x1,y1,x2,y2):
    return ((x1-x2)**2 + (y1-y2)**2)**(1/2)

#####################################################################
# which data

TRACK_NAME = 'exp1'
SAVE_RESULTS = True
optimal_racing_line = np.loadtxt('coordinates_nc3.txt', delimiter=',')[:,:-5].T*100.0
center_line = np.loadtxt('coordinates_c.txt', delimiter=',').T*100.0
#####################################################################
# plot best trajectory
minval = 10000

minval = 10000
x_ref_m = []
y_ref_m = []
width_right_m = []
width_left_m = []
x_normvec_m = []
y_normvec_m = []
alpha_m = []
s_racetraj_m = []
psi_racetraj_rad = []
kappa_racetraj_radpm = []
vx_racetraj_mps = []
ax_racetraj_mps2 = []

s = 0
prevx = -1
prevy = -1
ko = optimal_racing_line.shape[0]
k = center_line.shape[0]
for j in range(optimal_racing_line.shape[0]):
	x = optimal_racing_line[j,0]
	y = optimal_racing_line[j,1]
	v = optimal_racing_line[j,2]
	minval = 10000
	if prevx!=-1 :
		s=s+dist(x,y,prevx,prevy)
	for i in range(k):
		if dist(center_line[i,0], center_line[i,1], optimal_racing_line[j,0], optimal_racing_line[j,1]) < minval:
			minval = dist(center_line[i,0], center_line[i,1], optimal_racing_line[j,0], optimal_racing_line[j,1])
			mini = i

	pointu = center_line[(mini+1)%k,:]
	pointd = center_line[(mini-1)%k,:]
	point = center_line[mini,:]

	theta = atan2(pointu[1]-pointd[1],pointu[0]-pointd[0])
	A = sin(theta)
	B = -cos(theta)
	C = -A*point[0]-B*point[1]
	d = A*x+B*y+C
	x_ref = x-d*A#-A*C+(x-A*(A*x+B*y))*(-B)
	y_ref = y-d*B#-B*C+(y-B*(A*x+B*y))*(A)
	x_ref_m.append(x_ref)
	y_ref_m.append(y_ref)
	width_right_m.append(9)
	width_left_m.append(9)
	x_normvec_m.append(A)
	y_normvec_m.append(B)
	alpha_m.append(d)
	s_racetraj_m.append(s)
	p_f = optimal_racing_line[(j+1)%ko,:]
	p_b = optimal_racing_line[(j-1)%ko,:]
	psi_rad = atan2(p_f[0]-p_b[0],p_f[1]-p_b[1])
	psi_racetraj_rad.append(psi_rad)
	if abs(p_f[1]-y) < abs(p_f[0]-x) :
		dydx2 = (p_f[1]-y)/(p_f[0]-x)
		dydx1 = (p_b[1]-y)/(p_b[0]-x)
		dydx = (dydx2+dydx1)/2
		d2ydx2 = 2*(dydx2-dydx1)/(p_f[0]-p_b[0])
		Rk = d2ydx2/((1+(dydx)**2)**(3/2))
	else :
		dxdy2 = (p_f[0]-x)/(p_f[1]-y)
		dxdy1 = (p_b[0]-x)/(p_b[1]-y)
		dxdy = (dxdy2+dxdy1)/2
		d2xdy2 = 2*(dxdy2-dxdy1)/(p_f[1]-p_b[1])
		Rk = d2xdy2/((1+(dxdy)**2)**(3/2))
	kappa_racetraj_radpm.append(Rk)
	vx_racetraj_mps.append(v)
	a = (optimal_racing_line[(j+1)%ko,2]-optimal_racing_line[(j-1)%ko,2])/0.8
	ax_racetraj_mps2.append(a)
	prevx = x 
	prevy = y

output = [x_ref_m, y_ref_m, width_right_m, width_left_m, x_normvec_m, y_normvec_m, alpha_m, s_racetraj_m, psi_racetraj_rad, kappa_racetraj_radpm, vx_racetraj_mps, ax_racetraj_mps2]
np.savetxt('traj_ltpl_cl_test1.csv', np.array(output).T, delimiter = ';')
