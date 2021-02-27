import matplotlib.pyplot as plt
import numpy as np

T = 0.1
x_car_1 = 0
y_car_1 = 0
vx_car_1 = 10
vy_car_1 = 0
theta_car_1 = 0

x_car_2 = 0
y_car_2 = 0
vx_car_2 = 10
vy_car_2 = 0
theta_car_2 = 0

x_targ1 = 2
y_targ1 = -2
vx_targ1 = 8.33
vy_targ1 = 5

x_targ2 = 5
y_targ2 = 5
vx_targ2 = 10
vy_targ2 = 0

poses_car_1 = []
poses_car_2 = []
poses_targ1 = []
poses_targ2 = []

for i in range(10) :
	poses_car_1.append([x_car_1,y_car_1])
	poses_car_2.append([x_car_2,y_car_2])
	poses_targ1.append([x_targ1,y_targ1])
	poses_targ2.append([x_targ2,y_targ2])
	
	x_targ1 = x_targ1 + vx_targ1*T
	y_targ1 = y_targ1 + vy_targ1*T
	
	x_targ2 = x_targ2 + vx_targ2*T
	y_targ2 = y_targ2 + vy_targ2*T


	dx = x_targ1 - x_car_1
	dy = y_targ1 + 1 - y_car_1
	vtheta = (2 * 10 * (dy*np.cos(theta_car_1)-dx*np.sin(theta_car_1)))/(dx**2 + dy**2)
	x_car_1 = x_car_1 + 10*np.cos(theta_car_1+vtheta*T/2)*T
	y_car_1 = y_car_1 + 10*np.sin(theta_car_1+vtheta*T/2)*T
	theta_car_1 = theta_car_1 + vtheta*T

	dx = x_targ1 - x_car_2
	dy = y_targ1 - 1 - y_car_2
	vtheta = (2 * 10 * (dy*np.cos(theta_car_2)-dx*np.sin(theta_car_2)))/(dx**2 + dy**2)
	x_car_2 = x_car_2 + 10*np.cos(theta_car_2+vtheta*T/2)*T
	y_car_2 = y_car_2 + 10*np.sin(theta_car_2+vtheta*T/2)*T
	theta_car_2 = theta_car_2 + vtheta*T

poses_car_1 = np.array(poses_car_1)
poses_car_2 = np.array(poses_car_2)
poses_targ1 = np.array(poses_targ1)
poses_targ2 = np.array(poses_targ2)
fig, ax = plt.subplots(figsize=(14, 7))
ax.plot(poses_car_1[:,0],poses_car_1[:,1])
ax.plot(poses_car_2[:,0],poses_car_2[:,1])
ax.plot(poses_targ1[:,0],poses_targ1[:,1],'--')
ax.plot(poses_targ2[:,0],poses_targ2[:,1],'--')

plt.show()