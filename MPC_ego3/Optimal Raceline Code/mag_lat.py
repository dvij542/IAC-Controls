from os import wait
import numpy as np
import matplotlib.pyplot as plt
from numpy.lib.function_base import select
from numpy.lib.utils import _lookfor_generate_cache
import json
# Consts
pcy1 = 1.603
pdy1=1.654*0.7
pdy2=-0.1783
pdy3=0
pey1 = -1.409
pey2 = -1.6617
pey3 = 0.26886
pey4 = -13.61
pky1 = -53.05
pky2 = 4.1265
pky3 = 1.5016
phy1 = 0.0039
phy2 = -0.00253
pvy1 = -0.01038
pvy2 = -0.1
pvy3 = 0.4498
pvy4 = -1.5

rby1 = 35.304
rby2 = 15.666
rby3 = -0.01179

rcy1 = 1.018
rey1 = 0.35475
rey2 = 0.01966
rhy1 = 0.00667
rhy2 = 0.00207

Cd=0.5971
Clf=-0.483
Clr=-0.912
S=1.247
output_dict={} # (speed,ay)
dfzs_1 = np.arange(0,3,0.1)
# for dfz in np.arange(0,1,0.1) :
	# mayvals.append(0)

rho=1.225
l_front=2
l_rear=1
fz0=7056
fz0_front=(fz0)/(1+l_front/l_rear)
fz0_rear=fz0-fz0_front

for speed in range(84) :
	D=(Clf+Clr)*(rho*speed*speed)*S/2
	fz=fz0+abs(D)
	dfz=(fz-fz0)/fz0
	fz_front=(fz)/(1+l_front/l_rear)
	fz_rear=fz-fz0_front
	dfz_rear=(fz_rear-fz0_rear)/fz0_rear
	dfz_front=(fz_front-fz0_front)/fz0_front
	output_dict[speed]=-1
	# for delta in (-0.175,0.175) :  # 10 degrees
	delta = np.arange(-0.175,0.175,0.0001)
	delta_front=delta/(1+l_front/l_rear)
	delta_rear=delta-delta_front
	
	epsilon_front = 0
	# fz_front = 7056*(1+dfz_front)
	shy_front = phy1 + phy2*dfz_front
	ky_front = delta_front+shy_front
	Cy_front = pcy1
	muy_front = pdy1 + pdy2*dfz_front
	Dy_front = muy_front*fz_front
	Ey_front = (pey1 + pey2*dfz_front + pey3*dfz_front**2)*(1-pey4)
	K_front = fz_front*(pky1+pky2*dfz_front)*np.exp(pky3*dfz_front)
	By_front=K_front/(Cy_front*Dy_front+epsilon_front)
	svy_front = fz_front*(pvy1+pvy2*dfz_front+pvy3*dfz_front**2+pvy4*dfz_front**3)
	Fy_front = Dy_front*np.sin(Cy_front*np.arctan(By_front*ky_front - Ey_front*(By_front*ky_front-np.arctan(By_front*ky_front))))
	ay_front=max(Fy_front)/720

	epsilon_rear = 0
	# fz_rear = 7056*(1+dfz_rear)
	shy_rear = phy1 + phy2*dfz_rear
	ky_rear = delta_rear+shy_rear
	Cy_rear = pcy1
	muy_rear = pdy1 + pdy2*dfz_rear
	Dy_rear = muy_rear*fz_rear
	Ey_rear = (pey1 + pey2*dfz_rear + pey3*dfz_rear**2)*(1-pey4)
	K_rear = fz_rear*(pky1+pky2*dfz_rear)*np.exp(pky3*dfz_rear)
	By_rear=K_rear/(Cy_rear*Dy_rear+epsilon_rear)
	svy_rear = fz_rear*(pvy1+pvy2*dfz_rear+pvy3*dfz_rear**2+pvy4*dfz_rear**3)
	Fy_rear = Dy_rear*np.sin(Cy_rear*np.arctan(By_rear*ky_rear - Ey_rear*(By_rear*ky_rear-np.arctan(By_rear*ky_rear))))
	ay_rear=max(Fy_rear)/720

	ay=ay_front+ay_rear
	# print(type(ay))
	output_dict[speed]=ay
	# print(Fy_front)
	# time.sleep(50)
	# print(1+dfz, max(Fy))
	# mayvals.append(max(Fy))	
# print("hi")
# plt.plot(list(output_dict.keys()),[output_dict[x] for x in output_dict.keys()])
# plt.show()

# with open(f"./output.json","w") as f:
# 	json.dump(output_dict,f)

with open(f"./ay.csv","w") as f:
	for x in output_dict:
		f.write(f"{x},{output_dict[x]}\n")

f.close()
# for dfz in np.arange(0,3,0.1) :
# 	epsilon = 0
# 	slips = np.arange(-0.8,0.8,0.00001)
# 	dfz = dfz-1
# 	fz = 7056*(1+dfz)
# 	shy = phy1 + phy2*dfz
# 	ky = slips + shy
# 	Cy = pcy1
# 	muy = pdy1 + pdy2*dfz
# 	Dy = muy*fz
# 	Ey = (pey1 + pey2*dfz + pey3*dfz**2)*(1-pey4)
# 	K = fz*(pky1+pky2*dfz)*np.exp(pky3*dfz)
# 	By=K/(Cy*Dy+epsilon)
# 	svy = fz*(pvy1+pvy2*dfz+pvy3*dfz**2+pvy4*dfz**3)
# 	Fy = Dy*np.sin(Cy*np.arctan(By*ky - Ey*(By*ky-np.arctan(By*ky))))
# 	# plt.plot(slips*(180/3.14),Fy)
# 	print(1+dfz, max(Fy))
# 	mayvals.append(max(Fy))
# plt.plot(dfzs_1,mayvals)
# plt.show()
# dfz = 0.5
# fz = 7056*(1+dfz)
# shy = phy1 + phy2*dfz
# ky = slips + shy
# Cy = pcy1
# muy = pdy1 + pdy2*dfz
# Dy = muy*fz
# Ey = (pey1 + pey2*dfz + pey3*dfz**2)*(1-pey4)
# K = fz*(pky1+pky2*dfz)*np.eyp(pky3*dfz)
# By=K/(Cy*Dy+epsilon)
# svy = fz*(pvy1+pvy2*dfz+pvy3*dfz**2+pvy4*dfz**3)
# Fy = Dy*np.sin(Cy*np.arctan(By*ky - Ey*(By*ky-np.arctan(By*ky))))
# plt.plot(slips*(180/3.14),Fy)
# print(may(Fy))
# dfz = 1
# fz = 7056*(1+dfz)
# shy = phy1 + phy2*dfz
# ky = slips + shy
# Cy = pcy1
# muy = pdy1 + pdy2*dfz
# Dy = muy*fz
# Ey = (pey1 + pey2*dfz + pey3*dfz**2)*(1-pey4)
# K = fz*(pky1+pky2*dfz)*np.eyp(pky3*dfz)
# By=K/(Cy*Dy+epsilon)
# svy = fz*(pvy1+pvy2*dfz+pvy3*dfz**2+pvy4*dfz**3)
# Fy = Dy*np.sin(Cy*np.arctan(By*ky - Ey*(By*ky-np.arctan(By*ky))))
# plt.plot(slips*(180/3.14),Fy)
# print(may(Fy))
# plt.show()
