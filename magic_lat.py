import numpy as np
import matplotlib.pyplot as plt
import math

# Consts
pcy1 = 1.603
pdy1=1.654
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
mayvals = []
dfzs_1 = np.arange(0,3,0.1)
# for dfz in np.arange(0,1,0.1) :
	# mayvals.append(0)
# for speed in (0,83) :
	# dfz_rear, dfz_front = 
# 	for delta in (-0.5,0.5) :
#   	delta_rear , delta_forward = 

for dfz in np.arange(2,2.1,0.1) :
	epsilon = 0
	slips = np.arctan((-1.24703 - 0.222754*1.2)/63.8777)#np.arange(0,0.1,0.00001)
	dfz = dfz-1
	fz = 3600*(1+dfz)
	shy = phy1 + phy2*dfz
	ky = slips #+ shy
	Cy = pcy1
	muy = pdy1 + pdy2*dfz
	Dy = muy*fz
	Ey = (pey1 + pey2*dfz + pey3*dfz**2)*(1-pey4)
	K = fz*(pky1+pky2*dfz)*np.exp(pky3*dfz)
	By=K/(Cy*Dy+epsilon)
	svy = fz*(pvy1+pvy2*dfz+pvy3*dfz**2+pvy4*dfz**3)
	Fy = Dy*np.sin(Cy*np.arctan(By*ky - Ey*(By*ky-np.arctan(By*ky))))
	# plt.plot(slips*(180/3.14),Fy)
	# print(1+dfz, max(Fy))
	print(Fy)
	mayvals.append(max(Fy))
plt.plot(dfzs_1,mayvals)
plt.show()
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