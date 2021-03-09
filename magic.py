import numpy as np
import matplotlib.pyplot as plt

# Consts
pcx1 = 2
pdx1=1.7168
pdx2=-0.289
pdx3=0
pex1 = 0.6975
pex2 = 0.20123
pex3 = 0
pex4 = 0
pkx1 = 63.75
pkx2 = -15
pkx3 = 0.2891
phx1 = -0.00058
phx2 = 0.00079
pvx1 = 0
pvx2 = 0
rbx1 = 17.406
rbx2 = 18.243
rcx1 = 0.7119
rex1 = -10
rex2 = 3.9185
rhx1 = 0.00211

# epsilon = 0
# slips = np.arange(-1,1,0.001)
# dfz = 0
# fz = 7056
# shx = phx1 + phx2*dfz
# kx = slips + shx
# Cx = pcx1
# mux = pdx1 + pdx2*dfz
# Dx = mux*fz
# Ex = (pex1 + pex2*dfz + pex3*dfz**2)*(1-pex4)
# K = fz*(pkx1+pkx2*dfz)*np.exp(pkx3*dfz)
# Bx=K/(Cx*Dx+epsilon)
# svx = fz*(pvx1+pvx2*dfz)
# Fx = Dx*np.sin(Cx*np.arctan(Bx*kx - Ex*(Bx*kx-np.arctan(Bx*kx)))) + svx
# plt.plot(slips,Fx)

# dfz = 0.5
# fz = 7056*(1+dfz)
# shx = phx1 + phx2*dfz
# kx = slips + shx
# Cx = pcx1
# mux = pdx1 + pdx2*dfz
# Dx = mux*fz
# Ex = (pex1 + pex2*dfz + pex3*dfz**2)*(1-pex4)
# K = fz*(pkx1+pkx2*dfz)*np.exp(pkx3*dfz)
# Bx=K/(Cx*Dx+epsilon)
# svx = fz*(pvx1+pvx2*dfz)
# Fx = Dx*np.sin(Cx*np.arctan(Bx*kx - Ex*(Bx*kx-np.arctan(Bx*kx)))) + svx
# plt.plot(slips,Fx)
fz0 = 3528
clf = 0.522
clr = 1.034
speeds = range(0,83)
epsilon = 0
ax_max_s = []
for speed in range(0,83) :
	fz = fz0 + (1/2)*1.225*(clf+clr)*speed**2
	dfz = (fz-fz0)/fz0
	slips = np.arange(-1,1,0.001)
	shx = phx1 + phx2*dfz
	kx = slips + shx
	Cx = pcx1
	mux = pdx1 + pdx2*dfz
	Dx = mux*fz
	Ex = (pex1 + pex2*dfz + pex3*dfz**2)*(1-pex4)
	K = (fz+dfz*fz)*(pkx1+pkx2*dfz)*np.exp(pkx3*dfz)
	Bx=K/(Cx*Dx+epsilon)
	svx = fz*(pvx1+pvx2*dfz)
	Fx = Dx*np.sin(Cx*np.arctan(Bx*kx - Ex*(Bx*kx-np.arctan(Bx*kx)))) + svx
	ax_max = max(Fx)/720
	ax_max_s.append(ax_max) 
plt.plot(speeds,ax_max_s)
outp = np.array([speeds,np.array(ax_max_s)]).T
np.savetxt('gg_part_x.csv',outp,delimiter=',')

ax_max_machine = []
for speed in range(0,83) :
	if(speed<7.85) :
		ax_max = 2*437/(0.33*720)
	elif (speed<23.4) :
		ax_max = 2*437/(0.33*720) + ((speed-7.85)/(23.4-7.85))*2*(475-437)/(0.33*720)
	elif (speed<39.2) :
		ax_max = 2*475/(0.33*720) + ((speed-23.4)/(39.2-23.4))*2*(499.7-475)/(0.33*720)
	else :
		ax_max = 2*499.7/(0.33*720) + ((speed-39.2)/(90-39.2))*2*(513-499.7)/(0.33*720)
	ax_max_machine.append(ax_max)

plt.plot(speeds,ax_max_machine)
outp = np.array([speeds,np.array(ax_max_machine)]).T
np.savetxt('ax_max_machine.csv',outp,delimiter=',')
plt.show()