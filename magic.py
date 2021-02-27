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

epsilon = 0
slips = np.arange(-1,1,0.001)
dfz = 0
fz = 7056
shx = phx1 + phx2*dfz
kx = slips + shx
Cx = pcx1
mux = pdx1 + pdx2*dfz
Dx = mux*fz
Ex = (pex1 + pex2*dfz + pex3*dfz**2)*(1-pex4)
K = fz*(pkx1+pkx2*dfz)*np.exp(pkx3*dfz)
Bx=K/(Cx*Dx+epsilon)
svx = fz*(pvx1+pvx2*dfz)
Fx = Dx*np.sin(Cx*np.arctan(Bx*kx - Ex*(Bx*kx-np.arctan(Bx*kx)))) + svx
plt.plot(slips,Fx)
dfz = 0.5
fz = 7056*(1+dfz)
shx = phx1 + phx2*dfz
kx = slips + shx
Cx = pcx1
mux = pdx1 + pdx2*dfz
Dx = mux*fz
Ex = (pex1 + pex2*dfz + pex3*dfz**2)*(1-pex4)
K = fz*(pkx1+pkx2*dfz)*np.exp(pkx3*dfz)
Bx=K/(Cx*Dx+epsilon)
svx = fz*(pvx1+pvx2*dfz)
Fx = Dx*np.sin(Cx*np.arctan(Bx*kx - Ex*(Bx*kx-np.arctan(Bx*kx)))) + svx
plt.plot(slips,Fx)
dfz = 1
fz = 7056*(1+dfz)
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
plt.plot(slips,Fx)
plt.show()