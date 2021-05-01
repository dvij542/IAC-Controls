# import os
# os.path.append("C:\\users\\dvij.kalaria\\appdata\\roaming\\python\\python37\\site-packages")
from casadi import *
import params as pars
import util_funcs as utils

# STATES
x=SX.sym('x')
y=SX.sym('y')
theta=SX.sym('theta')
v=SX.sym('v')
vperp=SX.sym('vperp')
w=SX.sym('w')
n_states=6
states=vertcat(x,y,theta,v,vperp,w)

#CONTROLS
delta=SX.sym('delta')
c=SX.sym('c')
n_controls=2
controls=vertcat(c,delta)

a=SX.sym('a')
xopp = SX.sym('xopp')
yopp = SX.sym('yopp')
targ=vertcat(xopp,yopp)
U=SX.sym('U',n_controls,pars.N)
P=SX.sym('P',9+4*pars.max_no_of_vehicles+10)
X=SX.sym('X',n_states,(pars.N+1))
opp = SX.sym('opp',2,pars.N)
g=SX.sym('g',2,pars.N+2)

def calc_drafting_coeff_drag(x,y,xopp,yopp):
    dx = xopp-x-pars.L/2 
    dy = yopp-y
    val = pars.DCd0 + pars.DCdx*dx + pars.Dcdy*dy*(2*utils.sigmoid(dy*5)-1)
    return if_else((dx>0)*(val<1),val,1)

def calc_throttle_force(c,v) :
    force = ((v>=0)*(v<pars.gear_change_speeds[0])*c*pars.gear_throttles[0]+ \
    (v>=pars.gear_change_speeds[0])*(v<pars.gear_change_speeds[1])*c*pars.gear_throttles[1]+ \
    (v>=pars.gear_change_speeds[1])*(v<pars.gear_change_speeds[2])*c*pars.gear_throttles[2]+ \
    (v>=pars.gear_change_speeds[2])*(v<pars.gear_change_speeds[3])*c*pars.gear_throttles[3]+ \
    (v>=pars.gear_change_speeds[3])*(v<pars.gear_change_speeds[4])*c*pars.gear_throttles[4]+ \
    (v>=pars.gear_change_speeds[4])*c*pars.gear_throttles[5])
    return force

def calc_force_rx(c,v,x,y,xopp,yopp,vperp):
    wind_force = pars.air_resistance_const*v*v*(1 + (calc_drafting_coeff_drag(x,y,xopp,yopp)-1)*(v/pars.vmax))
    return ((c>0)*calc_throttle_force(c,v)-wind_force+(c<0)*c)

def calc_force_fy(w,v,vperp,delta):
    alpha = -atan((w*pars.Lf + vperp)/(v)) + delta/pars.steering_ratio
    fz = pars.fz0*pars.Lr/(pars.Lf+pars.Lr) + pars.lift_coeff_f*v**2
    fz0 = 4000#pars.fz0*pars.Lr/(pars.Lf+pars.Lr)
    return utils.calc_force_from_slip_angle(-alpha,fz,fz0)   
    

def calc_force_ry(w,v,vperp):
    alpha = atan((w*pars.Lr - vperp)/(v))
    fz = pars.fz0*pars.Lf/(pars.Lf+pars.Lr) + pars.lift_coeff_r*v**2
    fz0 = 4000#pars.fz0*pars.Lf/(pars.Lf+pars.Lr)
    return utils.calc_force_from_slip_angle(-alpha,fz,fz0)   

R1=SX([[0,0],  # Weights for magnitude of speed and steering angles
    [0,0.1]])
R2=SX([[2,0],   # Weights for rate of change of speed and steering angle
    [0,500]])

print(pars.moment_of_inertia)
# UPDATE RULE
rhs=[
        v*cos(theta) - vperp*sin(theta),
        v*sin(theta) + vperp*cos(theta),
        w,
        (calc_force_rx(c,v,x,y,xopp,yopp,vperp) - calc_force_fy(w,v,vperp,delta)*sin(delta/pars.steering_ratio) + pars.mass*vperp*w)/pars.mass, # Including drafting
        (calc_force_ry(w,v,vperp) + calc_force_fy(w,v,vperp,delta)*cos(delta/pars.steering_ratio) - pars.mass*v*w)/pars.mass,
        (calc_force_fy(w,v,vperp,delta)*pars.Lf*cos(delta/pars.steering_ratio) - calc_force_ry(w,v,vperp)*pars.Lr)/pars.moment_of_inertia
        # (c>=0)*calc_torque_from_gear_speed(car_speed_to_gear_speed(v),c)/(mass*get_gear_radii(v)) + (c<0)*c
    ]
rhs=vertcat(*rhs)
f=Function('f',[states,controls,targ],[rhs])

X[:-3,0]=0
X[-3,0]=P[7]   
X[-2,0]=P[-10]
X[-1,0]=P[-9]
itr = SX.sym('I',pars.no_iters,pars.N)
itr_l = SX.sym('Il',pars.no_iters,pars.N)
itr_r = SX.sym('Ir',pars.no_iters,pars.N)
pen = SX.sym('p',2,pars.N)
F_dash = SX.sym('Fd',pars.no_iters,pars.N)
F_val = SX.sym('Fv',pars.no_iters,pars.N)
F_dash_l = SX.sym('Fdl',pars.no_iters,pars.N)
F_val_l = SX.sym('Fvl',pars.no_iters,pars.N)
F_dash_r = SX.sym('Fdr',pars.no_iters,pars.N)
F_val_r = SX.sym('Fvr',pars.no_iters,pars.N)
other_vehicle_x = SX.sym('ovx',pars.max_no_of_vehicles,pars.N+1)
other_vehicle_y = SX.sym('ovy',pars.max_no_of_vehicles,pars.N+1)
other_vehicle_v = SX.sym('ovv',pars.max_no_of_vehicles,pars.N+1)
other_vehicle_t = SX.sym('ovt',pars.max_no_of_vehicles,pars.N+1)
R = SX.sym('R',1,1)

for k in range(0,pars.N,1):
    st=X[:,k]
    con=U[:,k]
    theta = atan2(P[12],P[11])
    opp[0,k] = P[9]+P[11]*k*pars.T
    opp[1,k] = P[10]+P[12]*k*pars.T
    target = opp[:,k]#[P[9]+P[11]*k*pars.T,P[10]+P[12]*k*pars.T]
    f_value=f(st,con,target)
    st_next=st+(pars.T*f_value)
    X[:,k+1]=st_next

g[0,pars.N] = X[0,0]
g[1,pars.N] = X[1,0]
g[0,pars.N+1] = X[2,0]
g[1,pars.N+1] = X[3,0]
ff=Function('ff',[U,P],[X])
obj=4000