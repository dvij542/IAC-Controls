from casadi import *
import params as pars
x=SX.sym('x')
y=SX.sym('y')
theta=SX.sym('theta')
v=SX.sym('v')
a=SX.sym('a')
states=vertcat(x,y,theta,v)
c=SX.sym('c')
delta=SX.sym('delta')
controls=vertcat(c,delta)


R1=SX([[0,0],  # Weights for magnitude of speed and steering angles
    [0,10]])
R2=SX([[0,0],   # Weights for rate of change of speed and steering angle
    [0,0]])
rhs=[
        (v)*cos(theta+((atan(tan(delta/9.9)))/2)),
        (v)*sin(theta+((atan(tan(delta/9.9)))/2)),
        (v)*sin(atan(tan(delta/9.9)))/pars.L,
        ((c>0)*((v>=0)*(v<pars.gear_change_speeds[0])*c*pars.gear_throttles[0]+(v>=pars.gear_change_speeds[0])*(v<pars.gear_change_speeds[1])*c*pars.gear_throttles[1]+(v>=pars.gear_change_speeds[1])*(v<pars.gear_change_speeds[2])*c*pars.gear_throttles[2]+(v>=pars.gear_change_speeds[2])*(v<pars.gear_change_speeds[3])*c*pars.gear_throttles[3]+(v>=pars.gear_change_speeds[3])*(v<pars.gear_change_speeds[4])*c*pars.gear_throttles[4]+(v>=pars.gear_change_speeds[4])*c*pars.gear_throttles[5])-pars.air_resistance_const*v*v+(c<0)*c)/pars.mass
        # (c>=0)*calc_torque_from_gear_speed(car_speed_to_gear_speed(v),c)/(mass*get_gear_radii(v)) + (c<0)*c
    ]
rhs=vertcat(*rhs)
f=Function('f',[states,controls],[rhs])
n_states=4
n_controls=2
U=SX.sym('U',n_controls,pars.N)
P=SX.sym('P',9+4*pars.max_no_of_vehicles+8)
X=SX.sym('X',n_states,(pars.N+1))
g=SX.sym('g',2,pars.N+2)
X[:-1,0]=0
X[-1,0]=P[7]         
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
    f_value=f(st,con)
    st_next=st+(pars.T*f_value)
    X[:,k+1]=st_next

g[0,pars.N] = X[0,0]
g[1,pars.N] = X[1,0]
g[0,pars.N+1] = X[2,0]
g[1,pars.N+1] = X[3,0]
ff=Function('ff',[U,P],[X])
obj=64000