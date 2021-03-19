# -*- coding: utf-8 -*-
from sys import path as sys_path
from os import path as os_path
from casadi import *
import math
import numpy as np
import time
import rticonnextdds_connector as rti
import matplotlib.pyplot as plt
import bisect
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle 

global x_bot
global y_bot
global control_count
control_count=0
no_of_vehicles = 0
trajectory_to_follow = []
file_path = os_path.dirname(os_path.realpath(__file__))
sys_path.append(file_path + "/../../../")
has_start=True

###########   States    ####################

x=SX.sym('x')
y=SX.sym('y')
theta=SX.sym('theta')
v=SX.sym('v')
a=SX.sym('a')
states=vertcat(x,y,theta,v,wheel_ang_speed)
c=SX.sym('c')
delta=SX.sym('delta')
controls=vertcat(c,delta)
EPSILON = 0
L=2.9

######### Global variables ##################

predicted_x = 0
predicted_y = 0
predicted_theta = 0
predicted_v = 0

##########   Hyperparameters     #################

wheel_radius = 0.33
vehicle_length_r = 2
blocking_maneuver_cost = 0
start_throttle = 1 # Throttle to give at start
start_speed = 10 # Speed in m/s to give start_throttle
air_resistance_const = 0.43
mass = 720 # in Kg
tolerance = 1
Q_ang = 10
save_path_after = -1 # Save path after these no of iterations for visualization, -1 if path is not to be saved
file_path_follow = "./coordinates_c.txt"  # File to read the global reference line, if None then centre line will be taken
file_new_path = "./test.txt" # File in which the new coordinates will be saved
Q_along=2  # Weight for progress along the road
Q_dist=0  # Weight for distance for the center of the road
penalty_out_of_road = 6 # Penalise for planning path outside the road
no_iters = 3
max_no_of_vehicles = 4
R1=SX([[0,0],  # Weights for magnitude of speed and steering angles
    [0,10]])
R2=SX([[0,0],   # Weights for rate of change of speed and steering angle
    [0,0]])
T = .04 # Time horizon
N = 20 # Number of control intervals
kp=1 # For PID controller
obs_dist = 10 # To maintain from other vehicles at each time step
k_slip = 0.1
k_lat_slip = 0
ki=0
kd=0
threshold = 20000
dist_threshold = 0.25
kp_start = 2
ki_start = 0.05
kd_start = 1.5
I_start = 1
lift_coeff = 1.0655
gravity_constant = 9.8
pdy1=1.654
pdy2=-0.1783
fz0 = 3528

# Lateral slip coefficients
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


##########   Utils    #################

gear_throttles = [0,3499,4155,4651,5026,5482,9800]
gear_change_speed = [0,18.2,28.4,38.5,47,55.5,85]


def calc_force_from_slip(slip,speed) :
    fz = fz0 + lift_coeff*speed**2
    dfz = (fz-fz0)/fz0
    shx = phx1 + phx2*dfz
    kx = slip + shx
    Cx = pcx1
    mux = pdx1 + pdx2*dfz
    Dx = mux*fz
    Ex = (pex1 + pex2*dfz + pex3*dfz**2)*(1-pex4)
    K = (fz+dfz*fz)*(pkx1+pkx2*dfz)*np.exp(pkx3*dfz)
    Bx=K/(Cx*Dx+epsilon)
    svx = fz*(pvx1+pvx2*dfz)
    Fx = Dx*np.sin(Cx*np.arctan(Bx*kx - Ex*(Bx*kx-np.arctan(Bx*kx)))) + svx
    return Fx
    


def car_speed_to_gear_speed(curr_speed):
    indx=(bisect.bisect(gear_change_speed,curr_speed))
    m = (gear_throttles[indx]-gear_throttles[indx-1])/(gear_change_speed[indx]-gear_change_speed[indx-1])
    gear_speed = (m*(curr_speed- gear_change_speed[indx-1])) + gear_throttles[indx-1]
    return gear_speed

col_1 = [0,500,1500,2500,6300,6600,6900,7200,7500,7800,8100,
        8400,8700,9000,9300,9600,9800]

row_1 = [0,10,20,30,40,50,60,70,80,90,100]

eng_data = [[0,1.9,1.9,3.8,5.7,9.5,17.1,22.8,24.7,38,43.7],
[0,1.9,1.9,3.8,5.7,9.5,17.1,22.8,24.7,38,43.7],
[0,1.9,3.8,5.7,7.6,13.3,20.9,26.6,30.4,41.8,47.5],
[-5.7,0,1.9,5.7,7.6,15.2,24.7,30.4,34.2,45.6,49.97],
[-7.6,0,1.9,7.6,9.5,19,26.6,32.3,36.1,47.5,51.3],
[-9.5,1.9,3.8,9.5,11.4,20.9,28.5,34.2,38,47.5,51.3],
[-11.4,3.8,7.6,13.3,15.2,22.8,30.4,36.1,39.9,49.4,51.11],
[-13.3,1.9,11.4,17.1,19,26.6,34.2,38,43.7,51.3,52.25],
[-15.2,3.8,15.2,19,20.9,28.5,36.1,39.9,44.65,52.25,53.01],
[-17.1,1.9,11.4,17.1,19,26.6,34.2,38,43.7,51.3,52.25],
[-19,0,7.6,13.3,15.2,22.8,30.4,36.1,39.9,48.45,51.3],
[-22.8,-3.8,3.8,9.5,11.4,19,28.5,32.3,38,47.5,50.16],
[-24.7,-5.7,1.9,7.6,9.5,15.2,24.7,28.5,34.2,45.6,49.02],
[-24.7,-6.65,0,5.7,5.7,13.3,22.8,26.6,32.3,43.7,47.31],
[-26.6,-7.6,-1.9,1.9,3.8,11.4,20.9,24.7,30.4,41.8,44.65],
[-27.55,-9.5,-2.85,0,1.9,9.5,19,22.8,28.5,39.9,42.56],
[-28.5,-10.45,-3.8,-1.9,0,7.6,17.1,20.9,26.6,37.05,40.66]]

def calc_torque_from_gear_speed(gear_speed,curr_c):
    if(gear_speed==9800):
        gear_speed = 9800-0.01
    if(curr_c ==100):
        curr_c =100-0.01


    indx1=(bisect.bisect(row_1,curr_c))
    indx2=(bisect.bisect(col_1,gear_speed))
    print(indx1)
    print(indx2)
        
    x1 = row_1[indx1-1]
    x2 = row_1[indx1]
    y1 = col_1[indx2-1]
    y2 = col_1[indx2]
    print(x1,x2,y1,y2)
    q11 = eng_data[indx2-1][indx1-1]
    q12 = eng_data[indx2-1][indx1]
    q21 = eng_data[indx2][indx1-1]
    q22 = eng_data[indx2][indx1]
    print(q11,q12,q21,q22)
    x = curr_c
    y = gear_speed
    return (q11 * (x2 - x) * (y2 - y) +
            q21 * (x - x1) * (y2 - y) +
            q12 * (x2 - x) * (y - y1) +
            q22 * (x - x1) * (y - y1)
           ) / ((x2 - x1) * (y2 - y1) + 0.0)



# print(calc_fx_from_gear_speed(1000,100))
##########   Global variables    #################

control_sample = np.zeros((2,N))


###########  Dynamic Model    ##################

rhs=[
        (v)*cos(theta+((atan(tan(delta/9.9)))/2)),
        (v)*sin(theta+((atan(tan(delta/9.9)))/2)),
        (v)*sin(atan(tan(delta/9.9)))/L,
        calc_force_from_slip((wheel_ang_speed*wheel_radius- v)/v,v)/mass
        (calc_torque_from_gear_speed(car_speed_to_gear_speed(v),c)-wheel_radius*calc_force_from_slip((wheel_ang_speed*wheel_radius- v)/v))/inertia
    ]
rhs=vertcat(*rhs)
f=Function('f',[states,controls],[rhs])
n_states=5
n_controls=2
U=SX.sym('U',n_controls,N)
P=SX.sym('P',9+4*max_no_of_vehicles+8)
X=SX.sym('X',n_states,(N+1))
g=SX.sym('g',2,N+2)
X[:-2,0]=P[0:n_states-2]
X[-2,0]=P[7]
X[-1,0]=P[2]         
itr = SX.sym('I',no_iters,N)
itr_l = SX.sym('Il',no_iters,N)
itr_r = SX.sym('Ir',no_iters,N)
pen = SX.sym('p',2,N)
F_dash = SX.sym('Fd',no_iters,N)
F_val = SX.sym('Fv',no_iters,N)
F_dash_l = SX.sym('Fdl',no_iters,N)
F_val_l = SX.sym('Fvl',no_iters,N)
F_dash_r = SX.sym('Fdr',no_iters,N)
F_val_r = SX.sym('Fvr',no_iters,N)
other_vehicle_x = SX.sym('ovx',max_no_of_vehicles,N+1)
other_vehicle_y = SX.sym('ovy',max_no_of_vehicles,N+1)
other_vehicle_v = SX.sym('ovv',max_no_of_vehicles,N+1)
other_vehicle_t = SX.sym('ovt',max_no_of_vehicles,N+1)
R = SX.sym('R',1,1)

for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    f_value=f(st,con)
    st_next=st+(T*f_value)
    X[:,k+1]=st_next

g[0,N] = X[0,0]
g[1,N] = X[1,0]
g[0,N+1] = X[2,0]
g[1,N+1] = X[3,0]
ff=Function('ff',[U,P],[X])
obj=64000


################# P ###########
# (posx = 0, posy = 0, heading angle = 0)
# 0,1,2 : buffer, buffer and current wheel angular velocity
# 3,4,5,6 : C0, C1, C2 and C3 for cubic equation of reference line
# 7,8 : Intial speed and steering angle
# (9,10,11,12), (13,14,15,16) ...... (9+4k,10+4k,11+4k,12+4k) : (x,y,velx,vely) for all the surrounding vehicles
# (-8,-7,-6,-5) : Left lane boundary C0, C1, C2, C3
# (-4,-3,-2,-1) : Right lane boundary C0, C1, C2, C3

for t in range(max_no_of_vehicles) : 
    other_vehicle_x[t,0] = P[9+4*t]
    other_vehicle_y[t,0] = P[10+4*t]
    other_vehicle_v[t,0] = (P[11+4*t]**2 + P[12+4*t]**2)**0.5
    other_vehicle_t[t,0] = atan(P[12+4*t]/P[11+4*t])

for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    
    itr[0,k]=st[0]
    for i in range(1,no_iters,1):
        F_dash[i,k] = P[4]+2*P[5]*itr[i-1,k]+3*P[6]*itr[i-1,k]**2
        F_val[i,k] = P[3] + P[4]*itr[i-1,k] + P[5]*itr[i-1,k]**2 + P[6]*itr[i-1,k]**3
        itr[i,k] = itr[i-1,k]*((F_dash[i,k]**2)/(1+F_dash[i,k]**2))+(st[0]+F_dash[i,k]*(st[1]-F_val[i,k]))/(1+F_dash[i,k]**2)
    F_dash[0,k] = P[4]+2*P[5]*itr[no_iters-1,k]+3*P[6]*itr[no_iters-1,k]**2
    F_val[0,k] = P[3] + P[4]*itr[no_iters-1,k] + P[5]*itr[no_iters-1,k]**2 + P[6]*itr[no_iters-1,k]**3
    
    itr_l[0,k]=st[0]
    for i in range(1,no_iters,1):
        F_dash_l[i,k] = P[-7]+2*P[-6]*itr_l[i-1,k]+3*P[-5]*itr_l[i-1,k]**2
        F_val_l[i,k] = P[-8]+P[-7]*itr_l[i-1,k]+P[-6]*itr_l[i-1,k]**2 + P[-5]*itr_l[i-1,k]**3
        itr_l[i,k] = itr_l[i-1,k]*((F_dash_l[i,k]**2)/(1+F_dash_l[i,k]**2))+(st[0]+F_dash_l[i,k]*(st[1]-F_val_l[i,k]))/(1+F_dash_l[i,k]**2)
    F_dash_l[0,k] = P[-7]+2*P[-6]*itr_l[no_iters-1,k]+3*P[-5]*itr_l[no_iters-1,k]**2
    F_val_l[0,k] = P[-8]+P[-7]*itr_l[no_iters-1,k]+P[-6]*itr_l[no_iters-1,k]**2 + P[-5]*itr_l[no_iters-1,k]**3
    distance_l =  ((st[0]-itr_l[no_iters-1,k])**2 + (st[1]-F_val_l[0,k])**2)**(1/2)*(2*(st[1]>F_val_l[0,k])-1)
    
    itr_r[0,k]=st[0]
    for i in range(1,no_iters,1):
        F_dash_r[i,k] = P[-3]+2*P[-2]*itr_r[i-1,k]+3*P[-1]*itr_r[i-1,k]**2
        F_val_r[i,k] = P[-4]+P[-3]*itr_r[i-1,k]+P[-2]*itr_r[i-1,k]**2 + P[-1]*itr_r[i-1,k]**3
        itr_r[i,k] = itr_r[i-1,k]*((F_dash_r[i,k]**2)/(1+F_dash_r[i,k]**2))+(st[0]+F_dash_r[i,k]*(st[1]-F_val_r[i,k]))/(1+F_dash_r[i,k]**2)
    F_dash_r[0,k] = P[-3]+2*P[-2]*itr_r[no_iters-1,k]+3*P[-1]*itr_r[no_iters-1,k]**2
    F_val_r[0,k] = P[-4]+P[-3]*itr_r[no_iters-1,k]+P[-2]*itr_r[no_iters-1,k]**2 + P[-1]*itr_r[no_iters-1,k]**3
    distance_r =  ((st[0]-itr_r[no_iters-1,k])**2 + (st[1]-F_val_r[0,k])**2)**(1/2)*(2*(st[1]<F_val_r[0,k])-1)
    R[0,0] = (((1+F_dash[0,k]**2)**(3/2))/(2*P[5]+6*P[6]*itr[no_iters-1,k]))/(((1+F_dash[0,k]**2)**(3/2))/(2*P[5]+6*P[6]*itr[no_iters-1,k]) - (st[1]-F_val[0,k]-F_dash[0,k]*(st[0]-itr[no_iters-1,k]))/(1+F_dash[0,k]**2)**(0.5))
    g[0,k] =  0#distance_l
    g[1,k] =  0#distance_r
    pen[0,k] = distance_l + tolerance
    pen[1,k] = distance_r + tolerance
    obj = obj + penalty_out_of_road*(P[0]<10)*(pen[0,k]>0)*pen[0,k]**2 # Penalise for going out of left lane
    obj = obj + penalty_out_of_road*(P[0]<10)*(pen[1,k]>0)*pen[1,k]**2 # Penalise for going out of right lane
    Radius = (((1+F_dash[0,k]**2)**(3/2))/(2*P[5]+6*P[6]*itr[no_iters-1,k]))
    dFz = lift_coeff*st[3]**2
    dfz = dFz/Fz0
    muy = pdy1 + pdy2*dfz
    

    # Penalty for lateral slip
    lateral_acc_max = muy*gravity_constant
    lateral_acc_req = (st[3]**2)/Radius
    obj = obj + k_lat_slip*(lateral_acc_req>lateral_acc_max)*(lateral_acc_max - lateral_acc_req)**2

    # Penalty for longitudnal slip
    slip = (st[4]*wheel_radius-st[2])/st[2]
    obj = obj + (slip>0.05)*k_slip*slip**2
    
    for t in range(max_no_of_vehicles) : 
        x_v = (other_vehicle_x[t,k]-st[0])*cos(atan(F_dash[0,k]))+(other_vehicle_y[t,k]-st[1])*sin(atan(F_dash[0,k]))
        y_v = (other_vehicle_y[t,k]-st[1])*cos(atan(F_dash[0,k]))-(other_vehicle_x[t,k]-st[0])*sin(atan(F_dash[0,k]))
        obj = obj + blocking_maneuver_cost*(x_v < -vehicle_length_r)*(y_v>0)*(y_v)
        obj = obj + (x_v > -vehicle_length_r)*((((P[9+4*t]-X[0,0])**2+(P[10+4*t]-X[1,0])**2))<100)*(obs_dist/((other_vehicle_x[t,k]-st[0])**2+(other_vehicle_y[t,k]-st[1])**2)) # To maintain safe distance from other vehicles
        other_vehicle_t[t,k+1] = other_vehicle_t[t,k] + T*(other_vehicle_v[t,k]/Radius)
        other_vehicle_x[t,k+1] = other_vehicle_x[t,k] + other_vehicle_v[t,k]*cos(other_vehicle_t[t,k])*T
        other_vehicle_y[t,k+1] = other_vehicle_y[t,k] + other_vehicle_v[t,k]*sin(other_vehicle_t[t,k])*T
        other_vehicle_v[t,k+1] = other_vehicle_v[t,k]
    obj = obj + Q_ang*(atan(F_dash[0,k])-st[2])**2
    obj = obj - Q_along*st[3]*cos(atan(F_dash[0,k])-st[2])*R[0,0] # To move along the lane 
    obj = obj + Q_dist*(P[3]+P[4]*st[0]+P[5]*st[0]*st[0]+P[6]*st[0]*st[0]*st[0]-st[1])**2 # Distance from the center lane
    obj = obj + con.T@R1@con # Penalise for more steering angle

for k in range(0,N-1,1):
    prev_con=U[:,k]
    next_con=U[:,k+1]
    obj=obj+(prev_con- next_con).T@R2@(prev_con- next_con)

opt_variables=vertcat(U)
OPT_variables = reshape(U,2*N,1)
g_func = reshape(g,2*N+4,1)  
nlp_prob = {'f': obj, 'x':OPT_variables, 'p': P,'g':g_func}
options = {
            'ipopt.print_level' : 0,
            'ipopt.max_iter' : 500,
            'ipopt.mu_init' : 0.01,
            'ipopt.tol' : 1e-8,
            'ipopt.warm_start_init_point' : 'yes',
            'ipopt.warm_start_bound_push' : 1e-9,
            'ipopt.warm_start_bound_frac' : 1e-9,
            'ipopt.warm_start_slack_bound_frac' : 1e-9,
            'ipopt.warm_start_slack_bound_push' : 1e-9,
            'ipopt.mu_strategy' : 'adaptive',
            'print_time' : False,
            'verbose' : False,
            'expand' : True
        }

solver=nlpsol("solver","ipopt",nlp_prob,options)

lbx=np.zeros(2*N)
ubx=np.zeros(2*N)
lbg=np.zeros(2*(N+2))
ubg=np.zeros(2*N+4)

for k in range (0,2*N,2): 
    lbx[k]=-6000
    ubx[k]=1
    lbg[k]=-100
    ubg[k] = 100
    #if k>N//4:
    #   ubg[k]=0

for k in range (1,(2*N),2): 
    lbx[k]=-math.pi
    ubx[k]=math.pi
    lbg[k]=-100
    ubg[k] = 100
    #if k>N//4:
    #   ubg[k]=0

lbg[2*N] = -100000
lbg[2*N+1] = -100000
lbg[2*N+2] = -100000
lbg[2*N+3] = -100000
ubg[2*N] = 100000
ubg[2*N+1] = 100000
ubg[2*N+2] = 100000
ubg[2*N+3] = 100000

#Initialisation

def dist1(x1,y1,x2,y2):
    return ((x1-x2)**2 + (y1-y2)**2)**(1/2)

def mpcCallback(trajectory_to_follow, curr_pos, angle_heading, curve, curve_l, curve_r, steering, speed, goaltheta, all_vehicles, roadwidth, curr_slip):
    x_bot = 0
    y_bot = 0
    ####### Special regions ############
    if curr_pos[1]<-1174 and curr_pos[1]>-1452 and curr_pos[0]<-170:
        print("Special region")
        x_bot = 12
    yaw_car = 0 # yaw in radians
    current_pose=[x_bot,y_bot,speed*(1+curr_slip)/wheel_radius]
    current_control = [speed, steering]
    if file_path_follow!=None :
        k = trajectory_to_follow.shape[0]
        mini = 0
        minval = 10000
        #print(curr_pos)
        #print(trajectory_to_follow.shape)
        for i in range(k):
            if dist1(trajectory_to_follow[i,0], trajectory_to_follow[i,1], curr_pos[0], curr_pos[1]) < minval:
                minval = dist1(trajectory_to_follow[i,0], trajectory_to_follow[i,1], curr_pos[0], curr_pos[1])
                mini = i
        points = [trajectory_to_follow[mini,:]-curr_pos,trajectory_to_follow[(mini+1)%k,:]-curr_pos,trajectory_to_follow[(mini+2)%k,:]-curr_pos,trajectory_to_follow[(mini+3)%k,:]-curr_pos]
        points = np.array(points)
        tr_matrix = np.array([[cos(angle_heading),-sin(angle_heading)],[sin(angle_heading),cos(angle_heading)]])
        points = np.matmul(points,tr_matrix)
        #print(points)
        M = np.array([points[:,0]**0,points[:,0] , points[:,0]**2, points[:,0]**3]).T
        
        C = np.matmul(np.linalg.inv(M),points[:,1:])
        curve = [C[0],C[1],C[2],C[3]]
        #print(curve)
    
    p=current_pose+curve+current_control
    for i in range(max_no_of_vehicles) : 
        p = p+all_vehicles[i].tolist()
    p = p+curve_l+curve_r
    mindist = 10000
    minindex = 0
    xc = x_bot
    yc = y_bot
    for i in range(N):
        k=0
        for vehicle in all_vehicles :
            xd = vehicle[0]+vehicle[2]*i*T
            yd = vehicle[1]+vehicle[3]*i*T
            dist = (yc-yd)**2 + (xc-xd)**2
            if dist<mindist :
                mindist = dist
                minx = xd
                miny = yd
                minindex = k
            k = k+1
        xc+=speed*cos(atan(curve[1]+2*curve[2]*xc+3*curve[3]*xc**2))
        yc =curve[0]+curve[1]*xc+curve[2]*xc**2+curve[3]*xc**3

    print("min distance is ", mindist)
    if mindist < dist_threshold :
        # Path from left
        vehicle = all_vehicles[minindex]
        xc=0
        yc=0
        theta = 0
        linelx = []
        linely = []
        for i in range(N):
            if vehicle[3] > 0 : 
                xd = vehicle[0]+vehicle[2]*(N)*(T)-xc
                yd = vehicle[1]+vehicle[3]*(N)*(T)+1-yc
            else :
                xd = minx - xc
                yd = miny - yc + 1
            xr = xd*cos(theta) + yd*sin(theta)
            yr = -xd*sin(theta) + yd*cos(theta)
            control_sample[0,i] = 0
            Rd = (xr**2 + yr**2)/(2*yr)
            control_sample[1,i] = 58*yr/(xr**2 + yr**2)
            dx = Rd*sin((speed*T)/Rd)
            dy = Rd-Rd*cos((speed*T)/Rd)
            xc+=dx*cos(theta)-dy*sin(theta)
            yc+=dx*sin(theta)+dy*cos(theta)
            theta+=(speed*T)/Rd
            linelx.append(xc)
            linely.append(yc)

        plt.plot(linelx,linely,'ro-')
        xl=reshape(control_sample.copy(),2*N,1)
        sl=solver(x0=xl,p=p,lbx=lbx,ubx=ubx,lbg=lbg,ubg=ubg)
        costl = sl['f']
        xc=0
        yc=0
        theta = 0
        linerx = []
        linery = []
        for i in range(N):
            if vehicle[3] < 0 : 
                xd = vehicle[0]+vehicle[2]*(N)*(T)-xc
                yd = vehicle[1]+vehicle[3]*(N)*(T)-1-yc
            else :
                xd = minx - xc
                yd = miny - yc - 1
            xr = xd*cos(theta) + yd*sin(theta)
            yr = -xd*sin(theta) + yd*cos(theta)
            control_sample[0,i] = 0
            Rd = (xr**2 + yr**2)/(2*yr)
            control_sample[1,i] = 58*yr/(xr**2 + yr**2)
            dx = Rd*sin((speed*T)/Rd)
            dy = Rd-Rd*cos((speed*T)/Rd)
            xc+=dx*cos(theta)-dy*sin(theta)
            yc+=dx*sin(theta)+dy*cos(theta)
            theta+=(speed*T)/Rd
            linerx.append(xc)
            linery.append(yc)
        
        plt.plot(linerx,linery,'ro-')
        xrt=reshape(control_sample.copy(),2*N,1)
        sr=solver(x0=xrt,p=p,lbx=lbx,ubx=ubx,lbg=lbg,ubg=ubg)
        costr = sr['f']
        x = 0
        print("Cost from right : ", costr)
        print("Cost from left : ", costl)
        if(costr<=costl):
            x = sr['x']
        else :
            x = sl['x']
        #rect = []
        ax = plt.gca()
        for i in range(no_of_vehicles):
            temp = Rectangle((all_vehicles[i,0]-2,all_vehicles[i,1]-0.5),4,1,linewidth=1,edgecolor='r',facecolor='r')
            plt.plot([all_vehicles[i,0]],[all_vehicles[i,1]],'ro-')
            #rect.append(temp)
            ax.add_patch(temp)
            xv = []
            yv = []
            for j in range(N):
                vehicle = all_vehicles[i]
                xd = vehicle[0]+vehicle[2]*(j)*(T)
                yd = vehicle[1]+vehicle[3]*(j)*(T)
                xv.append(xd)
                yv.append(yd)

            plt.plot(xv,yv,'ro-')

        #pc = PatchCollection(rect, facecolor='r', alpha=0.5, edgecolor=None)
        #fig, ax = plt.subplots(1)
        # Add collection to axes
        #ax.add_collection(pc)
        #plt.show()
        u = reshape(x.T,2,N).T      
        ctrlmsg = u[:,1]
        speed_output = u[:,0]
        return ctrlmsg, speed_output
            
    control_sample[0,:] = 1
    control_sample[1,:] = 0
    x0=reshape(control_sample,2*N,1)
    #print(lbg)
    #print(ubg)
    so=solver(x0=x0,p=p,lbx=lbx,ubx=ubx,lbg=lbg,ubg=ubg) 
    x=so['x']
    g=so['g']
    predicted_x = g[2*N]
    predicted_y = g[2*N+1]
    predicted_theta = g[2*N+2]
    predicted_v = g[2*N+3]
    u = reshape(x.T,2,N).T
    ctrlmsg = u[:,1]
    control_output = u[:,0]
    return ctrlmsg, control_output


def detect_anomaly(vehicles, no_of_vehicles) :
    # for i in range(no_of_vehicles):
    #    if vehicles[i,3] < 5 :
    #        return True
    return False


with rti.open_connector(
        config_name="MyParticipantLibrary::ObstacleParticipant",
        url=file_path + "/../Sensors_ego2.xml") as connector:

    input1 = connector.get_input("roadSubscriber::roadReader1")
    input2 = connector.get_input("roadSubscriber::roadReader2")
    output = connector.get_output("steeringPublisher::steeringPub")
    input_speed = connector.get_input("StateSubscriber::stateReader")
    output_speed = connector.get_output("SpeedPublisher::speedPub")
    input_radar_F = connector.get_input("radarSubscriber_F::radarReader_F")
    input_radar_left = connector.get_input("radarSubscriber_left::radarReader_left")
    input_radar_right = connector.get_input("radarSubscriber_right::radarReader_right")
    controls = connector.get_output("controlPublisher::controlPub")
    wait_topic = connector.get_input("simWaitSub::simWaitReader")
    done_topic = connector.get_output("simDonePub::simDoneWriter")
    
    # Read data from the input, transform it and write it into the output
    print("Waiting for data...")
    
    #Initialise
    curr_steering = 0
    curr_speed = 0
    target_throttle = 0
    aggregate = 0
    nr_dist = 0
    curr_slip = 0
    all_vehicles = np.ones((max_no_of_vehicles,4))*10000
    if file_path_follow != None:
        trajectory_to_follow = np.loadtxt(file_path_follow,delimiter = ",")[:2,:]*100.0
    else :
        trajectory_to_follow=None
    traj_followed = []
    itr = 0
    total_itr=0
    P = 0
    I = I_start
    D = 0
    throttle = 0
    while True:
        total_itr=total_itr+1
        itr = itr+1
        if total_itr > save_path_after and save_path_after!=-1:
            break
        print("Iteration no", total_itr)
        input_radar_F.wait()
        input_radar_F.take()
        no_of_vehicles = 0
        all_vehicles[:,:2] = 10000
        all_vehicles[:,2] = 1
        all_vehicles[:,3] = 0
        wait_topic.wait()
        wait_topic.take()
        wait_msg = []
        for sample in wait_topic.samples.valid_data_iter:
            data = sample.get_dictionary()
            wait_msg = data
        
        for sample in input_radar_F.samples.valid_data_iter:
            data = sample.get_dictionary()
            
            for k in range(len(data['targetsArray'])):
                all_vehicles[no_of_vehicles,0] = data['targetsArray'][k]['posXInChosenRef']
                all_vehicles[no_of_vehicles,1] = data['targetsArray'][k]['posYInChosenRef']
                all_vehicles[no_of_vehicles,2] = data['targetsArray'][k]['absoluteSpeedX']
                all_vehicles[no_of_vehicles,3] = data['targetsArray'][k]['absoluteSpeedY']
                no_of_vehicles += 1
                print("Vehicle no ", no_of_vehicles)
                print("X : ", data['targetsArray'][k]['posXInChosenRef'])
                print("Y : ", data['targetsArray'][k]['posYInChosenRef'])
                print("Speed X : ", data['targetsArray'][k]['absoluteSpeedX'])
                print("Speed Y : ", data['targetsArray'][k]['absoluteSpeedY'])
            break
        input_radar_left.wait()
        input_radar_left.take()
        print("From left radar")
        for sample in input_radar_left.samples.valid_data_iter:
            data = sample.get_dictionary()
            for k in range(len(data['targetsArray'])):
                if(data['targetsArray'][k]['posXInChosenRef']<0 or data['targetsArray'][k]['posXInChosenRef']>5) :
                    continue
                all_vehicles[no_of_vehicles,0] = -data['targetsArray'][k]['posYInChosenRef']
                all_vehicles[no_of_vehicles,1] = data['targetsArray'][k]['posXInChosenRef']
                all_vehicles[no_of_vehicles,2] = -data['targetsArray'][k]['absoluteSpeedY']
                all_vehicles[no_of_vehicles,3] = data['targetsArray'][k]['absoluteSpeedX']
                no_of_vehicles +=1
                print("Vehicle no ", no_of_vehicles)
                print("X : ", -data['targetsArray'][k]['posYInChosenRef'])
                print("Y : ", data['targetsArray'][k]['posXInChosenRef'])
                print("Speed X : ", -data['targetsArray'][k]['absoluteSpeedY'])
                print("Speed Y : ", data['targetsArray'][k]['absoluteSpeedX'])
            break
        
        print("From right radar")
        input_radar_right.wait()
        input_radar_right.take()
        for sample in input_radar_right.samples.valid_data_iter:
            data = sample.get_dictionary()
            for k in range(len(data['targetsArray'])):
                if (data['targetsArray'][k]['posXInChosenRef']<0 or data['targetsArray'][k]['posXInChosenRef']>5) :
                    continue
                all_vehicles[no_of_vehicles,0] = data['targetsArray'][k]['posYInChosenRef']
                all_vehicles[no_of_vehicles,1] = -data['targetsArray'][k]['posXInChosenRef']
                all_vehicles[no_of_vehicles,2] = data['targetsArray'][k]['absoluteSpeedY']
                all_vehicles[no_of_vehicles,3] = -data['targetsArray'][k]['absoluteSpeedX']
                no_of_vehicles += 1
                print("Vehicle no ", no_of_vehicles)
                print("X : ", data['targetsArray'][k]['posYInChosenRef'])
                print("Y : ", -data['targetsArray'][k]['posXInChosenRef'])
                print("Speed X : ", data['targetsArray'][k]['absoluteSpeedY'])
                print("Speed Y : ", -data['targetsArray'][k]['absoluteSpeedX'])
            break
        input_speed.wait() # Wait for data in the input
        input_speed.take()
        px = 0
        py = 0
        angle_heading = 0
        lsr = 0
        slip_angle = 0
        for sample in input_speed.samples.valid_data_iter:
            data = sample.get_dictionary()
            vx = data['cdgSpeed_x']
            vy = data['cdgSpeed_y']
            vz = data['cdgSpeed_z']
            px = data['cdgPos_x']  
            py = data['cdgPos_y']  
            lsr = data['LSR']
            forcex = data['tireForce_x']
            normalz = data['groundNormal_z']
            angle_heading = data['cdgPos_heading']
            slip_angle = data['slipAngle']
            curr_pedal = data['gasPedal']
            curr_slip = slip_angle
            curr_speed = math.sqrt(vx*vx+vy*vy+vz*vz)
            print("Current State :",[px,py,angle_heading,curr_speed])
            print("Predicted State :",[predicted_x,predicted_y,predicted_theta,predicted_v])
            traj_followed.append([px,py,curr_speed,lsr[0],lsr[1],lsr[2],lsr[3],forcex[0],forcex[1],forcex[2],forcex[3],normalz[0],normalz[1],normalz[2],normalz[3],throttle])
            print("Current Speed : ", curr_speed)
            
        input1.wait() # Wait for data in the input
        input1.take()
        data1 = []
        data2 = []
        for sample in input1.samples.valid_data_iter:
            st10 = time.time()
            data1 = sample.get_dictionary()
            break
        input2.wait() # Wait for data in the input
        input2.take()
        for sample in input2.samples.valid_data_iter:
            st10 = time.time()
            data2 = sample.get_dictionary()
            break
            
        if detect_anomaly(all_vehicles,no_of_vehicles) :
            print("Anomaly detected")
            target_throttle = 0
            curr_steering = 0
        else:
            ll1 = data1['roadLinesPolynomsArray'][0]
            lr1 = data1['roadLinesPolynomsArray'][1]
            ll2 = data2['roadLinesPolynomsArray'][0]
            lr2 = data2['roadLinesPolynomsArray'][1]
            c0 = (ll1['c0'] + lr1['c0'] + ll2['c0'] + lr2['c0'])/4
            roadwidth = (ll1['c0']+ll2['c0'])/2-c0-1
            c1 = (ll1['c1'] + lr1['c1'] + ll2['c1'] + lr2['c1'])/4
            c2 = (ll1['c2'] + lr1['c2'] + ll2['c2'] + lr2['c2'])/4
            c3 = (ll1['c3'] + lr1['c3'] + ll2['c3'] + lr2['c3'])/4
            
            # If one of the lanes is not visible
            if (c0<-threshold) : 
                roadwidth = 2
                c0 = ll1['c0']-2*sqrt(1+ll1['c1']**2)
                c1 = ll1['c1']
                c2 = ll1['c2']
                c3 = ll1['c3']

            if (c0>threshold) :
                roadwidth = 2
                c0 = lr2['c0']+2*sqrt(1+lr2['c1']**2)
                c1 = lr2['c1']
                c2 = lr2['c2']
                c3 = lr2['c3']

            curve_l = [(ll1['c0']+ll2['c0'])/2,(ll1['c1']+ll2['c1'])/2,(ll1['c2']+ll2['c2'])/2,(ll1['c3']+ll2['c3'])/2]
            curve_r = [(lr1['c0']+lr2['c0'])/2,(lr1['c1']+lr2['c1'])/2,(lr1['c2']+lr2['c2'])/2,(lr1['c3']+lr2['c3'])/2]
            curve = [c0,c1,c2,c3]
            print("Time", data['TimeOfUpdate'])
            print("No of vehicles : ", no_of_vehicles)
            print("Curve left : ", curve_l)
            print("Curve right : ", curve_r)
            print("Curve : ", curve)
            curr_steering_array, target_speed_array = (mpcCallback(trajectory_to_follow.T, np.array([px,py]), angle_heading, curve, curve_l, curve_r, curr_steering, curr_speed, 0, all_vehicles, roadwidth))
            curr_steering = float(curr_steering_array[0])
            target_throttle = float(target_speed_array[0])
            out = {}
            if curr_speed < start_speed :
                P = kp_start*(0.06 - lsr[2])
                target_throttle = P + I + D
                I = I + ki_start*(0.06 - lsr[2])
                D = kd_start*0
                print("Current throttle : ", curr_pedal)
                print("slip angle : ",slip_angle[2])
                print("Slip ratio : ", lsr[2])
                print("Speed", curr_speed)
                # target_throttle = float(input("Enter throttle command value : "))
                if target_throttle<0 :
                    target_throttle = target_throttle#*100
            else :
                Q_ang = 0
            if curr_speed > 83 :
                target_throttle = (316.3*5/18) - curr_speed
            out['AcceleratorAdditive'] = max(0,target_throttle)
            out['AcceleratorMultiplicative'] = 0
            out['BrakeAdditive'] = -min(0,target_throttle)
            out['BrakeMultiplicative'] = 0
            out['ClutchAdditive'] = 0
            out['ClutchMultiplicative'] = 0
            out['GearboxAutoMode'] = 1
            out['GearboxTakeOver'] = 0
            out['IsRatioLimit'] = 0
            out['MaxRatio'] = 1000
            out['MinRatio'] = 1
            out['ParkingBrakeAdditive'] = 0
            out['ParkingBrakeMultiplicative'] = 0
            out['ShiftDown'] = 0
            out['ShiftUp'] = 0
            out['WantedGear'] = 1
            throttle = target_throttle
            out['TimeOfUpdate'] = data['TimeOfUpdate']
            output_speed.instance.set_dictionary(out)
            output_speed.write()
            print("Target Throttle:", target_throttle)
            out_steering = {}
            out_steering['AdditiveSteeringWheelAngle'] = curr_steering 
            out_steering['AdditiveSteeringWheelAccel'] = 0
            out_steering['AdditiveSteeringWheelSpeed'] = 0
            out_steering['AdditiveSteeringWheelTorque'] = 0
            out_steering['MultiplicativeSteeringWheelAccel'] = 1
            out_steering['MultiplicativeSteeringWheelAngle'] = 0
            out_steering['MultiplicativeSteeringWheelSpeed'] = 1
            out_steering['MultiplicativeSteeringWheelTorque'] = 1
            out_steering['TimeOfUpdate'] = data['TimeOfUpdate']
            print("Steering Command : " , curr_steering)
            output.instance.set_dictionary(out_steering)
            output.write()
            #print("Time: ", data['TimeOfUpdate'])
        done_topic.instance.set_dictionary(wait_msg)
        done_topic.write()
        print("message written")
        print("")
    
    traj_followed = np.array(traj_followed).T
    print("Trajectory followed :-")
    print(traj_followed)
    plt.plot(traj_followed[0],traj_followed[1],'k', lw=0.5, alpha=0.5)
    plt.plot(trajectory_to_follow[0]/100.0,trajectory_to_follow[1]/100.0,'--k', lw=0.5, alpha=0.5)
    np.savetxt(file_new_path, traj_followed, delimiter=',')
    plt.show()

if __name__ == '__main__':  
    start()