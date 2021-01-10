# -*- coding: utf-8 -*-
from sys import path as sys_path
from os import path as os_path
from casadi import *
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle 

global x_bot
global y_bot
global control_count
control_count=0
no_of_vehicles = 0
trajectory_to_follow = []
has_start=True

###########   States    ####################

x=SX.sym('x')
y=SX.sym('y')
theta=SX.sym('theta')
v=SX.sym('v')
a=SX.sym('a')
states=vertcat(x,y,theta,v)
c=SX.sym('c')
delta=SX.sym('delta')
controls=vertcat(c,delta)
EPSILON = 0
L=2.9



##########     Hyperparameters     #################

mass = 460 # in Kg
tolerance = 2
save_path_after = 2500
file_path_follow = "./coordinates_c.txt"  # File to read the global reference line, if None then centre line will be taken
file_new_path = "./coordinates_nc1.txt" # File in which the new coordinates will be saved
Q_along=2  # Weight for progress along the road
Q_dist=0  # Weight for distance for the center of the road
penalty_out_of_road = 6 # Penalise for planning path outside the road
no_iters = 3
max_no_of_vehicles = 0
R1=SX([[0,0],  # Weights for magnitude of speed and steering angles
    [0,2]])
R2=SX([[0,0],   # Weights for rate of change of speed and steering angle
    [0,0]])
T = .04 # Time horizon
N = 20 # Number of control intervals
kp=1 # For PID controller
obs_dist = 10 # To maintain from other vehicles at each time step
ki=0
kd=0
threshold = 20000
dist_threshold = 0.25
vmax = 10

##########     Global variables     #################

control_sample = np.zeros((2,N))


###########     Dynamic Model      ##################

rhs=[
        (v)*cos(theta+((atan(tan(delta/9.9)))/2)),
        (v)*sin(theta+((atan(tan(delta/9.9)))/2)),
        (v)*sin(atan(tan(delta/9.9)))/L,
        a
    ]
rhs=vertcat(*rhs)
f=Function('f',[states,controls],[rhs])
n_states=4
n_controls=2
U=SX.sym('U',n_controls,N)
P=SX.sym('P',9+4*max_no_of_vehicles+8)
X=SX.sym('X',n_states,(N+1))
g=SX.sym('g',2,N)
X[:-1,0]=P[0:n_states-1]
X[-1,0]=P[7]         
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
R = SX.sym('R',1,1)

for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    f_value=f(st,con)
    st_next=st+(T*f_value)
    X[:,k+1]=st_next

ff=Function('ff',[U,P],[X])
obj=64000


################# P ###########
# 0,1,2 : Initial posx, posy and heading angle
# 3,4,5,6 : C0, C1, C2 and C3 for cubic equation of reference line
# 7,8 : Intial speed and steering angle
# (9,10,11,12), (13,14,15,16) ...... (9+4k,10+4k,11+4k,12+4k) : (x,y,velx,vely) for all the surrounding vehicles
# (-8,-7,-6,-5) : Left lane boundary C0, C1, C2, C3
# (-4,-3,-2,-1) : Right lane boundary C0, C1, C2, C3

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
    g[0,k] =  st[3]
    g[1,k] =  distance_r
    pen[0,k] = distance_l + tolerance
    pen[1,k] = distance_r + tolerance
    obj = obj + penalty_out_of_road*(P[0]<10)*(pen[0,k]>0)*pen[0,k]**2 # Penalise for going out of left lane
    obj = obj + penalty_out_of_road*(P[0]<10)*(pen[1,k]>0)*pen[1,k]**2 # Penalise for going out of right lane
    
    for t in range(max_no_of_vehicles) : 
        obj = obj + ((((P[9+4*t]-X[0,0])**2+(P[10+4*t]-X[1,0])**2))<100)*(obs_dist/((P[9+4*t]+(k)*P[11+4*t]*T-st[0])**2+(P[10+4*t]+(k)*P[12+4*t]*T-st[1])**2)) # To maintain safe distance from other vehicles
    
    obj = obj - Q_along*st[3]*cos(atan(F_dash[0,k])-st[2])*R[0,0] # To move along the lane 
    obj = obj + Q_dist*(P[3]+P[4]*st[0]+P[5]*st[0]*st[0]+P[6]*st[0]*st[0]*st[0]-st[1])**2 # Distance from the center lane
    obj = obj + con.T@R1@con # Penalise for more steering angle

for k in range(0,N-1,1):
    prev_con=U[:,k]
    next_con=U[:,k+1]
    obj=obj+(prev_con- next_con).T@R2@(prev_con- next_con)

opt_variables=vertcat(U)
OPT_variables = reshape(U,2*N,1)
g_func = reshape(g,2*N,1)  
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
lbg=np.zeros(2*N)
ubg=np.zeros(2*N)

for k in range (0,2*N,2): 
    lbx[k]=-6000
    ubx[k]=1
    lbg[k]=0
    ubg[k]=vmax
    
for k in range (1,(2*N)+1,2): 
    lbx[k]=-math.pi
    ubx[k]=math.pi
    lbg[k]=0
    ubg[k]=vmax

#Initialisation

def dist1(x1,y1,x2,y2):
    return ((x1-x2)**2 + (y1-y2)**2)**(1/2)

def mpcCallback(trajectory_to_follow, curr_pos, angle_heading, steering, speed):
    x_bot = 0
    y_bot = 0
    ####### Special regions ############
    if curr_pos[1]<-1174 and curr_pos[1]>-1452 and curr_pos[0]<-170:
        x_bot = 12
    yaw_car = 0 # yaw in radians
    current_pose=[x_bot,y_bot,yaw_car]
    current_control = [speed, steering]
    if file_path_follow!=None :
        k = trajectory_to_follow.shape[0]
        mini = 0
        minval = 10000
        print(curr_pos)
        print(trajectory_to_follow.shape)
        for i in range(k):
            if dist1(trajectory_to_follow[i,0], trajectory_to_follow[i,1], curr_pos[0], curr_pos[1]) < minval:
                minval = dist1(trajectory_to_follow[i,0], trajectory_to_follow[i,1], curr_pos[0], curr_pos[1])
                mini = i
        points = [trajectory_to_follow[mini,:]-curr_pos,trajectory_to_follow[(mini+1)%k,:]-curr_pos,trajectory_to_follow[(mini+2)%k,:]-curr_pos,trajectory_to_follow[(mini+3)%k,:]-curr_pos]
        points_l = [lane_l[mini,:]-curr_pos,lane_l[(mini+1)%k,:]-curr_pos,lane_l[(mini+2)%k,:]-curr_pos,lane_l[(mini+3)%k,:]-curr_pos]
        points_r = [lane_r[mini,:]-curr_pos,lane_r[(mini+1)%k,:]-curr_pos,lane_r[(mini+2)%k,:]-curr_pos,lane_r[(mini+3)%k,:]-curr_pos]
        
        points = np.array(points)
        points_l = np.array(points_l)
        points_r = np.array(points_r)
        tr_matrix = np.array([[cos(angle_heading),-sin(angle_heading)],[sin(angle_heading),cos(angle_heading)]])
        points = np.matmul(points,tr_matrix)
        points_l = np.matmul(points_l,tr_matrix)
        points_r = np.matmul(points_r,tr_matrix)
        M = np.array([points[:,0]**0,points[:,0] , points[:,0]**2, points[:,0]**3]).T
        M_l = np.array([points_l[:,0]**0,points_l[:,0] , points_l[:,0]**2, points_l[:,0]**3]).T
        M_r = np.array([points_r[:,0]**0,points_r[:,0] , points_r[:,0]**2, points_r[:,0]**3]).T
        
        C = np.matmul(np.linalg.inv(M),points[:,1:])
        C_l = np.matmul(np.linalg.inv(M_l),points_l[:,1:])
        C_r = np.matmul(np.linalg.inv(M_r),points_r[:,1:])
        curve = [C[0],C[1],C[2],C[3]]
        curve_l = [C_l[0],C_l[1],C_l[2],C_l[3]]
        curve_r = [C_r[0],C_r[1],C_r[2],C_r[3]]
        
        #print(curve)
    
    p=current_pose+curve+current_control
    p = p+curve_l+curve_r
    mindist = 10000
    minindex = 0
    xc = x_bot
    yc = y_bot        
    control_sample[0,:] = 1
    control_sample[1,:] = 0
    x0=reshape(control_sample,2*N,1)
    print(lbg)
    print(ubg)
    so=solver(x0=x0,p=p,lbx=lbx,ubx=ubx,lbg=lbg,ubg=ubg) 
    x=so['x']
    u = reshape(x.T,2,N).T
    ctrlmsg = u[:,1]
    control_output = u[:,0]
    return x[1,:]

glob_x = 0
glob_y = 0
glob_theta = 0
glob_v = 0
T = 100
if file_path_follow != None:
    trajectory_to_follow = np.loadtxt(file_path_follow,delimiter = ",")[:2,:]*100.0
else :
    trajectory_to_follow=None
traj_followed = []

for i in range(T):
    curr_pos = [glob_x, glob_y]
    print(curr_pos)
    delta = mpcCallback(trajectory_to_follow, curr_pos, glob_theta, 0, glob_v)
    glob_x += delta[0]*cos(theta) - delta[1]*sin(theta)
    glob_y += delta[0]*sin(theta) + delta[1]*cos(theta)
    glob_theta += delta[2]
    glob_v += delta[3]
    traj_followed.append([glob_x, glob_y, glob_theta, glob_v])