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
controls=vertcat(a,delta)
EPSILON = 0
L=0.029



##########     Hyperparameters     #################

no_of_iters = 10
mass = 460 # in Kg
tolerance = 0.1
save_path_after = 2500
file_path_follow = "./coordinates_c.txt"  # File to read the global reference line, if None then centre line will be taken
file_path_lanel = './coordinates_l.txt'
file_path_lanec = './coordinates_c.txt'
file_path_laner = './coordinates_r.txt'
file_new_path_prefix = "./coordinates_nc" # File in which the new coordinates will be saved
Q_along=200  # Weight for progress along the road
Q_dist=0  # Weight for distance for the center of the road
penalty_out_of_road = 60000 # Penalise for planning path outside the road
no_iters = 6
max_no_of_vehicles = 0
R1=SX([[0,0],  # Weights for magnitude of speed and steering angles
    [0,0]])
R2=SX([[0,0],   # Weights for rate of change of speed and steering angle
    [0,0]])
T = .04 # Time horizon
N = 10 # Number of control intervals
kp=1 # For PID controller
obs_dist = 10 # To maintain from other vehicles at each time step
ki=0
kd=0
threshold = 20000
dist_threshold = 0.25
vmax = 1

##########     Global variables     #################

control_sample = np.zeros((2,N))


###########     Dynamic Model      ##################

rhs=[
        (v+a*T/2)*cos(theta+((atan(tan(delta/9.9)))/2)),
        (v+a*T/2)*sin(theta+((atan(tan(delta/9.9)))/2)),
        (v+a*T/2)*sin(atan(tan(delta/9.9)))/L,
        a
    ]
rhs=vertcat(*rhs)
f=Function('f',[states,controls],[rhs])
n_states=4
n_controls=2
U=SX.sym('U',n_controls,N)
P=SX.sym('P',9+4*max_no_of_vehicles+8)
X=SX.sym('X',n_states,(N+1))
g=SX.sym('g',2,2*N+2)
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
R = SX.sym('R',1,N)

for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    f_value=f(st,con)
    st_next=st+(T*f_value)
    X[:,k+1]=st_next
    g[0,N+k] = st[0]
    g[1,N+k] = st[1]

g[0,2*N] = X[0,1]
g[1,2*N] = X[1,1]
g[0,2*N+1] = X[2,1]
g[1,2*N+1] = X[3,1]

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
    #distance = ((st[0]-itr[no_iters-1,k])**2 + (st[1]-F_val[0,k])**2)**(1/2)
    itr_l[0,k]=st[0]
    for i in range(1,no_iters,1):
        F_dash_l[i,k] = P[-7]+2*P[-6]*itr_l[i-1,k]+3*P[-5]*itr_l[i-1,k]**2
        F_val_l[i,k] = P[-8]+P[-7]*itr_l[i-1,k]+P[-6]*itr_l[i-1,k]**2 + P[-5]*itr_l[i-1,k]**3
        itr_l[i,k] = itr_l[i-1,k]*((F_dash_l[i,k]**2)/(1+F_dash_l[i,k]**2))+(st[0]+F_dash_l[i,k]*(st[1]-F_val_l[i,k]))/(1+F_dash_l[i,k]**2)
    F_dash_l[0,k] = P[-7]+2*P[-6]*itr_l[no_iters-1,k]+3*P[-5]*itr_l[no_iters-1,k]**2
    F_val_l[0,k] = P[-8]+P[-7]*itr_l[no_iters-1,k]+P[-6]*itr_l[no_iters-1,k]**2 + P[-5]*itr_l[no_iters-1,k]**3
    distance =  ((st[0]-itr_l[no_iters-1,k])**2 + (st[1]-F_val_l[0,k])**2)**(1/2)
    
    # itr_r[0,k]=st[0]
    # for i in range(1,no_iters,1):
    #     F_dash_r[i,k] = P[-3]+2*P[-2]*itr_r[i-1,k]+3*P[-1]*itr_r[i-1,k]**2
    #     F_val_r[i,k] = P[-4]+P[-3]*itr_r[i-1,k]+P[-2]*itr_r[i-1,k]**2 + P[-1]*itr_r[i-1,k]**3
    #     itr_r[i,k] = itr_r[i-1,k]*((F_dash_r[i,k]**2)/(1+F_dash_r[i,k]**2))+(st[0]+F_dash_r[i,k]*(st[1]-F_val_r[i,k]))/(1+F_dash_r[i,k]**2)
    # F_dash_r[0,k] = P[-3]+2*P[-2]*itr_r[no_iters-1,k]+3*P[-1]*itr_r[no_iters-1,k]**2
    # F_val_r[0,k] = P[-4]+P[-3]*itr_r[no_iters-1,k]+P[-2]*itr_r[no_iters-1,k]**2 + P[-1]*itr_r[no_iters-1,k]**3
    # distance_r =  ((st[0]-itr_r[no_iters-1,k])**2 + (st[1]-F_val_r[0,k])**2)**(1/2)*(2*(st[1]<F_val_r[0,k])-1)
    
    R[0,k] = 1+((2*P[5]+6*P[6]*itr[no_iters-1,k]<-EPSILON)+(2*P[5]+6*P[6]*itr[no_iters-1,k]>EPSILON))*(-1+(((1+F_dash[0,k]**2)**(3/2))/(2*P[5]+6*P[6]*itr[no_iters-1,k]))/(((1+F_dash[0,k]**2)**(3/2))/(2*P[5]+6*P[6]*itr[no_iters-1,k]) - (st[1]-F_val[0,k]-F_dash[0,k]*(st[0]-itr[no_iters-1,k]))/(1+F_dash[0,k]**2)**(0.5)))
    g[0,k] =  st[3]
    #g[1,k] =  st[3]
    pen[0,k] = distance - tolerance
    pen[1,k] = distance - tolerance
    obj = obj + penalty_out_of_road*(pen[0,k]>0)*pen[0,k]**2 # Penalise for going out of left lane
    obj = obj + penalty_out_of_road*(pen[1,k]>0)*pen[1,k]**2 # Penalise for going out of right lane
    
    for t in range(max_no_of_vehicles) : 
        obj = obj + ((((P[9+4*t]-X[0,0])**2+(P[10+4*t]-X[1,0])**2))<100)*(obs_dist/((P[9+4*t]+(k)*P[11+4*t]*T-st[0])**2+(P[10+4*t]+(k)*P[12+4*t]*T-st[1])**2)) # To maintain safe distance from other vehicles
    
    obj = obj - Q_along*st[3]*cos(atan(F_dash[0,k])-st[2])*R[0,k] # To move along the lane 
    g[1,k] = Q_along*st[3]*cos(atan(F_dash[0,k])-st[2])*R[0,k]
    obj = obj + Q_dist*(P[3]+P[4]*st[0]+P[5]*st[0]*st[0]+P[6]*st[0]*st[0]*st[0]-st[1])**2 # Distance from the center lane
    obj = obj + con.T@R1@con # Penalise for more steering angle

g[1,1] = R[0,0]
g[1,2] = atan(F_dash[0,0])-X[2,0]

for k in range(0,N-1,1):
    prev_con=U[:,k]
    next_con=U[:,k+1]
    obj=obj+(prev_con- next_con).T@R2@(prev_con- next_con)

opt_variables=vertcat(U)
OPT_variables = reshape(U,2*N,1)
g_func = reshape(g,4*N+4,1)  
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
lbg=np.zeros(4*N+4)
ubg=np.zeros(4*N+4)

for k in range (0,2*N,2): 
    lbx[k]=0
    ubx[k]=1

for k in range (0,4*N,2):     
    if k<2*N :
        lbg[k]=-1
        ubg[k]=vmax
    else :
        lbg[k]=-1000
        ubg[k]=1000
lbg[4*N] = -1000
ubg[4*N] = 1000
lbg[4*N+1] = -1000
ubg[4*N+1] = 1000
lbg[4*N+2] = -1000
ubg[4*N+2] = 1000
lbg[4*N+3] = -1000
ubg[4*N+3] = 1000
for k in range (1,(2*N)+1,2):
    lbx[k]=-3.14
    ubx[k]=3.14

for k in range (1,(4*N)+1,2):
    if k<2*N :
        lbg[k]=-10000
        ubg[k]=10000
    else :
        lbg[k]=-10000
        ubg[k]=10000

#Initialisation

def dist1(x1,y1,x2,y2):
    return ((x1-x2)**2 + (y1-y2)**2)**(1/2)

def mpcCallback(trajectory_to_follow, lane_c, lane_l, lane_r, curr_pos, angle_heading, steering, speed):
    x_bot = 0
    y_bot = 0
    angle_ref = 0
    ####### Special regions ############
    
    yaw_car = 0 # yaw in radians
    current_pose=[x_bot,y_bot,yaw_car]
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
        points = [trajectory_to_follow[(mini),:]-curr_pos,trajectory_to_follow[(mini+5)%k,:]-curr_pos,trajectory_to_follow[(mini+10)%k,:]-curr_pos,trajectory_to_follow[(mini+15)%k,:]-curr_pos]
        k = lane_c.shape[0]
        mini = 0
        minval = 10000
        for j in range(k):
            if dist1(lane_c[j,0], lane_c[j,1], curr_pos[0], curr_pos[1]) < minval:
                minval = dist1(lane_c[j,0], lane_c[j,1], curr_pos[0], curr_pos[1])
                mini = j
        points_l = [lane_c[mini,:]-curr_pos,lane_c[(mini+5)%k,:]-curr_pos,lane_c[(mini+10)%k,:]-curr_pos,lane_c[(mini+15)%k,:]-curr_pos]
        points = np.array(points)
        points_l = np.array(points_l)
        # points_r = np.array(points_r)
        angle_ref = atan2(points[-1,1]-points[0,1],points[-1,0]-points[0,0])
        #print(angle_ref)
        tr_matrix = np.array([[cos(angle_ref),-sin(angle_ref)],[sin(angle_ref),cos(angle_ref)]])
        current_pose[2] = angle_heading - angle_ref
        #print(current_pose[2])
        points = np.matmul(points,tr_matrix)
        points_l = np.matmul(points_l,tr_matrix)
        # points_r = np.matmul(points_r,tr_matrix)
        M = np.array([points[:,0]**0,points[:,0] , points[:,0]**2, points[:,0]**3]).T
        M_l = np.array([points_l[:,0]**0,points_l[:,0] , points_l[:,0]**2, points_l[:,0]**3]).T
        # M_r = np.array([points_r[:,0]**0,points_r[:,0] , points_r[:,0]**2, points_r[:,0]**3]).T
        #print(points_l*100)
        #print(points_r*100)
        #print(points*100)
        #print(M)
        C = np.matmul(np.linalg.inv(M),points[:,1:])
        C_l = np.matmul(np.linalg.inv(M_l),points_l[:,1:])
        # C_r = np.matmul(np.linalg.inv(M_r),points_r[:,1:])
        curve = [C[0],C[1],C[2]+0.0001,C[3]+0.000001]
        curve_l = [C_l[0],C_l[1],C_l[2]+0.0001,C_l[3]+0.000001]
        #curve_r = [C_r[0],C_r[1],C_r[2]+0.0001,C_r[3]+0.000001]
        #print(curve_l)
        #print(curve_r)
        #print(curve)
    
    p=current_pose+curve+current_control
    plt.plot(points[:,0],points[:,1])
    plt.plot([0,0.03*cos(current_pose[2])],[0,0.03*sin(current_pose[2])])

    p = p+curve_l+curve
    mindist = 10000
    minindex = 0
    xc = x_bot
    yc = y_bot        
    control_sample[0,:] = 0
    control_sample[1,:] = 0
    x0=reshape(control_sample,2*N,1)
    so=solver(x0=x0,p=p,lbx=lbx,ubx=ubx,lbg=lbg,ubg=ubg) 
    u=so['x']
    g = so['g']
    #print(u)
    #print(g.shape)
    x = g[2*N:4*N:2]
    y = g[2*N+1:4*N+1:2]
    #print(x)
    #print(y)
    plt.plot(x,y)
    #plt.show()
    theta = float(g[4*N+2])
    v = float(g[4*N+3])
    progress = float(g[1])
    theta_diff = float(g[3])
    R_ratio = float(g[5])

    return [x[1],y[1],theta,v,progress,theta_diff,R_ratio,angle_ref]

file_new_path = file_path_follow
for j in range(no_of_iters):
    glob_x = 0
    glob_y = -1.65
    glob_theta = 0
    glob_v = 0.5
    T = 500
    print("Preparing",j,"trajectory :-")
    if file_path_follow != None:
        trajectory_to_follow = np.loadtxt(file_new_path,delimiter = ",")
        lane_l = np.loadtxt(file_path_lanel,delimiter = ",")[:2,:]
        lane_r = np.loadtxt(file_path_laner,delimiter = ",")[:2,:]
        lane_c = np.loadtxt(file_path_lanec,delimiter = ",")[:2,:]
    else :
        trajectory_to_follow=None

    traj_followed = []
    total_progress = []
    progress = 0
    dipped = False
    ci = 0
    worst_progress = 0
    for i in range(T):
        curr_pos = [glob_x, glob_y, glob_theta, glob_v, progress]
        traj_followed.append([glob_x, glob_y, glob_theta, glob_v])
        print("   ", i,":",curr_pos)
        if j!=0 and i<(T-20):
            #print(trajectory_to_follow.shape)
            traj_to_follow = trajectory_to_follow[:2,:]
            delta = mpcCallback(traj_to_follow.T, lane_c.T, lane_l.T, lane_r.T, curr_pos[:2], glob_theta, 0, glob_v)
        else :
            delta = mpcCallback(lane_c.T, lane_c.T, lane_l.T, lane_r.T, curr_pos[:2], glob_theta, 0, glob_v)

        #print(delta)
        #print(delta[0]*cos(delta[-1]) - delta[1]*sin(delta[-1]), delta[0]*sin(delta[-1]) + delta[1]*cos(delta[-1]))
        glob_x = glob_x + delta[0]*cos(delta[-1]) - delta[1]*sin(delta[-1])
        glob_y = glob_y + delta[0]*sin(delta[-1]) + delta[1]*cos(delta[-1])
        glob_theta = delta[-1] + float(delta[2]) 
        if j!=0:
            #print(trajectory_to_follow[3,i]*Q_along)
            #print(delta[4], delta[5], delta[6])
            #print(200*delta[5]*cos(delta[6])*glob_v)
            progress_delta = trajectory_to_follow[3,i]*Q_along - delta[4]
        else :
            progress_delta = glob_v*Q_along - delta[4]

        progress += progress_delta
        if (progress_delta<0 and dipped==False) :
            dipped = False
            worst_progress = progress
        
        glob_v = delta[3]
        if progress>worst_progress and dipped and j!=0 and i<(T-50):
            
            #plt.plot(np.array(traj_followed)[ci:i,0], np.array(traj_followed)[ci:i,1], 'k', lw=0.5, color = 'r')
            #plt.plot(trajectory_to_follow[0,ci:i], trajectory_to_follow[1,ci:i], 'k', lw=0.5, color = 'g')
            #plt.plot(coordinates_right[0], coordinates_right[1], 'k', lw=0.5, alpha=0.5)
            #plt.show()
            ci = i
            print(ci)
            glob_x = trajectory_to_follow[0,ci+1]
            glob_y = trajectory_to_follow[1,ci+1]
            glob_theta = trajectory_to_follow[2,ci+1]
            glob_v = trajectory_to_follow[3,ci+1]
            progress = 0
            dipped = False

    print(ci)
    traj_followed = np.array(traj_followed).T
    if j!=0 :
        traj_followed[:,:ci+1] = trajectory_to_follow[:,:ci+1]
    #print(total_progress)
    #print(traj_followed)
    file_new_path = file_new_path_prefix + str(j) + '.txt'
    np.savetxt(file_new_path, traj_followed, delimiter=',')
