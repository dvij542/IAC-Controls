import models as models
import params as pars
import util_funcs as utils
import math
import numpy as np
from casadi import *


################# models.P ###########
# Initial posx,posy and heading angle are 0
# 0 : No of vehicles
# 1,2,3,4,5,6 : Vi, Vf, C0, C1, C2 and C3 for cubic equation of reference line
# 7,8 : Intial speed and steering angle
# (9,10,11,12), (13,14,15,16) ...... (9+4k,10+4k,11+4k,12+4k) : (x,y,velx,vely) for all the surrounding vehicles
# (-8,-7,-6,-5) : Left lane boundary C0, C1, C2, C3
# (-4,-3,-2,-1) : Right lane boundary C0, C1, C2, C3

for t in range(2) : 
    models.other_vehicle_x[t,0] = models.P[9+4*t]
    models.other_vehicle_y[t,0] = models.P[10+4*t]
    models.other_vehicle_v[t,0] = (models.P[11+4*t]**2 + models.P[12+4*t]**2+0.01)**0.5
    models.other_vehicle_t[t,0] = atan2(models.P[12+4*t],models.P[11+4*t])#models.P[12+4*t]/models.P[11+4*t])

Vi = models.P[1]
Vf = models.P[2]
for k in range(0,pars.N,1):
    st=models.X[:,k]
    con=models.U[:,k]
    
    models.itr[0,k]=st[0]
    for i in range(1,pars.no_iters,1):
        models.F_dash[i,k] = models.P[4]+2*models.P[5]*models.itr[i-1,k]+3*models.P[6]*models.itr[i-1,k]**2
        models.F_val[i,k] = models.P[3] + models.P[4]*models.itr[i-1,k] + models.P[5]*models.itr[i-1,k]**2 + models.P[6]*models.itr[i-1,k]**3
        models.itr[i,k] = models.itr[i-1,k]*((models.F_dash[i,k]**2)/(1+models.F_dash[i,k]**2))+(st[0]+models.F_dash[i,k]*(st[1]-models.F_val[i,k]))/(1+models.F_dash[i,k]**2)
    models.F_dash[0,k] = models.P[4]+2*models.P[5]*models.itr[pars.no_iters-1,k]+3*models.P[6]*models.itr[pars.no_iters-1,k]**2
    models.F_val[0,k] = models.P[3] + models.P[4]*models.itr[pars.no_iters-1,k] + models.P[5]*models.itr[pars.no_iters-1,k]**2 + models.P[6]*models.itr[pars.no_iters-1,k]**3
    
    models.itr_l[0,k]=st[0]
    for i in range(1,pars.no_iters,1):
        models.F_dash_l[i,k] = models.P[-7]+2*models.P[-6]*models.itr_l[i-1,k]+3*models.P[-5]*models.itr_l[i-1,k]**2
        models.F_val_l[i,k] = models.P[-8]+models.P[-7]*models.itr_l[i-1,k]+models.P[-6]*models.itr_l[i-1,k]**2 + models.P[-5]*models.itr_l[i-1,k]**3
        models.itr_l[i,k] = models.itr_l[i-1,k]*((models.F_dash_l[i,k]**2)/(1+models.F_dash_l[i,k]**2))+(st[0]+models.F_dash_l[i,k]*(st[1]-models.F_val_l[i,k]))/(1+models.F_dash_l[i,k]**2)
    models.F_dash_l[0,k] = models.P[-7]+2*models.P[-6]*models.itr_l[pars.no_iters-1,k]+3*models.P[-5]*models.itr_l[pars.no_iters-1,k]**2
    models.F_val_l[0,k] = models.P[-8]+models.P[-7]*models.itr_l[pars.no_iters-1,k]+models.P[-6]*models.itr_l[pars.no_iters-1,k]**2 + models.P[-5]*models.itr_l[pars.no_iters-1,k]**3
    distance_l =  ((st[0]-models.itr_l[pars.no_iters-1,k])**2 + (st[1]-models.F_val_l[0,k])**2)**(1/2)*(2*(st[1]>models.F_val_l[0,k])-1)
    
    models.itr_r[0,k]=st[0]
    for i in range(1,pars.no_iters,1):
        models.F_dash_r[i,k] = models.P[-3]+2*models.P[-2]*models.itr_r[i-1,k]+3*models.P[-1]*models.itr_r[i-1,k]**2
        models.F_val_r[i,k] = models.P[-4]+models.P[-3]*models.itr_r[i-1,k]+models.P[-2]*models.itr_r[i-1,k]**2 + models.P[-1]*models.itr_r[i-1,k]**3
        models.itr_r[i,k] = models.itr_r[i-1,k]*((models.F_dash_r[i,k]**2)/(1+models.F_dash_r[i,k]**2))+(st[0]+models.F_dash_r[i,k]*(st[1]-models.F_val_r[i,k]))/(1+models.F_dash_r[i,k]**2)
    models.F_dash_r[0,k] = models.P[-3]+2*models.P[-2]*models.itr_r[pars.no_iters-1,k]+3*models.P[-1]*models.itr_r[pars.no_iters-1,k]**2
    models.F_val_r[0,k] = models.P[-4]+models.P[-3]*models.itr_r[pars.no_iters-1,k]+models.P[-2]*models.itr_r[pars.no_iters-1,k]**2 + models.P[-1]*models.itr_r[pars.no_iters-1,k]**3
    distance_r =  ((st[0]-models.itr_r[pars.no_iters-1,k])**2 + (st[1]-models.F_val_r[0,k])**2)**(1/2)*(2*(st[1]<models.F_val_r[0,k])-1)
    
    models.R[0,0] = (((1+models.F_dash[0,k]**2)**(3/2))/(2*models.P[5]+6*models.P[6]*models.itr[pars.no_iters-1,k]))/(((1+models.F_dash[0,k]**2)**(3/2))/(2*models.P[5]+6*models.P[6]*models.itr[pars.no_iters-1,k]) - (st[1]-models.F_val[0,k]-models.F_dash[0,k]*(st[0]-models.itr[pars.no_iters-1,k]))/(1+models.F_dash[0,k]**2)**(0.5))
    models.g[0,k] =  0 #distance_l
    models.g[1,k] =  0 #distance_r
    models.pen[0,k] = distance_l + pars.tolerance
    models.pen[1,k] = distance_r + pars.tolerance
    models.obj = models.obj + pars.penalty_out_of_road*(models.P[0]<10)*(models.pen[0,k]>0)*models.pen[0,k]**2 # Penalise for going out of left lane
    models.obj = models.obj + pars.penalty_out_of_road*(models.P[0]<10)*(models.pen[1,k]>0)*models.pen[1,k]**2 # Penalise for going out of right lane
    Radius = (((1+models.F_dash[0,k]**2)**(3/2))/(2*models.P[5]+6*models.P[6]*models.itr[pars.no_iters-1,k]))
    
    dFz = pars.lift_coeff*st[3]**2
    dfz = dFz/pars.fz0
    muy = (pars.pdy1 + pars.pdy2*dfz)*pars.road_coeff
    

    # Penalty for lateral slip
    lateral_acc_max = muy*pars.gravity_constant*(1+dfz)
    lateral_acc_req = (st[3]**2)/Radius
    models.obj = models.obj + pars.k_lat_slip*utils.sigmoid(10*(lateral_acc_req-lateral_acc_max))*(lateral_acc_max - lateral_acc_req)**2

    for t in range(2) : 
        x_v = (models.other_vehicle_x[t,k]-st[0])*cos(atan(models.F_dash[0,k]))+(models.other_vehicle_y[t,k]-st[1])*sin(atan(models.F_dash[0,k]))
        y_v = (models.other_vehicle_y[t,k]-st[1])*cos(atan(models.F_dash[0,k]))-(models.other_vehicle_x[t,k]-st[0])*sin(atan(models.F_dash[0,k]))
        # models.obj = models.obj + blocking_maneuver_cost*(x_v < -vehicle_length_r)*(y_v>0)*(y_v)*((((models.P[9+4*t]-models.X[0,0])**2+(models.P[10+4*t]-models.X[1,0])**2))<100)
        models.obj = models.obj + (models.P[9+4*t]<500)*(pars.obs_dist/((models.other_vehicle_x[t,k]-st[0])**2+(models.other_vehicle_y[t,k]-st[1])**2+0.5)) # To maintain safe distance from other vehicles
        models.other_vehicle_t[t,k+1] = models.other_vehicle_t[t,k] + pars.T*(models.other_vehicle_v[t,k]/Radius)
        models.other_vehicle_x[t,k+1] = models.other_vehicle_x[t,k] + models.other_vehicle_v[t,k]*cos(models.other_vehicle_t[t,k])*pars.T
        models.other_vehicle_y[t,k+1] = models.other_vehicle_y[t,k] + models.other_vehicle_v[t,k]*sin(models.other_vehicle_t[t,k])*pars.T
        models.other_vehicle_v[t,k+1] = models.other_vehicle_v[t,k]
    models.obj = models.obj + pars.Q_ang*(atan(models.F_dash[0,k])-st[2])**2
    models.obj = models.obj - (1-utils.sigmoid(10*(lateral_acc_req-lateral_acc_max)))*pars.Q_along*st[3]*cos(atan(models.F_dash[0,k])-st[2])*models.R[0,0]/3 # To move along the lane 
    required_val = Vi + (k+1)*(Vf-Vi)/pars.N
    models.obj = models.obj + ((st[3]>required_val)*pars.k_vel_follow*(required_val-st[3])**2)/25 # Cost for speed difference from optimal racing line speeed
    models.obj = models.obj + (pars.Q_dist*(models.P[3]+models.P[4]*st[0]+models.P[5]*st[0]*st[0]+models.P[6]*st[0]*st[0]*st[0]-st[1])**2)/25 # Distance from the center lane
    models.obj = models.obj + con.T@models.R1@con # Penalise for more steering angle

for k in range(0,pars.N-1,1):
    prev_con=models.U[:,k]
    next_con=models.U[:,k+1]
    models.obj=models.obj+(prev_con- next_con).T@models.R2@(prev_con- next_con)

opt_variables=vertcat(models.U)
OPT_variables = reshape(models.U,2*pars.N,1)
g_func = reshape(models.g,2*pars.N+4,1)  
nlp_prob = {'f': models.obj, 'x':OPT_variables, 'p': models.P,'g':g_func}
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

lbx=np.zeros(2*pars.N)
ubx=np.zeros(2*pars.N)
lbg=np.zeros(2*(pars.N+2))
ubg=np.zeros(2*pars.N+4)

for k in range (0,2*pars.N,2): 
    lbx[k]=-720*30
    ubx[k]=1
    lbg[k]=-100
    ubg[k] = 100
    #if k>pars.N//4:
    #   ubg[k]=0

for k in range (1,(2*pars.N),2): 
    lbx[k]=-math.pi
    ubx[k]=math.pi
    lbg[k]=-100
    ubg[k] = 100
    #if k>pars.N//4:
    #   ubg[k]=0

lbg[2*pars.N] = -100000
lbg[2*pars.N+1] = -100000
lbg[2*pars.N+2] = -100000
lbg[2*pars.N+3] = -100000
ubg[2*pars.N] = 100000
ubg[2*pars.N+1] = 100000
ubg[2*pars.N+2] = 100000
ubg[2*pars.N+3] = 100000