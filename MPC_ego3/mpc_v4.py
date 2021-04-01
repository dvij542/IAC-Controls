# -*- coding: utf-8 -*-
from sys import path as sys_path
from os import path as os_path
from casadi import *
import math
import numpy as np
import time
import rticonnextdds_connector as rti
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle 

global x_bot
global y_bot
global control_count
control_count=0
no_of_vehicles = 0

file_path = os_path.dirname(os_path.realpath(__file__))
sys_path.append(file_path + "/../../../")
has_start=True

###########   States    ####################

x=SX.sym('x')
y=SX.sym('y')
theta=SX.sym('theta')
states=vertcat(x,y,theta)
v=SX.sym('v')
delta=SX.sym('delta')
controls=vertcat(v,delta)
EPSILON = 0
L=2.9



##########     Hyperparameters     #################

<<<<<<< HEAD
total_no_iters = 800
slip_penalty = 0
=======
slip_penalty = 2
>>>>>>> c3b7d61f569b55661c40269d4300a79eebf0bf06
FMax = 1  # mu X mass X g (The max force that can be exerted from the ground)
Q_along=2  # Weight for progress along the road
Q_dist=0  # Weight for distance for the center of the road
penalty_out_of_road = 3 # Penalise for planning path outside the road
no_iters = 3
max_no_of_vehicles = 4
R1=SX([[0,0],  # Weights for magnitude of speed and steering angles
    [0,2]])
R2=SX([[0.5,0],   # Weights for rate of change of speed and steering angle
    [0,0]])
T = .1 # Time horizon
N = 20 # Number of control intervals
<<<<<<< HEAD
v_max = 80 # Max speed (m/s)
=======
v_max = 50 # Max speed (m/s)
>>>>>>> c3b7d61f569b55661c40269d4300a79eebf0bf06
kp=1 # For PID controller
obs_dist = 10 # To maintain from other vehicles at each time step
ki=0
kd=0
threshold = 20000
dist_threshold = 0.25

##########     Global variables     #################

control_sample = np.zeros((2,N))


###########     Bicycle Model      ##################

rhs=[
        v*cos(theta+((atan(tan(delta/9.9)))/2)),
        v*sin(theta+((atan(tan(delta/9.9)))/2 )),
        v*sin((atan(tan(delta/9.9))) ) /L
    ]                                                                                   
rhs=vertcat(*rhs)
f=Function('f',[states,controls],[rhs])
n_states=3  
n_controls=2
U=SX.sym('U',n_controls,N)
P=SX.sym('P',10+4*max_no_of_vehicles)
X=SX.sym('X',n_states,(N+1))
X[:,0]=P[0:n_states]         
itr = SX.sym('I',no_iters,N)
pen = SX.sym('p',2,N)
F_dash = SX.sym('Fd',no_iters,N)
F_val = SX.sym('Fv',no_iters,N)
R = SX.sym('R',1,2)

st1 = time.time()
for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    f_value=f(st,con)
    st_next=st+(T*f_value)
    X[:,k+1]=st_next
et1 = time.time()
print("time 1 "+str(et1-st1))
ff=Function('ff',[U,P],[X])
obj=64000
g=0

st2 = time.time()
for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    itr[0,k]=st[0]
    for i in range(1,no_iters,1):
        F_dash[i,k] = P[n_states+1]+2*P[n_states+2]*itr[i-1,k]+3*P[n_states+3]*itr[i-1,k]**2
        F_val[i,k] = P[n_states] + P[n_states+1]*itr[i-1,k] + P[n_states+2]*itr[i-1,k]**2 + P[n_states+3]*itr[i-1,k]**3
        itr[i,k] = itr[i-1,k]*((F_dash[i,k]**2)/(1+F_dash[i,k]**2))+(st[0]+F_dash[i,k]*(st[1]-F_val[i,k]))/(1+F_dash[i,k]**2)
    F_dash[0,k] = P[n_states+1]+2*P[n_states+2]*itr[no_iters-1,k]+3*P[n_states+3]*itr[no_iters-1,k]**2
    F_val[0,k] = P[n_states] + P[n_states+1]*itr[no_iters-1,k] + P[n_states+2]*itr[no_iters-1,k]**2 + P[n_states+3]*itr[no_iters-1,k]**3
    R[0,0] = (((1+F_dash[0,k]**2)**(3/2))/(2*P[n_states+2]+6*P[n_states+3]*itr[no_iters-1,k]))/(((1+F_dash[0,k]**2)**(3/2))/(2*P[n_states+2]+6*P[n_states+3]*itr[no_iters-1,k]) - (st[1]-F_val[0,k]-F_dash[0,k]*(st[0]-itr[no_iters-1,k]))/(1+F_dash[0,k]**2)**(0.5))
    R[0,1] = (((1+F_dash[0,k]**2)**(3/2))/(2*P[n_states+2]+6*P[n_states+3]*itr[no_iters-1,k]))#/(((1+F_dash[0,k]**2)**(3/2))/(2*P[n_states+2]+6*P[n_states+3]*itr[no_iters-1,k]) - (st[1]-F_val[0,k]-F_dash[0,k]*(st[0]-itr[no_iters-1,k]))/(1+F_dash[0,k]**2)**(0.5))
    friction_excess_force = con[0]*con[0]/R[0,1] - FMax
    obj = obj + slip_penalty*(friction_excess_force>0)*(friction_excess_force**2) # Add cost for going beyong friction constraints of the road
    #pen[0,k] =  P[n_states]+P[n_states+1]*st[0]+P[n_states+2]*st[0]*st[0]+P[n_states+3]*st[0]*st[0]*st[0]-P[-1]-st[1]
    #pen[1,k] =  -(P[n_states]+P[n_states+1]*st[0]+P[n_states+2]*st[0]*st[0]+P[n_states+3]*st[0]*st[0]*st[0]+P[-1])+st[1]
    distance =  ((st[0]-itr[no_iters-1,k])**2 + (st[1]-F_val[0,k])**2)**(1/2)
    pen[0,k] =  distance-1.5
    pen[1,k] =  -(distance+1.5)
    
    obj = obj + penalty_out_of_road*(pen[0,k]>0)*pen[0,k]**2 # Penalise for going out of left lane
    obj = obj + penalty_out_of_road*(pen[1,k]>0)*pen[1,k]**2 # Penalise for going out of right lane
    for t in range(max_no_of_vehicles) : 
        obj = obj + ((((P[9+4*t]-X[0,0])**2+(P[10+4*t]-X[1,0])**2))<100)*(obs_dist/((P[9+4*t]+(k)*P[11+4*t]*T-st[0])**2+(P[10+4*t]+(k)*P[12+4*t]*T-st[1])**2)) # To maintain safe distance from other vehicles
    obj = obj - Q_along*con[0]*cos(atan(F_dash[0,k])-st[2])*R[0,0] # To move along the lane 
    obj = obj + Q_dist*(P[n_states]+P[n_states+1]*st[0]+P[n_states+2]*st[0]*st[0]+P[n_states+3]*st[0]*st[0]*st[0]-st[1])*(P[n_states]+P[n_states+1]*st[0]+P[n_states+2]*st[0]*st[0]+P[n_states+3]*st[0]*st[0]*st[0]-st[1]) # Distance from the center lane
    obj = obj + con.T@R1@con # Penalise for more steering angle
et2 = time.time()
print("time 2 "+str(et2-st2))
obj = obj+(P[n_states+4:n_states+6] - U[:,0]).T@R2@(P[n_states+4:n_states+6] - U[:,0])
st3 = time.time()
for k in range(0,N-1,1):
    prev_con=U[:,k]
    next_con=U[:,k+1]
    obj=obj+(prev_con- next_con).T@R2@(prev_con- next_con)
et3 = time.time()
print("time 3 "+str(et3-st3))
opt_variables=vertcat(U)
OPT_variables = reshape(U,2*N,1)
st4 = time.time()
for k in range (0,N,1): 
    g = X[0,k]
    g = X[1,k]  
et4 = time.time()
print("time 4 "+str(et4-st4)) 
   
nlp_prob = {'f': obj, 'x':OPT_variables, 'p': P,'g':g}
options = {
            'ipopt.print_level' : 0,
            'ipopt.max_iter' : 1000,
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
st5 = time.time()
for k in range (0,2*N,2): 
    lbx[k]=0
    ubx[k]=v_max
et5 = time.time()
print("time 5 "+str(et5-st5))
st6 = time.time()
for k in range (1,(2*N)-1,2): 
    lbx[k]=-math.pi
    ubx[k]=math.pi
et6 = time.time()
print("time 6 "+str(et6-st6))
#Initialisation



def mpcCallback(curve, steering, speed, goaltheta, all_vehicles, roadwidth):
    x_bot = 0
    y_bot = 0
    yaw_car = 0 # yaw in radians
    current_pose=[x_bot,y_bot,yaw_car]
    current_control = [speed, steering]
    p=current_pose+curve+current_control
    st12 = time.time()
    for i in range(max_no_of_vehicles) : 
        p = p+all_vehicles[i].tolist()
    et12 = time.time()
    print("time 12 "+str(et12-st12))
    mindist = 10000
    minindex = 0
    xc = x_bot
    yc = y_bot
    for i in range(N):
        k=0
        for vehicle in all_vehicles :
            xd = vehicle[0]+vehicle[2]*i*T
            yd = vehicle[1]+vehicle[3]*i*T
            dist = (yc-yd)**2
            if dist<mindist :
                mindist = dist
                minx = xd
                miny = yd
                minindex = k
            k = k+1
        xc+=speed

    print("min distance is ", mindist)
    if mindist < dist_threshold :
        # Path from left
        vehicle = all_vehicles[minindex]
        st13 = time.time()
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
            control_sample[0,i] = speed
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
        x0=reshape(control_sample,2*N,1)
        p = p + [roadwidth]
        sl=solver(x0=x0,p=p,lbx=lbx,ubx=ubx)
        costl = sl['f']
        et13 = time.time()
        print("time 13 "+str(et13-st13))
        st14 = time.time()
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
            control_sample[0,i] = speed
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
        x0=reshape(control_sample,2*N,1)
        sr=solver(x0=x0,p=p,lbx=lbx,ubx=ubx)
        costr = sr['f']
        et14 = time.time()
        print("time 14 "+str(et14-st14))
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
            
    control_sample[0,:] = speed
    control_sample[1,:] = 0
    x0=reshape(control_sample,2*N,1)
    p = p + [roadwidth]
    so=solver(x0=x0,p=p,lbx=lbx,ubx=ubx) 
    x=so['x']
    u = reshape(x.T,2,N).T        
    ctrlmsg = u[:,1]
    speed_output = u[:,0]
    return ctrlmsg, speed_output


def detect_anomaly(vehicles, no_of_vehicles) :
    for i in range(no_of_vehicles):
        if vehicles[i,3] < 5 :
            return True
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
    target_speed = 0
    aggregate = 0
    nr_dist = 0
    all_vehicles = np.ones((max_no_of_vehicles,4))*10000
    lane_new = []
    iter_no = 0
    while True:
        iter_no = iter_no + 1
        if iter_no > total_no_iters and total_no_iters != -1:
            break
        wait_topic.wait()
        wait_topic.take()
        wait_msg = []
        for sample in wait_topic.samples.valid_data_iter:
            data = sample.get_dictionary()
            wait_msg = data
        input_radar_F.wait()
        input_radar_F.take()
        no_of_vehicles = 0
        all_vehicles[:,:2] = 10000
        all_vehicles[:,2:] = 0
        for sample in input_radar_F.samples.valid_data_iter:
            data = sample.get_dictionary()
            st7 = time.time()
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
            et7 = time.time()
            print("time 7 "+str(et7-st7))
            break
        input_radar_left.wait()
        input_radar_left.take()
        for sample in input_radar_left.samples.valid_data_iter:
            data = sample.get_dictionary()
            st8 = time.time()
            for k in range(len(data['targetsArray'])):
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
            et8 = time.time()
            print("time 8 "+str(et8-st8))
            break
        
        input_radar_right.wait()
        input_radar_right.take()
        for sample in input_radar_right.samples.valid_data_iter:
            data = sample.get_dictionary()
            st9 = time.time()
            for k in range(len(data['targetsArray'])):
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
            et9 = time.time()
            print("time 9 "+str(et9-st9))
            break
        
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
            target_speed = 0
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
            print("")
            print("No of vehicles : ", no_of_vehicles)
            print("Curve left : ", curve_l)
            print("Curve right : ", curve_r)
            print("Time", data['timeOfUpdate'])
            print("Curve : ", curve)
            curr_steering_array, target_speed_array = (mpcCallback(curve, curr_steering, curr_speed, 0, all_vehicles, roadwidth))
            curr_steering = float(curr_steering_array[0])
            target_speed = float(target_speed_array[0])
        input_speed.wait() # Wait for data in the input
        input_speed.take()
        for sample in input_speed.samples.valid_data_iter:
            st11 = time.time()
            data = sample.get_dictionary()
            vx = data['cdgSpeed_x']
            vy = data['cdgSpeed_y']
            vz = data['cdgSpeed_z']
            accx = data['cdgAccel_x']
            accy = data['cdgAccel_y']
            posx = data['cdgPos_x']
            posy = data['cdgPos_y']
            curr_speed = math.sqrt(vx*vx+vy*vy+vz*vz)
            normalz = data['groundNormal_z']
            frictionx = data['tireForce_x']
            frictiony = data['tireForce_y']
            gear = data['GearEngaged']
            orientation = data['cdgPos_heading']
            if total_no_iters != -1 :
                lane_new.append([posx,posy,accx,accy,curr_speed] + normalz + frictionx + frictiony + [gear])
            print("Current Speed : ", curr_speed)
            out = {}
            throtle = kp*(target_speed-curr_speed)+ki*aggregate
            print("Pedal : ", throtle)
            aggregate = aggregate + (target_speed-curr_speed)
            out['AcceleratorAdditive'] = 1#max(0,throtle)
            out['AcceleratorMultiplicative'] = 0
            out['BrakeAdditive'] = 0#min(0,throtle)
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
            
            out['TimeOfUpdate'] = data['TimeOfUpdate']
            output_speed.instance.set_dictionary(out)
            output_speed.write()
            et11 = time.time()
            print("time 11 "+str(et11-st11))
            out = {}
            
            out['AdditiveSteeringWheelAngle'] = curr_steering         
            out['AdditiveSteeringWheelAccel'] = 0
            out['AdditiveSteeringWheelSpeed'] = 0
            out['AdditiveSteeringWheelTorque'] = 0
            out['MultiplicativeSteeringWheelAccel'] = 1
            out['MultiplicativeSteeringWheelAngle'] = 0
            out['MultiplicativeSteeringWheelSpeed'] = 1
            out['MultiplicativeSteeringWheelTorque'] = 1
            out['TimeOfUpdate'] = data['TimeOfUpdate']
            print("Target Speed : ", target_speed)
            print("Steering Command : " , curr_steering)
            output.instance.set_dictionary(out)
            output.write()
            et10 = time.time()
            print("time 10 "+str(et10-st10))
            break

        done_topic.instance.set_dictionary(wait_msg)
        done_topic.write()
        print("message written")
    lane_new = np.array(lane_new)
    plt.plot(lane_new[:,0],lane_new[:,1],'ro')
    np.savetxt('recorded_path.txt', lane_new, delimiter=',')
    plt.show()


if __name__ == '__main__':    

    start()