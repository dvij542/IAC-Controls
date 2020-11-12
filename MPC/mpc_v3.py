# -*- coding: utf-8 -*-
from sys import path as sys_path
from os import path as os_path
from casadi import *
import math
import numpy as np
import rticonnextdds_connector as rti
global x_bot
global y_bot
global control_count
control_count=0


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

Q_angle=2  # Weight for the difference in angle differences between ego vehicle and road
Q_dist=0.5  # Weight for distance for the center of the road
R1=SX([[-1.5,0],  # Weights for magnitude of speed and steering angles
    [0,2]])
R2=SX([[0.5,0],   # Weights for rate of change of speed and steering angle
    [0,0]])
T = .1 # Time horizon
N = 20 # Number of control intervals
v_max = 40 # Max speed (m/s)
kp=1 # For PID controller
obs_dist = 60
ki=0
kd=0
threshold = 20000

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
P=SX.sym('P',n_states*2+3+4)
X=SX.sym('X',n_states,(N+1))
X[:,0]=P[0:n_states]         

for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    f_value=f(st,con)
    st_next=st+(T*f_value)
    X[:,k+1]=st_next

ff=Function('ff',[U,P],[X])
obj=64000
g=0


for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    #obj = obj+;#+(P[n_states+7]+(k)*P[n_states+9]*T-st[1])*(P[n_states+7]+(k)*P[n_states+9]*T-st[1])))
    obj = obj+(obs_dist/((P[n_states+6]+(k)*P[n_states+8]*T-st[0])*(P[n_states+6]+(k)*P[n_states+8]*T-st[0])+(P[n_states+7]+(k)*P[n_states+9]*T-st[1])*(P[n_states+7]+(k)*P[n_states+9]*T-st[1])))+Q_angle*(P[n_states+1]-tan(st[2]))*(P[n_states+1]-tan(st[2])) + Q_dist*(P[n_states]+P[n_states+1]*st[0]+P[n_states+2]*st[0]*st[0]+P[n_states+3]*st[0]*st[0]*st[0]-st[1])*(P[n_states]+P[n_states+1]*st[0]+P[n_states+2]*st[0]*st[0]+P[n_states+3]*st[0]*st[0]*st[0]-st[1])+con.T@R1@con

obj = obj+(P[n_states+4:n_states+6] - U[:,0]).T@R2@(P[n_states+4:n_states+6] - U[:,0])
for k in range(0,N-1,1):
	prev_con=U[:,k]
	next_con=U[:,k+1]
	obj=obj+(prev_con- next_con).T@R2@(prev_con- next_con)

opt_variables=vertcat(U)
OPT_variables = reshape(U,2*N,1)

for k in range (0,N,1): 
    #g = (P[n_states+6]+(k)*P[n_states+8]*T-X[0,k])*(P[n_states+6]+(k)*P[n_states+8]*T-X[0,k])   
    g = X[0,k]
    g = X[1,k]  
   
   
nlp_prob = {'f': obj, 'x':OPT_variables, 'p': P,'g':g}
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

for k in range (0,2*N,2): 
    lbx[k]=0
    ubx[k]=v_max

for k in range (1,(2*N)-1,2): 
    lbx[k]=-math.pi
    ubx[k]=math.pi
u0=np.zeros((2,N))
u0[1,:] = -0.1 #Choose the rightmost path
u0[0,:] = v_max
x0=reshape(u0,2*N,1)


def mpcCallback(curve, steering, speed, goaltheta, nearest_vehicle):
    x_bot = 0
    y_bot = 0
    yaw_car = 0 # yaw in radians
    current_pose=[x_bot,y_bot,yaw_car]
    current_control = [speed, steering]
    p=current_pose+curve+current_control+nearest_vehicle
    so=solver(x0=x0,p=p,lbx=lbx,ubx=ubx) 
    x=so['x']
    u = reshape(x.T,2,N).T        
    st = current_pose
    con = u[0,:].T
    f_value = f(st,con)
    st = st+ (T*f_value)
    ctrlmsg = con[1]
    speed_output = con[0]
    return ctrlmsg, speed_output




with rti.open_connector(
        config_name="MyParticipantLibrary::ObstacleParticipant",
        url=file_path + "/../ShapeExample.xml") as connector:

    input = connector.get_input("roadSubscriber::roadReader")
    output = connector.get_output("steeringPublisher::steeringPub")
    input_speed = connector.get_input("StateSubscriber::stateReader")
    output_speed = connector.get_output("SpeedPublisher::speedPub")
    input_radar_F = connector.get_input("radarSubscriber_F::radarReader_F")
    input_radar_left = connector.get_input("radarSubscriber_left::radarReader_left")
    input_radar_right = connector.get_input("radarSubscriber_right::radarReader_right")

    # Read data from the input, transform it and write it into the output
    print("Waiting for data...")
    
    #Initialise
    curr_steering = 0
    curr_speed = 0
    target_speed = 0
    aggregate = 0
    x = 10000
    y = 10000
    vx = 0
    vy = 0
    nr_dist = 0
    while True:
        input_radar_F.wait()
        input_radar_F.take()
        for sample in input_radar_F.samples.valid_data_iter:
            data = sample.get_dictionary()
            nr = data['nearestTarget']
            if(nr<0) : 
                x = 10000
                y = 10000
                vx = 0
                vy = 0
                nr_dist = 100000
                break
            x = data['targetsArray'][nr]['posXInChosenRef']
            y = data['targetsArray'][nr]['posYInChosenRef']
            vx = data['targetsArray'][nr]['absoluteSpeedX']
            vy = data['targetsArray'][nr]['absoluteSpeedY']
            nr_dist = x*x + y*y
            print("X : ", data['targetsArray'][nr]['posXInChosenRef'])
            print("Y : ", data['targetsArray'][nr]['posYInChosenRef'])
            print("Speed X : ", data['targetsArray'][nr]['absoluteSpeedX'])
            print("Speed Y : ", data['targetsArray'][nr]['absoluteSpeedY'])
            break
        input_radar_left.wait()
        input_radar_left.take()
        for sample in input_radar_left.samples.valid_data_iter:
            data = sample.get_dictionary()
            nr = data['nearestTarget']
            if(nr<0) : 
                break
            x1 = data['targetsArray'][nr]['posXInChosenRef']
            y1 = data['targetsArray'][nr]['posYInChosenRef']
            vx1 = data['targetsArray'][nr]['absoluteSpeedX']
            vy1 = data['targetsArray'][nr]['absoluteSpeedY']
            nr_dist1 = x1*x1 + y1*y1
            if(nr_dist1<nr_dist) :
                x=-y1
                y=x1
                vx=-vy1
                vy=vx1
            print("STBD")
            print("X : ", -data['targetsArray'][nr]['posYInChosenRef'])
            print("Y : ", data['targetsArray'][nr]['posXInChosenRef'])
            print("Speed X : ", -data['targetsArray'][nr]['absoluteSpeedY'])
            print("Speed Y : ", data['targetsArray'][nr]['absoluteSpeedX'])
            break
        
        input_radar_right.wait()
        input_radar_right.take()
        for sample in input_radar_right.samples.valid_data_iter:
            data = sample.get_dictionary()
            nr = data['nearestTarget']
            if(nr<0) : 
                break
            x1 = data['targetsArray'][nr]['posXInChosenRef']
            y1 = data['targetsArray'][nr]['posYInChosenRef']
            vx1 = data['targetsArray'][nr]['absoluteSpeedX']
            vy1 = data['targetsArray'][nr]['absoluteSpeedY']
            nr_dist1 = x1*x1 + y1*y1
            if(nr_dist1<nr_dist) :
                x=y1
                y=-x1
                vx=vy1
                vy=-vx1
            print("STBD")
            print("X : ", -data['targetsArray'][nr]['posYInChosenRef'])
            print("Y : ", data['targetsArray'][nr]['posXInChosenRef'])
            print("Speed X : ", -data['targetsArray'][nr]['absoluteSpeedY'])
            print("Speed Y : ", data['targetsArray'][nr]['absoluteSpeedX'])
            break
        
        input.wait() # Wait for data in the input
        input.take()
        for sample in input.samples.valid_data_iter:
            data = sample.get_dictionary()
           
            if len(data['roadLinesPolynomsArray']) < 2 :
                continue
            ll = data['roadLinesPolynomsArray'][0]
            lr = data['roadLinesPolynomsArray'][1]
            c0 = (ll['c0'] + lr['c0'])/2
            c1 = (ll['c1'] + lr['c1'])/2
            c2 = (ll['c2'] + lr['c2'])/2
            c3 = (ll['c3'] + lr['c3'])/2
            out = {}
            if (c0<-threshold) : 
                c0 = ll['c0']-2*sqrt(1+ll['c1']*ll['c1'])
                c1 = ll['c1']
                c2 = ll['c2']
                c3 = ll['c3']

            if (c0>threshold) : 
                c0 = lr['c0']+2*sqrt(1+lr['c1']*lr['c1'])
                c1 = lr['c1']
                c2 = lr['c2']
                c3 = lr['c3']

            curve_l = [ll['c0'],ll['c1'],ll['c2'],ll['c3']]
            curve_r = [lr['c0'],lr['c1'],lr['c2'],lr['c3']]
            curve = [c0,c1,c2,c3]
            print("")
            print("Curve left : ", curve_l)
            print("Curve right : ", curve_r)
            print("Time", data['timeOfUpdate'])
            print("Curve : ", curve)
            curr_steering, target_speed = (mpcCallback(curve, curr_steering, curr_speed, 0, [x,y,vx,vy]))
            curr_steering = float(curr_steering)
            target_speed = float(target_speed)
            out['AdditiveSteeringWheelAngle'] = curr_steering         
            out['AdditiveSteeringWheelAccel'] = 0
            out['AdditiveSteeringWheelSpeed'] = 0
            out['AdditiveSteeringWheelTorque'] = 0
            out['MultiplicativeSteeringWheelAccel'] = 1
            out['MultiplicativeSteeringWheelAngle'] = 0
            out['MultiplicativeSteeringWheelSpeed'] = 1
            out['MultiplicativeSteeringWheelTorque'] = 1
            out['TimeOfUpdate'] = data['timeOfUpdate']
            print("Target Speed : ", target_speed)
            print("Steering Command : " , curr_steering)
            output.instance.set_dictionary(out)
            output.write()
            break
        
            
        input_speed.wait() # Wait for data in the input
        input_speed.take()
        for sample in input_speed.samples.valid_data_iter:
            data = sample.get_dictionary()
            vx = data['cdgSpeed_x']
            vy = data['cdgSpeed_y']
            vz = data['cdgSpeed_z']
            curr_speed = math.sqrt(vx*vx+vy*vy+vz*vz)
            print("Current Speed : ", curr_speed)
            out = {}
            throtle = kp*(target_speed-curr_speed)+ki*aggregate
            print("Pedal : ", throtle)
            aggregate = aggregate + (target_speed-curr_speed)
            out['AcceleratorAdditive'] = max(0,throtle)
            out['AcceleratorMultiplicative'] = 0
            out['BrakeAdditive'] = -min(0,throtle)
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
            break







if __name__ == '__main__':    

    start()