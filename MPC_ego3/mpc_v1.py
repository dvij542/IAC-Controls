# -*- coding: utf-8 -*-
from sys import path as sys_path
from os import path as os_path
from casadi import *
import math
import numpy as np


file_path = os_path.dirname(os_path.realpath(__file__))
sys_path.append(file_path + "/../../../")
import rticonnextdds_connector as rti
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
global x_bot
global y_bot
global control_count
control_count=0
has_start=True
T = .1 # Time horizon
N = 5# number of control intervals
###########   states    ####################
x=SX.sym('x')
y=SX.sym('y')
theta=SX.sym('theta')
states=vertcat(x,y,theta)
v=SX.sym('v')
a = SX.sym('a')
delta=SX.sym('delta')
controls=vertcat(a,delta)
EPSILON = 1e-5
L=29
rhs=[
        v*cos(theta+((atan(tan(delta))/2)+EPSILON)),
        v*sin(theta+((atan(tan(delta))/2)+EPSILON )),
        2*v*sin((atan(tan(delta))/2)+EPSILON )/L,
        a
    ]                                                                                   
rhs=vertcat(*rhs)
f=Function('f',[states,controls],[rhs])
#print(f)
n_states=3  
n_controls=2
U=SX.sym('U',n_controls,N)
P=SX.sym('P',n_states*2)
X=SX.sym('X',n_states,(N+1))
#print(X[1,1])
X[:,0]=P[0:n_states]         
#print(X[:,0])
for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    #print((st))
    f_value=f(st,con)
    #print("f_value  ",type(f_value))
    st_next=st+(T*f_value)
    X[:,k+1]=st_next

ff=Function('ff',[U,P],[X])
obj=0
Q=SX([[10,0,0],
    [0,10,0],
    [0,0,8]
    ])
R=SX([[0,0],
    [0,0]])

for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    obj=obj+(((st- P[n_states:6]).T)@Q)@(st- P[n_states:6])+con.T@R@con
opt_variables=vertcat(U)
OPT_variables = reshape(U,2*N,1)
#print(X[1,k])
for k in range (0,N+1,1): 
    g = X[0,k]   
    g = X[1,k]  
    #g = X[2,k]

nlp_prob = {'f': obj, 'x':OPT_variables, 'p': P,'g':g}
options = {

            'ipopt.print_level' : 0,

            'ipopt.max_iter' : 150,

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



ubx=math.pi/2
lbx=-math.pi/2
u0=np.random.rand(N,2)
x0=reshape(u0,2*N,1)


def mpcCallback(goalx, goaly, goaltheta):

    
    global x_dot

    global y_dot

    global psi_dot

    global epsi

    global ey

    global s

    global yaw

    global vel

    global ep # min distance

    global cp # index of closest point

    global ep_max

    global ep_sum

    global ep_avg

    global n

    global cp1

    global path_length

    global control_count

    global has_start

    has_start=True

    control_count=0   

    global goal_point



    #goal_point_index=calculate_goal_point_index()



    goal_point = [goalx, goaly]

    x_bot_init = 0

    y_bot_init = 0

    yaw_car_init = 0 # yaw in radians
    
    current_pose=[x_bot_init,y_bot_init,yaw_car_init]

    print('*******  current pose:',current_pose)



    #current_pose=[0,0,0]

    xs=[goalx,goaly,goaltheta]
    #print('x_dot',x_dot )

    # x_next=x_init

    p=current_pose+xs

    print('*********************************************************************************')

    print('*********************************************************************************')

    print('*********************************************************************************')

    print('goal point has changed')

    print("*********  current_pose",current_pose)

    print('xs **********',xs)

    print('*********************************************************************************')

    print('*********************************************************************************')

    print('*********************************************************************************')



    #print('**********goal point index',goal_point_index)



    #while(((np.linalg.norm(np.subtract(x_next[0],xs[0])))+(np.linalg.norm(np.subtract(x_next[1],xs[1]))))>1e-1):

    count=0

    x_bot = 0

    y_bot = 0

        # print("x_bot",x_bot)

        # print('y_bot',y_bot)

        # print("total index       ",len(x_p.poses))
    yaw_car = 0 # yaw in radians

    current_pose=[x_bot,y_bot,yaw_car]

        #print('*******  current pose:',current_pose)

    print('###############################   Inside while loop                   ##########################################################')

    print('###############################   Inside while loop                   ##########################################################')
            #e=sqrt(((current_pose[0] - xs[0])**2 + (current_pose[1] - xs[1])**2)+(current_pose[2]-xs[2])**2)

    print('************************************************************')

    print("error       :",e)

        

    so=solver(x0=x0,p=p,lbx=lbx,ubx=ubx) 

    x=so['x']

    u = reshape(x.T,2,N).T        

    st = current_pose

    print('st     ',st) 
        #print("***********     init   state",st)
    con = u[0,:].T
    print('********controls',con)
    f_value = f(st,con)
    st = st+ (T*f_value)   
    ctrlmsg = con[1]

    return ctrlmsg




with rti.open_connector(
        config_name="MyParticipantLibrary::MyParticipant",
        url=file_path + "/../Sensors_ego2.xml") as connector:

    input = connector.get_input("roadSubscriber::roadReader")
    output = connector.get_output("steeringPublisher::steeringPub")

    # Read data from the input, transform it and write it into the output
    print("Waiting for data...")
    while True:
        input.wait() # Wait for data in the input
        input.take()
        for sample in input.samples.valid_data_iter:
            data = sample.get_dictionary()
           
            if len(data['roadLinesPolynomsArray']) < 2 :
                continue
            ll = data['roadLinesPolynomsArray'][0]
            lr = data['roadLinesPolynomsArray'][1]
            print(ll)
            c0 = (ll['c0'] + lr['c0'])/2
            c1 = (ll['c1'] + lr['c1'])/2
            c2 = (ll['c2'] + lr['c2'])/2
            c3 = (ll['c3'] + lr['c3'])/2
            angle=0
            dist=100000
            L=29.0
            R=20.0
            for i in np.arange(-1.57,1.57,0.01) :
                temp = c0+c1*R*math.cos(i)+c2*R*math.cos(i)*R*math.cos(i)+c3*R*math.cos(i)*R*math.cos(i)*R*math.cos(i)-R*math.sin(i)
                if abs(temp)<dist :
                    dist=abs(temp)
                    angle=i
            out = {}
            #print(angle)
            if dist>10 : 
                angle = 0

            out['AdditiveSteeringWheelAngle'] = float(mpcCallback(R*math.cos(angle), R*math.sin(angle), 2*angle))
         
            out['AdditiveSteeringWheelAccel'] = 0
            out['AdditiveSteeringWheelSpeed'] = 0
            out['AdditiveSteeringWheelTorque'] = 0
            out['MultiplicativeSteeringWheelAccel'] = 1
            out['MultiplicativeSteeringWheelAngle'] = 0
            out['MultiplicativeSteeringWheelSpeed'] = 1
            out['MultiplicativeSteeringWheelTorque'] = 1
            out['TimeOfUpdate'] = data['timeOfUpdate']
            print("Steering Command : " , out['AdditiveSteeringWheelAngle'])
            output.instance.set_dictionary(out)
            output.write()






if __name__ == '__main__':    

    start()