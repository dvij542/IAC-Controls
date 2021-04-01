# -*- coding: utf-8 -*-
#!/usr/bin/env python
from casadi import *
import numpy as np
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
#from ._tf2 import *

#from tf.transformations import euler_from_quaternion, quaternion_from_euler


global x_bot
global y_bot
global control_count
control_count=0
has_start=True

T = .08	 # Time horizon
N = 5# number of control intervals
###########   states    ####################3
x=SX.sym('x')
y=SX.sym('y')
theta=SX.sym('theta')
v=SX.sym('v')
states=vertcat(x,y,theta,v)

delta=SX.sym('delta')
a = SX.sym('a')

controls=vertcat(a,delta)
EPSILON = 1e-5


L=0.25



rhs=[
        v*cos(theta+((atan(tan(delta))/2)+EPSILON)),
        v*sin(theta+((atan(tan(delta))/2)+EPSILON )),
        2*v*sin((atan(tan(delta))/2)+EPSILON )/L,
        a


    ]                                                                                   
rhs=vertcat(*rhs)

# dae = {'x':states, 'p':controls, 'ode':rhs}
# opts = {'tf':T/N}
# I = integrator('I', 'rk', dae, opts)
# print(I)
f=Function('f',[states,controls],[rhs])
#print(f)
n_states=4  
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

# print(ff)
obj=0

Q=SX([[100,0,0,0],
    [0,100,0,0],
    [0,0,0.1,0],
    [0,0,0,1]
    ])

R=SX([[.5,0],
    [0,1]])

for k in range(0,N,1):
    st=X[:,k]
    con=U[:,k]
    obj=obj+(((st- P[n_states:8]).T)@Q)@(st- P[n_states:8])+con.T@R@con

for k in range(0,N-1,1):
	prev_con=U[:,k]
	next_con=U[:,k+1]
	obj=obj+(prev_con- next_con).T@R@(prev_con- next_con)

OPT_variables = reshape(U,2*N,1)
#print(X[1,k])
for k in range (0,N+1,1): 
    g = X[0,k]   
    g = X[1,k]  
    #g = X[2,k]
   

#print(atan(-1))
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
v=2

lbx=np.zeros(2*N)

ubx=np.zeros(2*N)
for k in range (0,2*N,2): 
    lbx[k]=-0.2 
    ubx[k]=v
for k in range (1,(2*N)-1,2): 
    lbx[k]=-math.pi
    ubx[k]=math.pi



# lbx=math.pi
# ubx=math.pi



u0=np.random.rand(N,2)
x0=reshape(u0,2*N,1)






def dist(a, x, y):
    '''
    Calculates distance between two points.
    :param a [float]
    :param x [float]
    :param y [float]
    '''
    # calculate distance
    return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5

def path_length_distance(a,b):
    return (((a.pose.position.x - b.pose.position.x)**2) + ((a.pose.position.y - b.pose.position.y)**2))**0.5

def calc_path_length(data):
    global path_length
    path_length = []

    for i in range(len(data.poses)):
        if i == 0:
            path_length.append(0)

        else:
            path_length.append(path_length[i-1] + path_length_distance(data.poses[i], data.poses[i-1]))


def posCallback(posedata):
    global data
    data=posedata

def SpeedCallback(speedData):
	global car_speed
	car_speed = sqrt(speedData.twist.twist.linear.x**2 + speedData.twist.twist.linear.y**2)
    


def pathCallback(pathdata):

    global x_p
    global has_start
    has_start=True
    x_p = pathdata
    mpcCallback()
    

    
def calculate_goal_point_index():

     


    x_bot1 = data.pose.position.x
    y_bot1 = data.pose.position.y
    calc_path_length(x_p)
    distances = []    
    for i in range(len(x_p.poses)):
        a = x_p.poses[i]
        distances += [dist(a, x_bot1, y_bot1)]
    ep = min(distances)
    total_index=len(x_p.poses)
    cp = distances.index(ep)
    calculated_goal_point_index = cp+50
    if(calculated_goal_point_index>len(x_p.poses)):
        calculated_goal_point_index=10

    # if(3910<=calculated_goal_point_index<=3930):
    #     calculated_goal_point_index=calculated_goal_point_index+50

    return calculated_goal_point_index

        


    

def mpcCallback():
   
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

    goal_point_index=calculate_goal_point_index()

    goal_point = [x_p.poses[goal_point_index].pose.position.x,
                  x_p.poses[goal_point_index].pose.position.y]

    siny = +2.0 * (x_p.poses[goal_point_index].pose.orientation.w *
                   x_p.poses[goal_point_index].pose.orientation.z +
                   x_p.poses[goal_point_index].pose.orientation.x *
                   x_p.poses[goal_point_index].pose.orientation.y)

    cosy = +1.0 - 2.0 * (x_p.poses[goal_point_index].pose.orientation.y *
                         x_p.poses[goal_point_index].pose.orientation.y +
                         x_p.poses[goal_point_index].pose.orientation.z *
                         x_p.poses[goal_point_index].pose.orientation.z)
    goal_point_yaw=math.atan2(siny, cosy)
    x_bot_init = data.pose.position.x
    y_bot_init = data.pose.position.y
    siny = +2.0 * (data.pose.orientation.w *
                       data.pose.orientation.z +
                       data.pose.orientation.x *
                       data.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.orientation.y *
                             data.pose.orientation.y +
                             data.pose.orientation.z *
                             data.pose.orientation.z)
    yaw_car_init = math.atan2(siny,cosy) # yaw in radians

    current_pose=[x_bot_init,y_bot_init,yaw_car_init,car_speed]
    print('*******  current pose:',current_pose)

    #current_pose=[0,0,0]
    xs=[x_p.poses[goal_point_index].pose.position.x,x_p.poses[goal_point_index].pose.position.y,goal_point_yaw,1]


    
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

    print('**********goal point index',goal_point_index)

    #while(((np.linalg.norm(np.subtract(x_next[0],xs[0])))+(np.linalg.norm(np.subtract(x_next[1],xs[1]))))>1e-1):
    count=0
    while(sqrt((current_pose[0] - xs[0])**2 + (current_pose[1] - xs[1])**2+(current_pose[2]-xs[2])**2)>.110):
    #while(sqrt(((x_bot - xs[0])**2 + (y_bot - xs[1])**2)+(yaw_car-xs[2])**2)>.1):

        x_bot = data.pose.position.x
        y_bot = data.pose.position.y
        # print("x_bot",x_bot)
        # print('y_bot',y_bot)
        # print("total index       ",len(x_p.poses))
        
        siny = +2.0 * (data.pose.orientation.w *
                       data.pose.orientation.z +
                       data.pose.orientation.x *
                       data.pose.orientation.y)
        cosy = +1.0 - 2.0 * (data.pose.orientation.y *
                             data.pose.orientation.y +
                             data.pose.orientation.z *
                             data.pose.orientation.z)
        yaw_car = math.atan2(siny, cosy) # yaw in radians




        current_pose=[x_bot,y_bot,yaw_car,car_speed]
        #print('*******  current pose:',current_pose)
        print('###############################   Inside while loop                   ##########################################################')
        print('###############################   Inside while loop                   ##########################################################')
        #print('###############################   Inside while loop                   ##########################################################')
        e=sqrt(((current_pose[0] - xs[0])**2 + (current_pose[1] - xs[1])**2)+(current_pose[2]-xs[2])**2)
        #print('************************************************************')
        print("error       :",e)
        
        so=solver(x0=x0,p=p,lbx=lbx,ubx=ubx) 
        x=so['x']
        u = reshape(x.T,2,N).T        
        st = current_pose
        print('st     ',st)
        
        #print("***********     init   state",st)
        con = u[0,:].T
        if(abs(con[0]+con[1])<1e-2):
            break

        
        # if((abs(con[0])<1e-3) and (abs(con[1])>1e-1) ):
        #     con[0]=1e-1

            


            #control_count+=1
        print('********controls',u)
        f_value = f(st,con)
        ff_value=ff(u.T,p).T
       	first_predicted_state=ff_value[1,:]

       	print('first_predicted_state',first_predicted_state)
       	
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()
        ctrlmsg.drive.jerk = 1
        ctrlmsg.drive.steering_angle = con[1]
        pub1.publish(ctrlmsg)
        print('current_pose',current_pose)
       
        
        print("ff_value   ",ff_value)
        global predicted_trajectory
        predicted_trajectory=Path()
        predicted_trajectory.header.frame_id='map'
        
        print()
        for i in range(0,N,1):
        	#b.header.frame_id='map'
        	pose=PoseStamped()
        	pose.pose.position.x=ff_value[i,0]
        	pose.pose.position.y=ff_value[i,1]
        	#print("pose  ",a)
        	yaw = 0.0
        	#q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        	pose.pose.orientation.x=0
        	pose.pose.orientation.y=0
        	pose.pose.orientation.z=0
        	pose.pose.orientation.w=1
        	predicted_trajectory.poses.append(pose)	
			
        	
        
        pub3.publish(predicted_trajectory)
        print('*******  current pose:',current_pose)
        print('********** xs',xs)
        print('x_coordinate',ff_value[0,0])
        #print('goal point index',goal_point_index)
        #p=[float(x_next[0]),float(x_next[1]),float(x_next[2]),xs[0],xs[1],xs[2]]
        p=[float(x_bot),float(y_bot),float(yaw_car),car_speed,xs[0],xs[1],xs[2],0,1]
        #print("total index       ",len(x_p.poses))
        #print('goal point distances  ',distances[goal_point_index])
        #print('x bot   ',data.pose.position.x)
        #x_init=current_pose
        #print(len(x_p.poses))
       # print('No of time negative control input  ',control_count)
        print('************************************************************')
        print('###############################   Inside while loop                   ##########################################################')
        print('###############################   Inside while loop                   ##########################################################')
        #print('###############################   Inside while loop                   ##########################################################')
       
        a=PointStamped()
        a.header.stamp=rospy.Time.now()
        a.header.frame_id='/map'
        a.point.x=xs[0]
        a.point.y=xs[1]
        a.point.z=0
        pub2.publish(a)
        #print('ggggggggg',g)
       
        print('lbg',lbg)
        
        
        print('called called')



def start():
    global pub1
    global pub2
    global pub3
    rospy.init_node('path_tracking', anonymous=True)
    
    pub2 = rospy.Publisher('goal_point', PointStamped, queue_size=1)
    pub1 = rospy.Publisher("/mux/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
    pub3 = rospy.Publisher('/predicted_trajectory', Path,queue_size=1000)
    # rospy.Subscriber("/move_base/TebLocalPlannerROS/teb_poses", PoseArray, callback_path)
    rospy.Subscriber("car_pose", PoseStamped, posCallback,queue_size=1)
    rospy.Subscriber("/vesc/odom",Odometry,SpeedCallback,queue_size=1)
    rospy.Subscriber("astroid_path", Path, pathCallback,queue_size=1)
    rospy.spin()
    r=rospy.Rate(1000)
    r.sleep()


if __name__ == '__main__':    
    start()