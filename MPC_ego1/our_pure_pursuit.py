#!/usr/bin/env python
'''
This is the implementation of pure pursuit controller for path tracking
Link to reference paper: https://www.ri.cmu.edu/pub_files/2009/2
/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
Authors: Adarsh Patnaik, Anand Jhunjhunwala
'''

# import rospy
import math
import sys
import os
os.environ['OPENBLAS_NUM_THREADS'] = str(1)

import numpy as np
import json
import time
import configparser

# import graph_ltpl

from sys import path as sys_path
from os import path as os_path

file_path = os_path.dirname(os_path.realpath(__file__))
sys_path.append(file_path + "/../../../")
import rticonnextdds_connector as rti

from sys import path as sys_path
from os import path as os_path

file_path = os_path.dirname(os_path.realpath(__file__))
print(file_path)

import rticonnextdds_connector as rti


# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped
# # # from nav_msgs.msg import Path
# from prius_msgs.msg import Control

# node name: path_tracking
# Publish Topic: cmd_delta
# Subscribe Topic: base_pose_ground_truth, astroid_path

max_vel = 40 # maximum linear velocity
global steer
k = 1.0 # constant for relating look ahead distance and velocity
wheelbase = 9.9 # wheel base for the vehicle
d_lookahead = 0.1 # look ahead distance to calculate target point on path
global n
global ep_max
global ep_sum
global ep_avg
global q

print ("start")
q=0
n=0
ep_avg = 0
ep_sum = 0
ep_max = 0
start_throttle=1
start_speed=2

def callback_feedback(odom_x,odom_y,odom_z,odom_heading):
	'''
	Assigns the position of the robot to global variables from odometry and
	calculates target path point and the steering angle
	:param x_bot [float]
	:param y_bot [float]
	:param yaw [float]
	:param vel [float]
	:param data [Path]
	:param ep [float]
	:param cp [int]
	'''
	global x_bot
	global y_bot
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
	global x_p

	x_bot = odom_x
	y_bot = odom_y
	# quarternion to euler conversion
	# siny = +2.0 * (data.pose.pose.orientation.w *
	# 			   data.pose.pose.orientation.z +
	# 			   data.pose.pose.orientation.x *
	# 			   data.pose.pose.orientation.y)
	# cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
	# 					 data.pose.pose.orientation.y +
	# 					 data.pose.pose.orientation.z *
	# 					 data.pose.pose.orientation.z)
	# yaw = math.atan2(siny, cosy) # yaw in radians

	yaw = odom_heading
	# printing the odometry readings
	print ("x of car:", str(x_bot))
	print ('y of car:', str(y_bot))
	print ('angle of car:', str(yaw))
	# print ('vel of car:', str(odom_x), str(odom_y),str(odom_z))
	print ('c')

	# cross_err = Twist()
	calc_path_length(x_p)
	data1 = x_p

	distances = []
	for i in range(len(x_p)):
		a = (x_p[i][1],x_p[i][2])
		distances .append( dist(a, x_bot, y_bot))
	# print(distances)
	ep = min(distances)
	ep1 = ep

	if (ep > ep_max):
		ep_max = ep

	n = n + 1
	ep_sum = ep_sum + ep
	ep_avg = ep_sum / n


	cp = distances.index(ep)
	cp1 = cp
	# cross2 = [(x_bot - data1.poses[cp1].pose.position.x),
			#   (y_bot - data1.poses[cp1].pose.position.y)]
	# cross = [math.cos(yaw), math.sin(yaw)]
	# cross_prod = cross[0] * cross2[1] - cross[1] * cross2[0]
	# if (cross_prod > 0):
	# 	ep1 = -ep1

	print ('ep_sum: ' , str(ep_sum))
	print ('ep_avg: ' , str(ep_avg))
	# cross_err.linear.x = ep1
	# cross_err.angular.x = ep_max
	# cross_err.angular.y = ep_avg

	   
	print ('old index:', str(cp))
	# calculate index of target point on path
	# cmd = Twist()
	# cmd = #######
	# cmd1 = Twist()
	# prius_vel = Control()
	L = 0
	Lf = 20.0

	while Lf > L and (cp + 1) < len(x_p):
		dx = data1[cp + 1][1] - \
			x_bot
		dy = data1[cp + 1][2] - \
			y_bot
		L = math.sqrt(dx ** 2 + dy ** 2)
		# L= math.sqrt
		cp = cp + 1
	print (len(x_p))
	print ('new index is:', str(cp))

	goal_point = [x_p[cp][1], x_p[cp][2]]

	print ('current goal is:', str(goal_point))
	error = [goal_point[0] - x_bot, goal_point[1] - y_bot]
	print (error)
	steer_angle = pure_pursuit(goal_point,L)

	# siny = +2.0 * (x_p.poses[cp].pose.orientation.w *
	# 			   x_p.poses[cp].pose.orientation.z +
	# 			   x_p.poses[cp].pose.orientation.x *
	# 			   x_p.poses[cp].pose.orientation.y)

	# cosy = +1.0 - 2.0 * (x_p.poses[cp].pose.orientation.y *
	# 					 x_p.poses[cp].pose.orientation.y +
	# 					 x_p.poses[cp].pose.orientation.z *
	# 					 x_p.poses[cp].pose.orientation.z)

	# steer_path = math.atan2(siny, cosy)
	# steer_err = (yaw - steer_path)
	# cross_err.linear.y =  (-1)*(yaw - steer_path)


	print ("steer_angle :", str(steer_angle * 180 / math.pi))
	cmd = min(30, max(-30, steer_angle * 180 / math.pi))
	# cmd.linear.y = math.sqrt(error[0]**2 + error[1]**2)
	print ('omega:', str(cmd))
	# cross_err.linear.z = path_length[cp]

	return cmd , cp
	# pub2.publish(cross_err)
	
	print ("cmd published")

	# print (ep)
	# print (x_p.poses[cp].pose.orientation)


def dist(a, x, y):
	'''
	Calculates distance between two points.
	:param a [float]
	:param x [float]
	:param y [float]
	'''
	# calculate distance
	return (((a[0] - x)**2) + ((a[1] - y)**2))**0.5

# def path_length_distance(a,b):
# 	return (((a[0] - b[0])**2) + ((a[1] - b[1])**2))**0.5

def calc_path_length(data):
	global path_length
	path_length = []

	for i in range(len(data)):
		if i == 0:
			path_length.append(0)

		else:
			path_length.append(path_length[i-1] + dist((data[i][1],data[i][2]), data[i-1][1] , data[i-1][2]))


def callback_path(data):

	global ep # min distance
	global cp # index of closest point
	global ep_max
	global ep_sum
	global ep_avg
	global n
	global cp1
	global path_length
	global x_p

	x_p = data


def pure_pursuit(goal_point,Lf):
	'''
	Calculates the steering angle required for path tracking
	:params goal_point [float,float] goal point coordinates
	:params Delta [float] steering angle in radians 
	'''
	tx = goal_point[0]
	ty = goal_point[1]
	print ('yaw:', str(yaw))
	# measuring the slope of path point
	print ('slope:', str(math.atan2(ty - y_bot, tx - x_bot)))
	# measuring heading angle 
	alpha = math.atan2(ty - y_bot, tx - x_bot) - yaw
	print ('alpha:', str(alpha))
	# Lf = k * max_vel + d_lookahead
	# measuring the steering angle using pure pursuit controller
	# Delta = math.atan2(2.0 * wheelbase * math.sin(alpha) / Lf, 1)
	L=29.0
	# R=20.0
	Delta = (2*L*math.sin(alpha))/Lf
	print('Lf:',Lf)
	print ('Delta:', str(Delta))
	return Delta


def start():
	# global pub1
	# global pub2
	# rospy.init_node('path_tracking', anonymous=True)
	# pub2 = rospy.Publisher('cross_track_error', Twist, queue_size=100)
	# pub1 = rospy.Publisher('cmd_delta', Twist, queue_size=100)
	# rospy.Subscriber("frenet_path", Path, callback_path)
	# rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
	# rospy.spin()
	with rti.open_connector(
			config_name="MyParticipantLibrary::MyPurePursuitParticipant",
			url=file_path + "/../Sensors_ego1.xml") as connector:
	
		input = connector.get_input("pathSubscriber::pathSub")
		input_odom=connector.get_input("StateSubscriber::stateReader")
		output = connector.get_output("steeringPublisher::steeringPub")
		output_speed = connector.get_output("SpeedPublisher::speedPub")
		wait_topic = connector.get_input("simWaitSub::simWaitReader")
		done_topic = connector.get_output("simDonePub::simDoneWriter")
		
		# controls = connector.get_output("controlPublisher::controlPub")

		# Read data from the input, transform it and write it into the output
		print("Waiting for data...")
		
		#Initialise
		curr_steering = 0
		curr_speed = 0
		target_speed = 0
		aggregate = 0
		# nr_dist = 0
		# all_vehicles = np.ones((max_no_of_vehicles,4))*10000
		while True:
			wait_topic.wait()
			wait_topic.take()
			print("in while loop")
			wait_msg = []
			
			for sample in wait_topic.samples.valid_data_iter:
				data = sample.get_dictionary()
				wait_msg = data
			input_odom.wait() # Wait for data in the input
			input_odom.take()
			# target_point=
			print("received odom data")
			for sample in input_odom.samples.valid_data_iter:
				data = sample.get_dictionary()
				break
				# vel_est = data['cdgSpeed_x']
				# pos_est = [data['cdgPos_x'],data['cdgPos_y']]
				# pos_est = np.asarray(pos_est)
			
			input.wait() 
			# Wait for data in the input
			input.take()
			print("received path data")
			for sample1 in input.samples.valid_data_iter:
				data1 = sample1.get_dictionary()
				data1 = data1['param']
				# callback_path(data1)
				break
			data1=np.array(data1)
			data1 =np.reshape(data1, (-1, 7)).tolist()
			# print(data1)
			callback_path(data1)
			vx = data['cdgSpeed_x']
			vy = data['cdgSpeed_y']
			vz = data['cdgSpeed_z'] 
			curr_speed = math.sqrt(vx*vx+vy*vy+vz*vz)
			print(F'vx->{vx}  vy->{vy} vz->{vz} velocity of car {curr_speed}')
			steer_angle,goal_idx = callback_feedback(data['cdgPos_x'],data['cdgPos_y'],data['cdgPos_z'],data['cdgPos_heading'])
			out = {}
			out['AdditiveSteeringWheelAngle'] = steer_angle*math.pi/180
			out['AdditiveSteeringWheelAccel'] = 0
			out['AdditiveSteeringWheelSpeed'] = 0
			out['AdditiveSteeringWheelTorque'] = 0
			out['MultiplicativeSteeringWheelAccel'] = 1
			out['MultiplicativeSteeringWheelAngle'] = 0
			out['MultiplicativeSteeringWheelSpeed'] = 1
			out['MultiplicativeSteeringWheelTorque'] = 1
			out['TimeOfUpdate'] = data['TimeOfUpdate']
			print("XXXXX")
			print("")
			print("")
			print("")
			print(data['TimeOfUpdate'])
			output.instance.set_dictionary(out)
			output.write()
			print("XXXXX")
			
			# out_controls['speedsArray'] = # list of speeds
			# out_controls['steeringArray'] = # list of yaws

			# input_speed.wait() # Wait for data in the input
			# input_speed.take()
			# print("6")
			# for sample in input_speed.samples.valid_data_iter:
			#     st11 = time.time()
			#     data = sample.get_dictionary()
			target_speed = data1[goal_idx][5]

			
			print(f"Current Speed :{curr_speed}, Target Speed: {target_speed} " )
			out_speed = {}
			kp=1
			ki=0
			throtle = kp*(target_speed-curr_speed)+ki*aggregate
			
			if curr_speed < start_speed:
				throtle = start_throttle
			print("Pedal : ", throtle)
			aggregate = aggregate + (target_speed-curr_speed)
			out_speed['AcceleratorAdditive'] = max(0,throtle)
			out_speed['AcceleratorMultiplicative'] = 0
			out_speed['BrakeAdditive'] = -min(0,throtle)
			out_speed['BrakeMultiplicative'] = 0
			out_speed['ClutchAdditive'] = 0
			out_speed['ClutchMultiplicative'] = 0
			out_speed['GearboxAutoMode'] = 1
			out_speed['GearboxTakeOver'] = 0
			out_speed['IsRatioLimit'] = 0
			out_speed['MaxRatio'] = 1000
			out_speed['MinRatio'] = 1
			out_speed['ParkingBrakeAdditive'] = 0
			out_speed['ParkingBrakeMultiplicative'] = 0
			out_speed['ShiftDown'] = 0
			out_speed['ShiftUp'] = 0
			out_speed['WantedGear'] = 1
			
			out_speed['TimeOfUpdate'] = data['TimeOfUpdate']
			output_speed.instance.set_dictionary(out_speed)
			output_speed.write()
			done_topic.instance.set_dictionary(wait_msg)
			done_topic.write()
			print("message written")
			# en = time.time()
			# print(time.time())
			print("XXXXX")
			# print("")
			# print("")
			# print("")
			print(out_speed['TimeOfUpdate'])
			print(time.time())
			# et11 = time.time()
			# print("time 11 "+str(et11-st11))
		



		

		


if __name__ == '__main__':
	start()