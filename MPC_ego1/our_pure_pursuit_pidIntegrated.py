#!/usr/bin/env python
'''
This is the implementation of pure pursuit controller for path tracking
Link to reference paper: https://www.ri.cmu.edu/pub_files/2009/2
/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
Authors: Adarsh Patnaik, Anand Jhunjhunwala
'''

# import rospy
import rticonnextdds_connector as rti
from os import path as os_path
from sys import path as sys_path
import configparser
import time
import json
import numpy as np
import math
import sys
import os
os.environ['OPENBLAS_NUM_THREADS'] = str(1)

file_path = os_path.dirname(os_path.realpath(__file__))
sys_path.append(file_path + "/../../../")


file_path = os_path.dirname(os_path.realpath(__file__))
print(file_path)


max_vel = 40  # maximum linear velocity
global steer
k = 1.0  # constant for relating look ahead distance and velocity
wheelbase = 9.9  # wheel base for the vehicle
d_lookahead = 0.1  # look ahead distance to calculate target point on path
global n
global ep_max
global ep_sum
global ep_avg
global q

print("start")
q = 0
n = 0
ep_avg = 0
ep_sum = 0
ep_max = 0
start_throttle = 1
start_speed = 2
brake_threshold = 20 # threshold for braking
kp = 8.0 # proprtional gain
ki = 2.0 # integral gain
kd = 0.2 # derivative gain

yp = 20.0 # kp gain
yi = 0.5 # ki gain
yd = 0.1 # kd gain
prev_error=0
error_sum = 0

acc_thershold = 0 #threshold for acceleration

def callback_feedback(odom_x, odom_y, odom_z, odom_heading):
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
	global ep  # min distance
	global cp  # index of closest point
	global ep_max
	global ep_sum
	global ep_avg
	global n
	global cp1
	global path_length
	global x_p

	x_bot = odom_x
	y_bot = odom_y

	yaw = odom_heading
	# printing the odometry readings
	print("x of car:", str(x_bot))
	print('y of car:', str(y_bot))
	print('angle of car:', str(yaw))
	# print ('vel of car:', str(odom_x), str(odom_y),str(odom_z))
	print('c')

	calc_path_length(x_p)
	data1 = x_p

	distances = []
	for i in range(len(x_p)):
		a = (x_p[i][1], x_p[i][2])
		distances .append(dist(a, x_bot, y_bot))

	ep = min(distances)
	ep1 = ep

	if (ep > ep_max):
		ep_max = ep

	n = n + 1
	ep_sum = ep_sum + ep
	ep_avg = ep_sum / n

	cp = distances.index(ep)
	cp1 = cp

	print('ep_sum: ', str(ep_sum))
	print('ep_avg: ', str(ep_avg))

	print('old index:', str(cp))
	L = 0
	Lf = 20.0

	while Lf > L and (cp + 1) < len(x_p):
		dx = data1[cp + 1][1] - \
			x_bot
		dy = data1[cp + 1][2] - \
			y_bot
		L = math.sqrt(dx ** 2 + dy ** 2)
		cp = cp + 1
	print(len(x_p))
	print('new index is:', str(cp))

	goal_point = [x_p[cp][1], x_p[cp][2]]

	print('current goal is:', str(goal_point))
	error = [goal_point[0] - x_bot, goal_point[1] - y_bot]
	print(error)
	steer_angle = pure_pursuit(goal_point, L)

	print("steer_angle :", str(steer_angle * 180 / math.pi))
	cmd = min(30, max(-30, steer_angle * 180 / math.pi))
	print('omega:', str(cmd))

	return cmd

	# print ("cmd published")


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
			path_length.append(
				path_length[i-1] + dist((data[i][1], data[i][2]), data[i-1][1], data[i-1][2]))


def callback_path(data):

	global ep  # min distance
	global cp  # index of closest point
	global ep_max
	global ep_sum
	global ep_avg
	global n
	global cp1
	global path_length
	global x_p

	x_p = data


def pure_pursuit(goal_point, Lf):
	'''
	Calculates the steering angle required for path tracking
	:params goal_point [float,float] goal point coordinates
	:params Delta [float] steering angle in radians 
	'''
	tx = goal_point[0]
	ty = goal_point[1]
	print('yaw:', str(yaw))
	# measuring the slope of path point
	print('slope:', str(math.atan2(ty - y_bot, tx - x_bot)))
	# measuring heading angle
	alpha = math.atan2(ty - y_bot, tx - x_bot) - yaw
	print('alpha:', str(alpha))
	# Lf = k * max_vel + d_lookahead
	# measuring the steering angle using pure pursuit controller
	# Delta = math.atan2(2.0 * wheelbase * math.sin(alpha) / Lf, 1)
	L = 29.0
	# R=20.0
	Delta = (2*L*math.sin(alpha))/Lf
	print('Lf:', Lf)
	print('Delta:', str(Delta))
	return Delta


def start():
	with rti.open_connector(
			config_name="MyParticipantLibrary::MyPurePursuitParticipant",
			url=file_path + "/../Sensors_ego1.xml") as connector:

		input = connector.get_input("pathSubscriber::pathSub")
		input_odom = connector.get_input("StateSubscriber::stateReader")
		output = connector.get_output("steeringPublisher::steeringPub")
		output_speed = connector.get_output("SpeedPublisher::speedPub")
		wait_topic = connector.get_input("simWaitSub::simWaitReader")
		done_topic = connector.get_output("simDonePub::simDoneWriter")

		# controls = connector.get_output("controlPublisher::controlPub")

		# Read data from the input, transform it and write it into the output
		print("Waiting for data...")

		# Initialise
		# curr_steering = 0
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
			input_odom.wait()  # Wait for data in the input
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
			data1 = np.array(data1)
			data1 = np.reshape(data1, (-1, 7)).tolist()
			# print(data1)
			callback_path(data1)
			vx = data['cdgSpeed_x']
			vy = data['cdgSpeed_y']
			vz = data['cdgSpeed_z']
			curr_speed = math.sqrt(vx*vx+vy*vy+vz*vz)
			print(F'vx->{vx}  vy->{vy} vz->{vz} velocity of car {curr_speed}')
			steer_angle = callback_feedback(
				data['cdgPos_x'], data['cdgPos_y'], data['cdgPos_z'], data['cdgPos_heading'])
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
			


			###############################
			target_speed = 22.22 #-> get from data1
			###############################



			# print("Current Speed : ", curr_speed)
			out_speed = {}
			kp1 = 1
			ki1 = 0
			throtle = kp1*(target_speed-curr_speed)+ki1*aggregate
			global start_throttle
			global start_speed
			if curr_speed < start_speed:
				throtle = start_throttle
			print("Pedal : ", throtle)
			aggregate = aggregate + (target_speed-curr_speed)
			
########################################PID MIT VEL CONTROL##########################################
			tar_vel = target_speed
			# tar_delta = steer_angle 
			active_vel = curr_speed

					# plot = Twist()
					# output = Twist()
			output_linear_x=float()
			error = tar_vel - active_vel
			global error_sum
			global prev_error
			error_sum += error
			error_diff = error - prev_error
			prev_error = error
			if error == 0:
				if tar_vel == 0:
					output_linear_x = 0
				else:
					output_linear_x = output_linear_x - 5      ############ why?????????????

			# # updating kp, ki, kd using MIT rule
			global kp
			global ki
			global kd
			global yp
			global yi
			global yd
			kp = kp + yp * error * error
			ki = ki + yi * error * error_sum
			kd = kd + yd * error * error_diff

			print(f"kp is : {kp}")
			print(f"ki is : {ki}")
			print(f"kd is : {kd}")

			global brake_threshold
			# # PID on velocity with updated parameters
			if error > 0.01:
				output_linear_x = (kp * error + ki * error_sum + kd * error_diff)
			if error < -0.01:
				output_linear_x = ((kp * error + ki * error_sum + kd * error_diff) -
					brake_threshold)

			# thresholding the forward velocity
			if output_linear_x > 100:
				output_linear_x = 100
			if output_linear_x < -100:
				output_linear_x = -100
			# thresholding the angle
			# output_angular_z = min(30.0, max(-30.0, tar_delta))

			
			print(f"linear velocity : {active_vel}")
			print(f"target linear velocity : {tar_vel}")
			print(f"delta : {tar_vel-active_vel}")
					
			if(output_linear_x > 0):
					prius_vel_throttle = output_linear_x / 100
					prius_vel_brake = 0
					print ("acc")
					print (prius_vel_throttle)

			if(output_linear_x < 0):
					prius_vel_brake = -output_linear_x / 100
					prius_vel_throttle = 0
					print ("brake")
					print (prius_vel_brake)

			# prius_vel_steer = output_angular_z / 30
#################################PID MIT ENDS####################################
			
			
			out_speed['AcceleratorAdditive'] = prius_vel_throttle
			out_speed['AcceleratorMultiplicative'] = 0
			out_speed['BrakeAdditive'] = prius_vel_brake
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
			print("XXXXX")
			print(out_speed['TimeOfUpdate'])
			print(time.time())

if __name__ == '__main__':
	start()
