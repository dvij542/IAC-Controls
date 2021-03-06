###############################################################################
# (c) Copyright, Real-Time Innovations, 2019.  All rights reserved.           #
# No duplications, whole or partial, manual or electronic, may be made        #
# without express written permission.  Any such copies, or revisions thereof, #
# must display this notice unaltered.                                         #
# This code contains trade secrets of Real-Time Innovations, Inc.             #
###############################################################################

"""Reads Squares, transforms them and writes them as Circles."""

from sys import path as sys_path
from os import path as os_path

file_path = os_path.dirname(os_path.realpath(__file__))
sys_path.append(file_path + "/../../../")
import rticonnextdds_connector as rti
import numpy as np
import math

curr_index = 0
last_index = -1
kp = 1
curr_steerings = np.zeros(20)
curr_speeds = np.zeros(20)
with rti.open_connector(
        config_name="MyParticipantLibrary::SpeedParticipant",
        url=file_path + "/../ShapeExample.xml") as connector:

    input = connector.get_input("StateSubscriber::stateReader")
    output = connector.get_output("SpeedPublisher::speedPub")
    input_controls = connector.get_input("controlSubscriber::controlSub")
    output_steering = connector.get_output("steeringPublisher::steeringPub")
    # Read data from the input, transform it and write it into the output
    print("Waiting for data...")
    input_controls.wait()
    input.wait()
    while True:
        input_controls.take()
        for sample in input_controls.samples.valid_data_iter:
            data = sample.get_dictionary()
            #print(data)
            curr_steerings = data['steeringArray']
            curr_speeds = data['speedsArray']
            curr_index = curr_index - last_index - 1
            last_index = curr_index
            break
        input.wait() # Wait for data in the input
        input.take()
        if curr_index > 15 :
            curr_index = 15
        for sample in input.samples.valid_data_iter:
            data = sample.get_dictionary()
            vx = data['cdgSpeed_x']
            vy = data['cdgSpeed_y']
            vz = data['cdgSpeed_z']
            curr_speed = math.sqrt(vx*vx+vy*vy+vz*vz);
            out = {}
            out_steering = {}
            target_speed=curr_speeds[curr_index]
            #print(target_speed-curr_speed)
            out['AcceleratorAdditive'] = max(0,kp*(target_speed-curr_speed))
            out['AcceleratorMultiplicative'] = 0
            out['BrakeAdditive'] = -min(0,kp*(target_speed-curr_speed))
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
            output.instance.set_dictionary(out)
            output.write()
            print("Current Index:", curr_index)
            print("Target Speed:", target_speed)
            curr_steering = curr_steerings[curr_index]
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
            output_steering.instance.set_dictionary(out_steering)
            output_steering.write()
            print("Time: ", data['TimeOfUpdate'])
            break
        curr_index += 1