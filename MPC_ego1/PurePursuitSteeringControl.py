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
import time

with rti.open_connector(
        config_name="MyParticipantLibrary::MyParticipant",
        url=file_path + "/../ShapeExample.xml") as connector:

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
            print(time.time())
            for i in np.arange(-1.57,1.57,0.01) :
                temp = c0+c1*R*math.cos(i)+c2*R*math.cos(i)*R*math.cos(i)+c3*R*math.cos(i)*R*math.cos(i)*R*math.cos(i)-R*math.sin(i)
                if abs(temp)<dist :
                    dist=abs(temp)
                    angle=i
            out = {}
            print(data['timeOfUpdate'])
            if dist>10 : 
                angle = 0
            out['AdditiveSteeringWheelAngle'] = (2*L*math.sin(angle))/R
         
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
