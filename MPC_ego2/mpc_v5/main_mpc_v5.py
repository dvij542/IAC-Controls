# -*- coding: utf-8 -*-
from sys import path as sys_path
from os import path as os_path
from casadi import *
import math
import numpy as np
import time
import rticonnextdds_connector as rti
print("Started")
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
print("Started1")
import util_funcs as utils
import params as pars
import cost_func as cf
print("Started2")
global x_bot
global y_bot
global control_count
control_count=0
trajectory_to_follow = []
file_path = os_path.dirname(os_path.realpath(__file__))
sys_path.append(file_path + "/../../../")
has_start=True

predicted_x = 0
predicted_y = 0
predicted_theta = 0
predicted_v = 0
control_sample = np.zeros((2,pars.N))
no_of_vehicles = 0

def mpcCallback(no_of_vehicles, trajectory_to_follow, speeds_to_follow, curr_pos, angle_heading, curve, curve_l, curve_r, steering, speed, goaltheta, all_vehicles, roadwidth,opp_vehicle_detected,opp_vehicle_detected_state):
    x_bot = 0
    y_bot = 0
    ####### Special regions ############
    if curr_pos[1]<-1174 and curr_pos[1]>-1452 and curr_pos[0]<-170:
        print("Special region")
        x_bot = 12
    yaw_car = 0 # yaw in radians
    current_pose=[x_bot,y_bot,yaw_car]
    current_control = [speed, steering]
    if pars.file_path_follow!=None :
        k = trajectory_to_follow.shape[0]
        mini = 0
        minval = 10000
        #print(curr_pos)
        #print(trajectory_to_follow.shape)
        for i in range(k):
            if utils.dist1(trajectory_to_follow[i,0], trajectory_to_follow[i,1], curr_pos[0], curr_pos[1]) < minval:
                minval = utils.dist1(trajectory_to_follow[i,0], trajectory_to_follow[i,1], curr_pos[0], curr_pos[1])
                mini = i
        points = [trajectory_to_follow[mini,:]-curr_pos,trajectory_to_follow[(mini+1)%k,:]-curr_pos,trajectory_to_follow[(mini+2)%k,:]-curr_pos,trajectory_to_follow[(mini+3)%k,:]-curr_pos]
        points = np.array(points)
        tr_matrix = np.array([[cos(angle_heading),-sin(angle_heading)],[sin(angle_heading),cos(angle_heading)]])
        points = np.matmul(points,tr_matrix)
        #print(points)
        M = np.array([points[:,0]**0,points[:,0] , points[:,0]**2, points[:,0]**3]).T
        current_pose[1] = speeds_to_follow[mini]  # Set start speed of the path to be followed
        current_pose[2] = speeds_to_follow[(mini+3)%k] # Set end speed of the path to be followed
        C = np.matmul(np.linalg.inv(M),points[:,1:])
        curve = [C[0],C[1],C[2],C[3]]
        print(curve)
        print(current_pose)
    current_pose[0]= no_of_vehicles
    p=current_pose+curve+current_control
    for i in range(pars.max_no_of_vehicles) : 
        p = p+all_vehicles[i].tolist()
    p = p+curve_l+curve_r
    mindist = 10000
    minindex = 0
    xc = x_bot
    yc = y_bot
    for i in range(pars.N):
        k=0
        for vehicle in all_vehicles :
            xd = vehicle[0]+vehicle[2]*i*pars.T
            yd = vehicle[1]+vehicle[3]*i*pars.T
            dist = ((yc-yd)**2 + (xc-xd)**2)**(1/2)
            if dist<mindist :
                mindist = dist
                minx = xd
                miny = yd
                minindex = k
            k = k+1
        xc+=speed*cos(atan(curve[1]+2*curve[2]*xc+3*curve[3]*xc**2))
        yc+=speed*sin(atan(curve[1]+2*curve[2]*xc+3*curve[3]*xc**2))

    print("min distance is ", mindist)
    if mindist < pars.dist_threshold :
        # Path from left
        vehicle = all_vehicles[minindex]
        opp_target_id = opp_vehicle_detected[minindex]
        print("Opp vehicle id ",str(opp_target_id))
        if opp_vehicle_detected_state[opp_target_id] != 0:
            prev_path = opp_vehicle_detected_state[opp_target_id] # prev_path = 1 for right, 2 for left
            print("continuous vehicle detected with prefered path ",str(prev_path))
        else:
            opp_vehicle_detected_state[:]= 0
            print("new vehicle detected ")
        xc=0
        yc=0
        theta = 0
        linelx = []
        linely = []
        for i in range(pars.N):
            if vehicle[3] > 0 : 
                xd = vehicle[0]+vehicle[2]*(pars.N)*(pars.T)-xc
                yd = vehicle[1]+vehicle[3]*(pars.N)*(pars.T)+1-yc
            else :
                xd = minx - xc
                yd = miny - yc + 1
            xr = xd*cos(theta) + yd*sin(theta)
            yr = -xd*sin(theta) + yd*cos(theta)
            control_sample[0,i] = 0
            Rd = (xr**2 + yr**2)/(2*yr)
            control_sample[1,i] = 58*yr/(xr**2 + yr**2)
            dx = Rd*sin((speed*pars.T)/Rd)
            dy = Rd-Rd*cos((speed*pars.T)/Rd)
            xc+=dx*cos(theta)-dy*sin(theta)
            yc+=dx*sin(theta)+dy*cos(theta)
            theta+=(speed*pars.T)/Rd
            linelx.append(xc)
            linely.append(yc)

        plt.plot(linelx,linely,'ro-')
        xl=reshape(control_sample.copy(),2*pars.N,1)
        sl=cf.solver(x0=xl,p=p,lbx=cf.lbx,ubx=cf.ubx,lbg=cf.lbg,ubg=cf.ubg)
        costl = sl['f']
        xc=0
        yc=0
        theta = 0
        linerx = []
        linery = []
        for i in range(pars.N):
            if vehicle[3] < 0 : 
                xd = vehicle[0]+vehicle[2]*(pars.N)*(pars.T)-xc
                yd = vehicle[1]+vehicle[3]*(pars.N)*(pars.T)-1-yc
            else :
                xd = minx - xc
                yd = miny - yc - 1
            xr = xd*cos(theta) + yd*sin(theta)
            yr = -xd*sin(theta) + yd*cos(theta)
            control_sample[0,i] = 0
            Rd = (xr**2 + yr**2)/(2*yr)
            control_sample[1,i] = 58*yr/(xr**2 + yr**2)
            dx = Rd*sin((speed*pars.T)/Rd)
            dy = Rd-Rd*cos((speed*pars.T)/Rd)
            xc+=dx*cos(theta)-dy*sin(theta)
            yc+=dx*sin(theta)+dy*cos(theta)
            theta+=(speed*pars.T)/Rd
            linerx.append(xc)
            linery.append(yc)
        
        plt.plot(linerx,linery,'ro-')
        xrt=reshape(control_sample.copy(),2*pars.N,1)
        sr=cf.solver(x0=xrt,p=p,lbx=cf.lbx,ubx=cf.ubx,lbg=cf.lbg,ubg=cf.ubg)
        costr = sr['f']

        #cf for prev_path . If prev path = 1 , rightis prefereed. If prev path = 2 - left is preferred

        x = 0
        if curve_l[0]<3 :
            costl = costl + 10000
        if curve_r[0]>-3 :
            costr = costr + 10000
        if(costr<=costl):
            x = sr['x']
            opp_vehicle_detected_state[opp_target_id] = 1
            print('right path chosen')
        else :
            x = sl['x']
            opp_vehicle_detected_state[opp_target_id] = 2
            print('left path chosen')

        print("Cost from right : ", costr)
        print("Cost from left : ", costl)
        #rect = []
        ax = plt.gca()
        for i in range(no_of_vehicles):
            temp = Rectangle((all_vehicles[i,0]-2,all_vehicles[i,1]-0.5),4,1,linewidth=1,edgecolor='r',facecolor='r')
            plt.plot([all_vehicles[i,0]],[all_vehicles[i,1]],'ro-')
            #rect.append(temp)
            ax.add_patch(temp)
            xv = []
            yv = []
            for j in range(pars.N):
                vehicle = all_vehicles[i]
                xd = vehicle[0]+vehicle[2]*(j)*(pars.T)
                yd = vehicle[1]+vehicle[3]*(j)*(pars.T)
                xv.append(xd)
                yv.append(yd)

            plt.plot(xv,yv,'ro-')

        #pc = PatchCollection(rect, facecolor='r', alpha=0.5, edgecolor=p.None)
        #fig, ax = plt.subplots(1)
        # Add collection to axes
        #ax.add_collection(pc)
        #plt.show()
        u = reshape(x.T,2,pars.N).T      
        ctrlmsg = u[:,1]
        speed_output = u[:,0]
        return ctrlmsg, speed_output
            
    control_sample[0,:] = 1
    control_sample[1,:] = 0
    x0=reshape(control_sample,2*pars.N,1)
    #print(lbg)
    #print(ubg)
    print(p)
    so=cf.solver(x0=x0,p=p,lbx=cf.lbx,ubx=cf.ubx,lbg=cf.lbg,ubg=cf.ubg) 
    opp_vehicle_detected_state[:]= 0
    print("no vehicle detected. Reseting all opp_vehichle_detected_state to 0.")
    x=so['x']
    g=so['g']
    predicted_x = g[2*pars.N]
    predicted_y = g[2*pars.N+1]
    predicted_theta = g[2*pars.N+2]
    predicted_v = g[2*pars.N+3]
    u = reshape(x.T,2,pars.N).T
    ctrlmsg = u[:,1]
    control_output = u[:,0]
    return ctrlmsg, control_output


def detect_anomaly(vehicles, no_of_vehicles) :
    # for i in range(no_of_vehicles):
    #    if vehicles[i,3] < 5 :
    #        return True
    return False


required_gear = 1
with rti.open_connector(
        config_name="MyParticipantLibrary::ObstacleParticipant",
        url=file_path + "/../../Sensors_ego2.xml") as connector:

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
    target_throttle = 0
    aggregate = 0
    nr_dist = 0
    all_vehicles = np.ones((pars.max_no_of_vehicles,6))*10000
    opp_vehicle_detected = np.zeros((pars.max_no_of_vehicles),dtype = int)
    opp_vehicle_detected_state = np.zeros((pars.max_no_of_vehicles),dtype = int)
    if pars.file_path_follow != None:
        trajectory_to_follow = np.loadtxt(pars.file_path_follow,delimiter = ",")[:,[0,1,-2]].T
    else :
        trajectory_to_follow=None
    traj_followed = []
    itr = 0
    total_itr=0
    P = 0
    I = pars.I_start
    D = 0
    hii=[]
    throttle = 0
    while True:
        total_itr=total_itr+1
        itr = itr+1
        if total_itr > pars.save_path_after and pars.save_path_after!=-1:
            break
        print("Iteration no", total_itr)
        input_radar_F.wait()
        input_radar_F.take()
        no_of_vehicles = 0
        all_vehicles[:,:2] = 10000
        all_vehicles[:,2] = 1
        all_vehicles[:,3] = 0
        all_vehicles[:,4:6] = 10000
        opp_vehicle_detected[:] = 0

        wait_topic.wait()
        wait_topic.take()
        wait_msg = []
        for sample in wait_topic.samples.valid_data_iter:
            data = sample.get_dictionary()
            wait_msg = data
        
        for sample in input_radar_F.samples.valid_data_iter:
            data = sample.get_dictionary()
            
            for k in range(len(data['targetsArray'])):
                all_vehicles[no_of_vehicles,0] = data['targetsArray'][k]['posXInChosenRef']
                all_vehicles[no_of_vehicles,1] = data['targetsArray'][k]['posYInChosenRef']
                all_vehicles[no_of_vehicles,2] = data['targetsArray'][k]['absoluteSpeedX']
                all_vehicles[no_of_vehicles,3] = data['targetsArray'][k]['absoluteSpeedY']
                opp_vehicle_detected[no_of_vehicles] = data['targetsArray'][k]['scanerId']
                all_vehicles[no_of_vehicles,0], all_vehicles[no_of_vehicles,1] \
                    = utils.anchorPointToCenter(\
                        all_vehicles[no_of_vehicles,0], \
                        all_vehicles[no_of_vehicles,1], \
                            math.atan2(all_vehicles[no_of_vehicles,3], all_vehicles[no_of_vehicles,2]),\
                            data['targetsArray'][k]['anchorPoint']) 
                all_vehicles[no_of_vehicles,4] = all_vehicles[no_of_vehicles,0] + 1.791102
                all_vehicles[no_of_vehicles,5] = all_vehicles[no_of_vehicles,1]

                print("Vehicle no ", no_of_vehicles)
                print("Vehicle id : ",opp_vehicle_detected[no_of_vehicles])
                print("X : ", all_vehicles[no_of_vehicles,0])
                print("Y : ", all_vehicles[no_of_vehicles,1])
                print("Ego Vehicle frame X : ", all_vehicles[no_of_vehicles,4])
                print("Ego Vehicle frame Y : ", all_vehicles[no_of_vehicles,5])
                print("Speed X : ", all_vehicles[no_of_vehicles,2])
                print("Speed Y : ", all_vehicles[no_of_vehicles,3])
                print("detectionStatus :", data['targetsArray'][k]['detectionStatus'])
                print("type :", data['targetsArray'][k]['type_'])
                #print("name :", data['targetsArray'][k]['name'])
                print("beamIndex :", data['targetsArray'][k]['beamIndex'])
                print("existenceTime :", data['targetsArray'][k]['existenceTime'])
                print("anchorPoint :", data['targetsArray'][k]['anchorPoint'])
                print("referenceFrame :", data['targetsArray'][k]['referenceFrame'])
                no_of_vehicles += 1
            break
        input_radar_left.wait()
        input_radar_left.take()
        print("From left radar")
        for sample in input_radar_left.samples.valid_data_iter:
            data = sample.get_dictionary()
            for k in range(len(data['targetsArray'])):
                if(data['targetsArray'][k]['posXInChosenRef']<0 or data['targetsArray'][k]['posXInChosenRef']>5) :
                    continue
                all_vehicles[no_of_vehicles,0] = -data['targetsArray'][k]['posYInChosenRef']
                all_vehicles[no_of_vehicles,1] = data['targetsArray'][k]['posXInChosenRef']
                all_vehicles[no_of_vehicles,2] = -data['targetsArray'][k]['absoluteSpeedY']
                all_vehicles[no_of_vehicles,3] = data['targetsArray'][k]['absoluteSpeedX']
                opp_vehicle_detected[no_of_vehicles] = data['targetsArray'][k]['scanerId']
                all_vehicles[no_of_vehicles,0], all_vehicles[no_of_vehicles,1] \
                    = utils.anchorPointToCenter(\
                        all_vehicles[no_of_vehicles,0], \
                        all_vehicles[no_of_vehicles,1], \
                            math.atan2(all_vehicles[no_of_vehicles,3], all_vehicles[no_of_vehicles,2]),\
                            data['targetsArray'][k]['anchorPoint']) 
                all_vehicles[no_of_vehicles,4] = all_vehicles[no_of_vehicles,0] + 2.064369
                all_vehicles[no_of_vehicles,5] = all_vehicles[no_of_vehicles,1] + 0.220795
                print("Vehicle no ", no_of_vehicles)
                print("Vehicle id : ",opp_vehicle_detected[no_of_vehicles])
                print("X : ", all_vehicles[no_of_vehicles,0])
                print("Y : ", all_vehicles[no_of_vehicles,1])
                print("Ego Vehicle frame X : ", all_vehicles[no_of_vehicles,4])
                print("Ego Vehicle frame Y : ", all_vehicles[no_of_vehicles,5])
                print("Speed X : ", all_vehicles[no_of_vehicles,2])
                print("Speed Y : ", all_vehicles[no_of_vehicles,3])
                print("detectionStatus :", data['targetsArray'][k]['detectionStatus'])
                print("type :", data['targetsArray'][k]['type_'])
                #print("name :", data['targetsArray'][k]['name'])
                print("beamIndex :", data['targetsArray'][k]['beamIndex'])
                print("existenceTime :", data['targetsArray'][k]['existenceTime'])
                print("anchorPoint :", data['targetsArray'][k]['anchorPoint'])
                print("referenceFrame :", data['targetsArray'][k]['referenceFrame'])
                no_of_vehicles +=1
            break
        
        print("From right radar")
        input_radar_right.wait()
        input_radar_right.take()
        for sample in input_radar_right.samples.valid_data_iter:
            data = sample.get_dictionary()
            for k in range(len(data['targetsArray'])):
                if (data['targetsArray'][k]['posXInChosenRef']<0 or data['targetsArray'][k]['posXInChosenRef']>5) :
                    continue
                all_vehicles[no_of_vehicles,0] = data['targetsArray'][k]['posYInChosenRef']
                all_vehicles[no_of_vehicles,1] = -data['targetsArray'][k]['posXInChosenRef']
                all_vehicles[no_of_vehicles,2] = data['targetsArray'][k]['absoluteSpeedY']
                all_vehicles[no_of_vehicles,3] = -data['targetsArray'][k]['absoluteSpeedX']
                opp_vehicle_detected[no_of_vehicles] = data['targetsArray'][k]['scanerId']
                all_vehicles[no_of_vehicles,0], all_vehicles[no_of_vehicles,1] \
                    = utils.anchorPointToCenter(\
                        all_vehicles[no_of_vehicles,0], \
                        all_vehicles[no_of_vehicles,1], \
                            math.atan2(all_vehicles[no_of_vehicles,3], all_vehicles[no_of_vehicles,2]),\
                            data['targetsArray'][k]['anchorPoint']) 
                all_vehicles[no_of_vehicles,4] = all_vehicles[no_of_vehicles,0] + 2.064369
                all_vehicles[no_of_vehicles,5] = all_vehicles[no_of_vehicles,1] - 0.220795
                print("Vehicle no ", no_of_vehicles)
                print("Vehicle id : ",opp_vehicle_detected[no_of_vehicles])
                print("X : ", all_vehicles[no_of_vehicles,0])
                print("Y : ", all_vehicles[no_of_vehicles,1])
                print("Ego Vehicle frame X : ", all_vehicles[no_of_vehicles,4])
                print("Ego Vehicle frame Y : ", all_vehicles[no_of_vehicles,5])
                print("Speed X : ", all_vehicles[no_of_vehicles,2])
                print("Speed Y : ", all_vehicles[no_of_vehicles,3])
                print("detectionStatus :", data['targetsArray'][k]['detectionStatus'])
                print("type :", data['targetsArray'][k]['type_'])
                #print("name :", data['targetsArray'][k]['name'])
                print("beamIndex :", data['targetsArray'][k]['beamIndex'])
                print("existenceTime :", data['targetsArray'][k]['existenceTime'])
                print("anchorPoint :", data['targetsArray'][k]['anchorPoint'])
                print("referenceFrame :", data['targetsArray'][k]['referenceFrame'])
                no_of_vehicles += 1
            break
        
        input_speed.wait() # Wait for data in the input
        input_speed.take()
        px = 0
        py = 0
        angle_heading = 0
        lsr = 0
        slip_angle = 0
        for sample in input_speed.samples.valid_data_iter:
            data = sample.get_dictionary()
            vx = data['cdgSpeed_x']
            vy = data['cdgSpeed_y']
            vz = data['cdgSpeed_z']
            px = data['cdgPos_x']  
            py = data['cdgPos_y']  
            lsr = data['LSR']
            curr_engine_speed = data['EngineSpeed']
            forcex = data['tireForce_x']
            normalz = data['groundNormal_z']
            angle_heading = data['cdgPos_heading']
            slip_angle = data['slipAngle']
            curr_pedal = data['gasPedal']
            curr_gear = data['GearEngaged']
            curr_speed = math.sqrt(vx*vx+vy*vy+vz*vz)
            print("Current State :",[px,py,angle_heading,curr_speed])
            print("Predicted State :",[predicted_x,predicted_y,predicted_theta,predicted_v])
            print("Current gear :",curr_gear)
            print("Current engine speed :",curr_engine_speed)
            print(pars.gear_change_engine_thres)
            if curr_engine_speed > pars.gear_change_engine_thres and (curr_gear+1>required_gear):
                required_gear = curr_gear + 1
            print("Required gear :",required_gear)
            traj_followed.append([px,py,curr_speed,lsr[0],lsr[1],lsr[2],lsr[3],forcex[0],forcex[1],forcex[2],forcex[3],normalz[0],normalz[1],normalz[2],normalz[3],throttle,curr_engine_speed,curr_gear])
            print("Current Speed : ", curr_speed)

        for l in range(no_of_vehicles):
            vehicle_in_world_X = all_vehicles[l,4] *cos(angle_heading) - all_vehicles[l,5] * sin(angle_heading) +px
            vehicle_in_world_Y = all_vehicles[l,4] * sin(angle_heading) + all_vehicles[l,5] * cos(angle_heading) +py
            all_vehicles[l,4] = vehicle_in_world_X
            all_vehicles[l,5] = vehicle_in_world_Y
            print("Vehicle id : ",opp_vehicle_detected[l])
            print("Global X : ", all_vehicles[l,4])
            print("Global Y : ", all_vehicles[l,5])

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
            target_throttle = 0
            curr_steering = 0
        else:
            if len(data1['roadLinesPolynomsArray'])!=0 :
                ll1 = data1['roadLinesPolynomsArray'][0]
                lr1 = data1['roadLinesPolynomsArray'][1]
                ll2 = data2['roadLinesPolynomsArray'][0]
                lr2 = data2['roadLinesPolynomsArray'][1]
            else :
                print("Changed")
                ll1 = dict()
                ll1['c0'] = 7
                ll1['c1'] = 0
                ll1['c2'] = 0
                ll1['c3'] = 0
                ll2 = dict()
                ll2['c0'] = 7
                ll2['c1'] = 0
                ll2['c2'] = 0
                ll2['c3'] = 0
                lr1 = dict()
                lr1['c0'] = -7
                lr1['c1'] = 0
                lr1['c2'] = 0
                lr1['c3'] = 0
                lr2 = dict()
                lr2['c0'] = -7
                lr2['c1'] = 0
                lr2['c2'] = 0
                lr2['c3'] = 0

            c0 = (ll1['c0'] + lr1['c0'] + ll2['c0'] + lr2['c0'])/4
            roadwidth = (ll1['c0']+ll2['c0'])/2-c0-1
            c1 = (ll1['c1'] + lr1['c1'] + ll2['c1'] + lr2['c1'])/4
            c2 = (ll1['c2'] + lr1['c2'] + ll2['c2'] + lr2['c2'])/4
            c3 = (ll1['c3'] + lr1['c3'] + ll2['c3'] + lr2['c3'])/4
            
            # If one of the lanes is not visible
            if (c0<-pars.threshold) : 
                roadwidth = 2
                c0 = ll1['c0']-2*sqrt(1+ll1['c1']**2)
                c1 = ll1['c1']
                c2 = ll1['c2']
                c3 = ll1['c3']

            if (c0>pars.threshold) :
                roadwidth = 2
                c0 = lr2['c0']+2*sqrt(1+lr2['c1']**2)
                c1 = lr2['c1']
                c2 = lr2['c2']
                c3 = lr2['c3']

            curve_l = [(ll1['c0']+ll2['c0'])/2,(ll1['c1']+ll2['c1'])/2,(ll1['c2']+ll2['c2'])/2,(ll1['c3']+ll2['c3'])/2]
            curve_r = [(lr1['c0']+lr2['c0'])/2,(lr1['c1']+lr2['c1'])/2,(lr1['c2']+lr2['c2'])/2,(lr1['c3']+lr2['c3'])/2]
            curve = [c0,c1,c2,c3]
            print("Time", data['TimeOfUpdate'])
            print("No of vehicles : ", no_of_vehicles)
            print("Curve left : ", curve_l)
            print("Curve right : ", curve_r)
            print("Curve : ", curve)
            curr_steering_array, target_speed_array = (mpcCallback(no_of_vehicles, trajectory_to_follow[:2,:].T, trajectory_to_follow[2,:], np.array([px,py]), angle_heading, curve, curve_l, curve_r, curr_steering, curr_speed, 0, all_vehicles[:,:4], roadwidth, opp_vehicle_detected,opp_vehicle_detected_state))
            curr_steering = float(curr_steering_array[0])
            target_throttle = float(target_speed_array[0])
            out = {}
            if curr_speed < pars.start_speed :
                P = pars.kp_start*(0.06 - lsr[2])
                target_throttle = P + I + D
                I = I + pars.ki_start*(0.06 - lsr[2])
                D = pars.kd_start*0
                print("Current throttle : ", curr_pedal)
                print("slip angle : ",slip_angle[2])
                print("Slip ratio : ", lsr[2])
                print("Speed", curr_speed)
                # target_throttle = float(input("Enter throttle command value : "))
                if target_throttle<0 :
                    target_throttle = target_throttle#*100
            else :
                Q_ang = 0
            if curr_speed > 83 :
                target_throttle = (316.3*5/18) - curr_speed
            out['AcceleratorAdditive'] = max(0,target_throttle)
            out['AcceleratorMultiplicative'] = 0
            out['BrakeAdditive'] = -min(0,target_throttle)
            out['BrakeMultiplicative'] = 0
            out['ClutchAdditive'] = 0
            out['ClutchMultiplicative'] = 0
            out['GearboxAutoMode'] = 9
            
            if pars.manual_gear_change :
                out['GearboxAutoMode'] = 0
            
            out['GearboxTakeOver'] = 1
            out['IsRatioLimit'] = 0
            out['MaxRatio'] = 1000
            out['MinRatio'] = 1
            out['ParkingBrakeAdditive'] = 0
            out['ParkingBrakeMultiplicative'] = 0
            out['ShiftDown'] = 0
            out['ShiftUp'] = 0
            print("Wanted gear :",required_gear)
            out['WantedGear'] = required_gear
            throttle = target_throttle
            out['TimeOfUpdate'] = data['TimeOfUpdate']
            output_speed.instance.set_dictionary(out)
            output_speed.write()
            print("Target Throttle:", target_throttle)
            out_steering = {}
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
            output.instance.set_dictionary(out_steering)
            output.write()
            #print("Time: ", data['TimeOfUpdate'])
        done_topic.instance.set_dictionary(wait_msg)
        done_topic.write()
        print("message written")
        print("")
        # if no_of_vehicles>0:
        #     hii.append((all_vehicles[:no_of_vehicles,4],all_vehicles[:no_of_vehicles,5]))
        
        # if len(hii) >5:
        #     hii=hii[1:]
        # plt.clf()
        # plt.show()
        # plt.ion() 
        # for j in hii:
        #     # hii = np.array(traj_followed).T
            
        #     # # plt.plot(hii[0],hii[1],'k', lw=0.5, alpha=0.5)
        #     # # plt.pause(0.001)
        #     # # plt.plot(trajectory_to_follow[0],trajectory_to_follow[1],'--k', lw=0.5, alpha=0.5)
        #     # # plt.pause(0.001)
        #     plt.plot(j[0] ,j[1] ,'o', lw=0.5, alpha=0.5)
        #     plt.pause(0.001)

               
    # traj_followed = np.array(traj_followed).T
    # print("Trajectory followed :-")
    # print(traj_followed)
    # plt.plot(traj_followed[0],traj_followed[1],'k', lw=0.5, alpha=0.5)
    # plt.plot(trajectory_to_follow[0],trajectory_to_follow[1]/,'--k', lw=0.5, alpha=0.5)
    # plt.plot(all_vehicles[:,4] ,all_vehicles[:,5] ,'o', lw=0.5, alpha=0.5)
    # np.savetxt(pars.file_new_path, traj_followed, delimiter=',')
    # plt.show()

if __name__ == '__main__':  
    start()