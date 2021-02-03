import sys
import os

# -- Limit number of OPENBLAS library threads --
# On linux based operation systems, we observed a occupation of all cores by the underlying openblas library. Often,
# this slowed down other processes, as well as the planner itself. Therefore, it is recommended to set the number of
# threads to one. Note: this import must happen before the import of any openblas based package (e.g. numpy)
os.environ['OPENBLAS_NUM_THREADS'] = str(1)

import numpy as np
import math
import json
import time
import configparser

import graph_ltpl

from sys import path as sys_path
from os import path as os_path

file_path = os_path.dirname(os_path.realpath(__file__))
sys_path.append(file_path + "/../../../")
import rticonnextdds_connector as rti


"""
This is the main script to run a minimal example of the graph-based local trajectory planner. The minimal example
generates an offline graph for a race track and drives around the track without the interface to an object list.
Furthermore, logs and advanced vehicle dynamics are not considered.

:Authors:
    * Tim Stahl <tim.stahl@tum.de>

:Created on:
    06.05.2020
"""

# ----------------------------------------------------------------------------------------------------------------------
# IMPORT (should not change) -------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# top level path (module directory)
toppath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(toppath)

track_param = configparser.ConfigParser()
if not track_param.read(toppath + "/params/driving_task.ini"):
    raise ValueError('Specified online parameter config file does not exist or is empty!')

track_specifier = json.loads(track_param.get('DRIVING_TASK', 'track'))

# define all relevant paths
path_dict = {'globtraj_input_path': toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + track_specifier + ".csv",
             'graph_store_path': toppath + "/inputs/stored_graph.pckl",
             'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
             'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini"
             }

# ----------------------------------------------------------------------------------------------------------------------
# INITIALIZATION AND OFFLINE PART --------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# intialize graph_ltpl-class
ltpl_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=path_dict,
                                            visual_mode=True,
                                            log_to_file=False)

# calculate offline graph
ltpl_obj.graph_init(1.0,0.43,720)

# set start pose based on first point in provided reference-line
refline = graph_ltpl.imp_global_traj.src.\
    import_globtraj_csv.import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]



from sys import path as sys_path
from os import path as os_path

file_path = os_path.dirname(os_path.realpath(__file__))
print(file_path)
# sys_path.append(file_path + "/../../../")
import rticonnextdds_connector as rti
# url=file_path + "\\..\\..\\..\\ShapeExample.xml"
# C:\Users\IndyAdmin\rticonnextdds-connector-py\examples\python\ShapeExample.xml
with rti.open_connector(
        config_name="MyParticipantLibrary::GraphParticipant",
        url=file_path+"/../../../Sensors_ego1.xml") as connector:

        input_odom = connector.get_input("StateSubscriber::stateReader") #where to take odom data
        output = connector.get_output("pathPublisher::pathPub")
        done_topic = connector.get_output("simDonePub::simDoneWriter")
        wait_topic = connector.get_input("simWaitSub::simWaitReader")
        input_radar_F = connector.get_input("radarSubscriber_F::radarReader_F")
        input_radar_left = connector.get_input("radarSubscriber_left::radarReader_left")
        input_radar_right = connector.get_input("radarSubscriber_right::radarReader_right")
        # input_odom.wait() # Wait for data in the input
        max_no_of_vehicles = 4
        all_vehicles = np.ones((max_no_of_vehicles,4))*10000

        # wait_topic.wait()
        # wait_topic.take()
        # wait_msg = []
        # for sample in wait_topic.samples.valid_data_iter:
        #     data = sample.get_dictionary()
        #     wait_msg = data
        #     break
        
        # input_odom.wait()
        # input_odom.take()
        #     # print(time.time())
        # for sample in input_odom.samples.valid_data_iter:
        #     odom_data = sample.get_dictionary()
        #     break
        
        # done_topic.instance.set_dictionary(wait_msg)
        # done_topic.write()
        # vel_est = odom_data['cdgSpeed_x'] ## use sample for odom data
        # pos_est = [odom_data['cdgPos_x'],odom_data['cdgPos_y']]
        # print("truee")
        # print(pos_est)
        pos_est = [-298.74 , -1808.66]
        vel_est = 0
        pos_est = np.asarray(pos_est)
        # print("XXXXX" + str(odom_data['cdgPos_heading']))
        # print("ggg")
        # pos_est = refline[0, :]
        heading_est = np.arctan2(np.diff(refline[0:2, 1]), np.diff(refline[0:2, 0])) - np.pi / 2
        # vel_est = 0.0

        # set start pos
        ltpl_obj.set_startpos(pos_est=pos_est,
                            heading_est=heading_est,vel_est=vel_est)

        # ----------------------------------------------------------------------------------------------------------------------
        # ONLINE LOOP ----------------------------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------

        traj_set = {'straight': None}
        sel_action = 'straight'

        tic = time.time()
        while True:
            # -- SELECT ONE OF THE PROVIDED TRAJECTORIES -----------------------------------------------------------------------
            # (here: brute-force, replace by sophisticated behavior planner)
            print("started\n\n")
            obj_list =[]
            
            no_of_vehicles = 0

            all_vehicles[:,:2] = 10000
            all_vehicles[:,2] = 1
            all_vehicles[:,3] = 0
            print("yes")
            wait_topic.wait()
            wait_topic.take()
            wait_msg = []
            for sample in wait_topic.samples.valid_data_iter:
                data = sample.get_dictionary()
                wait_msg = data
            input_radar_F.wait()
            input_radar_F.take()
            print("yes")
            for sample in input_radar_F.samples.valid_data_iter:
                data = sample.get_dictionary()
                
                for k in range(len(data['targetsArray'])):
                    # all_vehicles[no_of_vehicles,0] = data['targetsArray'][k]['posXInChosenRef']
                    # all_vehicles[no_of_vehicles,1] = data['targetsArray'][k]['posYInChosenRef']
                    # all_vehicles[no_of_vehicles,2] = data['targetsArray'][k]['absoluteSpeedX']
                    # all_vehicles[no_of_vehicles,3] = data['targetsArray'][k]['absoluteSpeedY']
                    all_vehicles[no_of_vehicles,0] = data['targetsArray'][k]['posYInChosenRef']
                    all_vehicles[no_of_vehicles,1] = -data['targetsArray'][k]['posXInChosenRef']
                    all_vehicles[no_of_vehicles,2] = math.sqrt(data['targetsArray'][k]['absoluteSpeedX'] ** 2 + data['targetsArray'][k]['absoluteSpeedY'] ** 2 )
                    all_vehicles[no_of_vehicles,3] = data['targetsArray'][k]['posHeadingInChosenRef']
                    no_of_vehicles += 1
                    print("Vehicle no ", no_of_vehicles)
                    print("X : ", data['targetsArray'][k]['posYInChosenRef'])
                    print("Y : ", -data['targetsArray'][k]['posXInChosenRef'])
                    print("Speed X : ", data['targetsArray'][k]['absoluteSpeedY'])
                    print("Speed Y : ", data['targetsArray'][k]['absoluteSpeedX'])
                break
            input_radar_left.wait()
            input_radar_left.take()
            print("From left radar")
            for sample in input_radar_left.samples.valid_data_iter:
                data = sample.get_dictionary()
                for k in range(len(data['targetsArray'])):
                    # if(-data['targetsArray'][k]['posYInChosenRef']<0) :
                    #     continue
                    # all_vehicles[no_of_vehicles,0] = -data['targetsArray'][k]['posXInChosenRef']
                    # all_vehicles[no_of_vehicles,1] = -data['targetsArray'][k]['posYInChosenRef']
                    # all_vehicles[no_of_vehicles,2] = -data['targetsArray'][k]['absoluteSpeedX']
                    # all_vehicles[no_of_vehicles,3] = -data['targetsArray'][k]['absoluteSpeedY']
                    all_vehicles[no_of_vehicles,0] = data['targetsArray'][k]['posYInChosenRef']
                    all_vehicles[no_of_vehicles,1] = -data['targetsArray'][k]['posXInChosenRef']
                    all_vehicles[no_of_vehicles,2] = math.sqrt(data['targetsArray'][k]['absoluteSpeedX'] ** 2 + data['targetsArray'][k]['absoluteSpeedY'] ** 2 )
                    all_vehicles[no_of_vehicles,3] = data['targetsArray'][k]['posHeadingInChosenRef']
                    no_of_vehicles += 1
                    print("Vehicle no ", no_of_vehicles)
                    print("X : ", data['targetsArray'][k]['posYInChosenRef'])
                    print("Y : ", -data['targetsArray'][k]['posXInChosenRef'])
                    print("Speed X : ", data['targetsArray'][k]['absoluteSpeedY'])
                    print("Speed Y : ", data['targetsArray'][k]['absoluteSpeedX'])
                break
            
            print("From right radar")
            input_radar_right.wait()
            input_radar_right.take()
            for sample in input_radar_right.samples.valid_data_iter:
                data = sample.get_dictionary()
                for k in range(len(data['targetsArray'])):
                    # if (data['targetsArray'][k]['posXInChosenRef']>0) :
                    #     continue
                    # all_vehicles[no_of_vehicles,0] = -data['targetsArray'][k]['posYInChosenRef']
                    # all_vehicles[no_of_vehicles,1] = data['targetsArray'][k]['posXInChosenRef']
                    # all_vehicles[no_of_vehicles,2] = -data['targetsArray'][k]['absoluteSpeedY']
                    # all_vehicles[no_of_vehicles,3] = data['targetsArray'][k]['absoluteSpeedX']
                    all_vehicles[no_of_vehicles,0] = data['targetsArray'][k]['posYInChosenRef']
                    all_vehicles[no_of_vehicles,1] = -data['targetsArray'][k]['posXInChosenRef']
                    all_vehicles[no_of_vehicles,2] = math.sqrt(data['targetsArray'][k]['absoluteSpeedX'] ** 2 + data['targetsArray'][k]['absoluteSpeedY'] ** 2 )
                    all_vehicles[no_of_vehicles,3] = data['targetsArray'][k]['posHeadingInChosenRef']
                    no_of_vehicles += 1
                    print("Vehicle no ", no_of_vehicles)
                    print("X : ", data['targetsArray'][k]['posYInChosenRef'])
                    print("Y : ", -data['targetsArray'][k]['posXInChosenRef'])
                    print("Speed X : ", data['targetsArray'][k]['absoluteSpeedY'])
                    print("Speed Y : ", data['targetsArray'][k]['absoluteSpeedX'])
                break
            # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
            # st = time.time()
            input_odom.wait()
            input_odom.take()
                # print(time.time())
            for sample in input_odom.samples.valid_data_iter:
                odom_data = sample.get_dictionary()
                # vel_est = odom_data['cdgSpeed_x'] ## use sample for odom data
                # pos_est = [odom_data['cdgPos_x'],odom_data['cdgPos_y']]
                # # print("truee")
                # # print(pos_est)
                # pos_est = np.asarray(pos_est)
                # print(time.time())
                break
            print('odom X :'+str(odom_data['cdgPos_x']))
            print('odom Y :'+str(odom_data['cdgPos_y']))
            for i in range(no_of_vehicles) :
                cur_veh = {'id': "unique_id"+str(i),   # id of the object
                            'type': "physical",     # type 'physical' (only class implemented so far)
                            'X': all_vehicles[i,0] + odom_data['cdgPos_x'],             # x coordinate
                            'Y': all_vehicles[i,1] + odom_data['cdgPos_y'],             # y coordinate
                            'theta': all_vehicles[i,3] + odom_data['cdgPos_heading'] - math.pi/2,          # orientation (north = 0.0)
                            'v': all_vehicles[i,2],              # velocity along theta
                            'length': 2.0,          # length of the object 
                            'width': 0.5             # width of the object
                            }
                obj_list.append(cur_veh)
            print(obj_list)
            path_check = ltpl_obj.calc_paths(prev_action_id=sel_action,
                                object_list=obj_list)
            print("XXXXXXX Only Paths")
            # for sel_action in ["right","straight","left","follow" ]:

            print(path_check.keys())
            print("XXXXXXX")
            # -- GET POSITION AND VELOCITY ESTIMATE OF EGO-VEHICLE -------------------------------------------------------------
            # (here: simulation dummy, replace with actual sensor readings)
            if traj_set[sel_action] is not None:
                # print(time.time())
                # input_odom.wait() # Wait for data in the input
                # print(time.time())
                
                
                # input_odom.take()
                # # print(time.time())
                # for sample in input_odom.samples.valid_data_iter:
                #     odom_data = sample.get_dictionary()
                vel_est = odom_data['cdgSpeed_x'] ## use sample for odom data
                pos_est = [odom_data['cdgPos_x'],odom_data['cdgPos_y']]
                # print("truee")
                # print(pos_est)
                pos_est = np.asarray(pos_est)
                    # print(time.time())
                    # break
                # pos_est, vel_est = graph_ltpl.testing_tools.src.vdc_dummy.\
                #     vdc_dummy(pos_est=pos_est,
                #             last_s_course=(traj_set[sel_action][0][:, 0]),
                #             last_path=(traj_set[sel_action][0][:, 1:3]),
                #             last_vel_course=(traj_set[sel_action][0][:, 5]),
                #             iter_time=time.time() - tic)
                # print("XXXXXXXXXXXXXXXXXXXXXXXX")    
            tic = time.time()

            # -- CALCULATE VELOCITY PROFILE AND RETRIEVE TRAJECTORIES ----------------------------------------------------------
            traj_set = ltpl_obj.calc_vel_profile(pos_est=pos_est,
                                                vel_est=vel_est)[0]
            print("XXXXXXX Path + Velocity")
            # for sel_action in ["right","straight","left","follow" ]:

            print(traj_set.keys())
            print("XXXXXXX")
            for sel_action in ["right","straight","left","follow" ]:  # try to force 'right', else try next in list
                if sel_action in traj_set.keys():
                    break
            out={}
            out['param']=traj_set[sel_action][0].flatten().tolist() 
            # print(traj_set[sel_action][0][1:3][:10])
            output.instance.set_dictionary(out)
            output.write()
            done_topic.instance.set_dictionary(wait_msg)
            done_topic.write()
            print("message written")
            # en = time.time()
            print(time.time())
            # -- LIVE PLOT (if activated) --------------------------------------------------------------------------------------
            ltpl_obj.visual()
