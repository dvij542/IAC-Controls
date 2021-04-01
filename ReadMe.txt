Ego ID: “3” 
Topic Suffix: “_ego3” 
Wait Topic: “sim_done3” 
Done Topic: “ego_done3” 
Domain ID: 31 

List of VRX DDS published topics :-
1. cameraRoadLinesPolynoms_F1_ego3 
2. cameraRoadLinesPolynoms_F2_ego3 
3. cameraRoadLinesPolynoms_FL_ego3 
4. cameraRoadLinesPolynoms_FR_ego3 
5. cameraRoadLinesPolynoms_RL_ego3 
6. cameraRoadLinesPolynoms_RR_ego3 
7. cameraSensorMovableTargetsBoundingBoxes_F1_ego3 
8. cameraSensorMovableTargetsBoundingBoxes_F2_ego3 
9. cameraSensorMovableTargetsBoundingBoxes_FL_ego3 
10. cameraSensorMovableTargetsBoundingBoxes_FR_ego3 
11. cameraSensorMovableTargetsBoundingBoxes_RL_ego3 
12. cameraSensorMovableTargetsBoundingBoxes_RR_ego3 
13. gpsGPS_ego3 
14. lidarLaserMeter_FlashA_ego3 
15. lidarLaserMeter_FlashB_ego3 
16. lidarLaserMeter_FlashC_ego3 
17. radarSensorMovableTargets_F_ego3 
18. radarSensorMovableTargets_Port_ego3 
19. radarSensorMovableTargets_R_ego3 
20. radarSensorMovableTargets_Stbd_ego3 
21. vehicleStateVehicleOutput_ego3 
22. sim_done3

List of subscribed topics by VRX DDS:
1. toVehicleCabToModelCorrective_ego3 
2. toVehicleSteeringCorrective_ego3 
3. ego_done3

Instruction to start the controller :-
1. Change current directory to ~/IAC-Controls/MPC_ego3/mpc_v5
'cd ~/IAC-Controls/MPC_ego3/mpc_v5'
2. Run the python file main_mpc_v5.py
'python3 main_mpc_v5.py'

Required dependencies are :-
python3
rticonnextdds_connector
casadi