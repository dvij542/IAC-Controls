Ego ID: “3” 
Topic Suffix: “_ego3” 
Wait Topic: “sim_done3” 
Done Topic: “ego_done3” 
Domain ID: 31 

List of VRX DDS published topics :-
1. cameraRoadLinesPolynoms_F1_ego3 
2. cameraRoadLinesPolynoms_F2_ego3 
3. radarSensorMovableTargets_F_ego3 
4. radarSensorMovableTargets_Port_ego3 
5. radarSensorMovableTargets_Stbd_ego3 
6. vehicleStateVehicleOutput_ego3 
7. sim_done3

List of subscribed topics by VRX DDS:
1. toVehicleCabToModelCorrective_ego3 
2. toVehicleSteeringCorrective_ego3 
3. ego_done3

Instruction to start the controller :-
1. Install dependencies from requirements.txt
'pip3 install -r requirements.txt'
2. Change current directory to ~/IAC-Controls/MPC_ego3/mpc_v5
'cd ~/IAC-Controls/MPC_ego3/mpc_v5'
3. Run the python file main_mpc_v5.py
'python3 main_mpc_v5.py'
