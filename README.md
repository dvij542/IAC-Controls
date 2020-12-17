# IAC-Control-Strategies
Currently used approach :-

Step 1: Use bayesian optimization to find the global optimal racing line and save the coordinates for the trajectory generated in a text file. References :- https://arxiv.org/pdf/2002.04794.pdf 

Step 2: Run MPC on the global trajectory generated to locally avoid obstacles and perform overtaking maneuver with competitor vehicles. References :- 
[Model-predictive active steering and obstacle avoidance for autonomous ground vehicles](https://www.sciencedirect.com/science/article/pii/S0967066108002025), 
[Optimization‐based autonomous racing of 1:43 scale RC cars](https://arxiv.org/abs/1711.07300), 
[“Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design” ,Jason Kong , Mark Pfeiffer, Georg Schildbach , Francesco Borrelli](https://www.researchgate.net/publication/308851864_Kinematic_and_dynamic_vehicle_models_for_autonomous_driving_control_design)


# Prerequisites
1. Install rtidds-connext, casadi python libraries using pip install

# For hackathon-2, make the following changes as per the old configuration
1. Switch back to Indy_scheduled configuration 
2. Change the directory for controller process (Point it to the new location of ds_control__controller.exe from DS_DDS folder
3. Remove the extra arguments
4. Run the controller scripts from MPC folder

# For hackathon-3, make the following changes as per the new configuration
1. Swith the configuration to hack3_1ego for 1 ego vehicle, hack3_2ego for 2 ego vehicles
2. Comment 2 lines in cnxwrapper.cpp (Refer to the video)
3. Open scade and open scade_controller project
4. Change the id constant to 1
5. Swith the build method to DS_DDS1
6. Check the topic names. If they do not already contain \_ego1 as suffix, add it by going to tools->update topicnames and eecute the command '-o Control::Controller -s _ego1'
7. Rebuild the project to generate the new ds_control__controller.exe file
8. Launch the controller scripts from MPC_ego1 folder

Refer to the following video : [Link](https://drive.google.com/file/d/1ECSju_FBsIroLEYXFWxJVQumX81lfDNb/view?usp=sharing)
