# IAC-Control-Strategies
Currently used approach :-

Step 1: Use bayesian optimization to find the global optimal racing line and save the coordinates for the trajectory generated in a text file. References :- https://arxiv.org/pdf/2002.04794.pdf 

Step 2: Run MPC on the global trajectory generated to locally avoid obstacles and perform overtaking maneuver with competitor vehicles. References :- 
[Model-predictive active steering and obstacle avoidance for autonomous ground vehicles](https://www.sciencedirect.com/science/article/pii/S0967066108002025), 
[Optimization‐based autonomous racing of 1:43 scale RC cars](https://arxiv.org/abs/1711.07300), 
[“Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design” ,Jason Kong , Mark Pfeiffer, Georg Schildbach , Francesco Borrelli](https://www.researchgate.net/publication/308851864_Kinematic_and_dynamic_vehicle_models_for_autonomous_driving_control_design)


# Prerequisites
1. Install rtidds-connext, casadi python libraries using pip install
