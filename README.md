# IAC-Controls
Control strategies used Indy Autonomous Challenge from IUPUI-IITKGP-USB team
MPC_v1 : Cost function is the summation of distances wrt a target point
MPC_v2 : Cost function is modified to be compatible with the inputs i.e. take the distance from the center line path and angle difference with the path angle.
MPC_v3 : Added inverse distance term with the nearest obstacle to stay away from obstacles.
MPC_v4 : Replaced distance and angle terms with progress along the centerline which is take to be the length between the perpendicular projections of starting and end points to mimic and follow optimum race line whenever possible
MPC_v4_parallel : Run in parallel to SpeedController to increase parallel computtions in MPC_v4
MPC_v5_parallel : Replace center line with global optimal racing line read from file coordiates_c.txt
