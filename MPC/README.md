# IAC-Controls
Control strategies used Indy Autonomous Challenge from IUPUI-IITKGP-USB team
<pre>
mpc_v1.py : Cost function is the summation of distances wrt a target point
mpc_v2.py : Cost function is modified to be compatible with the inputs i.e. take the distance from the center line path and angle difference with the path angle.
mpc_v3.py : Added inverse distance term with the nearest obstacle to stay away from obstacles.
mpc_v4.py : Replaced distance and angle terms with progress along the centerline which is take to be the length between the perpendicular projections of starting and end points to mimic and follow optimum race line whenever possible\n
mpc_v4_parallel.py : Run in parallel to SpeedController to increase parallel computtions in mpc_v4.py
mpc_v5_parallel.py : Replace center line with global optimal racing line read from file coordiates_c.txt
</pre>
