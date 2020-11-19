# IAC-Control-Strategies
Currently used approach :-
<pre>
Step 1: Use bayesian optimization to find the global optimal racing line and save the coordinates for the trajectory generated in a text file. References :- https://arxiv.org/pdf/2002.04794.pdf
Step 2: Run MPC on the global trajectory generated to locally avoid obstacles and perform overtaking maneuver with competitor vehicles 
</pre>
