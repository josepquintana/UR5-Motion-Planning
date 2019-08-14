These programs are based on the toolbox, UR5Robot-MEX-Matlab, which is used to provide the basic kinematics and visulization of UR5. You can find the toolbox from the link (https://se.mathworks.com/matlabcentral/fileexchange/70125-ur5robot-mex-matlab). However, you do not need to download it since it is already downloaded in this folder. 

To make the toolbox work, you have to set up a a MEX compiler by running the command in the Command Window:

mex -setup C++

and then run:

run_me

to set up some parameters to make this toolbox function

=======================================================================
 
This toolbox contains interface functions, which you do not have to modify:
 
get_this_dir
run_me
UR5Display
UR5Kinematics

The functions below are used for calculating the shortest distance between two spatial line segments, which you do not have to modify:

case1
case2
case3
minp2l
Dmin

The functions below are support functions for the RRT algorithm. You don't have to modify them but you need to read the description of each function inside it to help you understand how to implement you codes:

Draw
HasRobotReachedGoal
IsValidMovement
IsValidState
MPGetPath
MPInitialize
ParaInitialize
RandomReal
SampleState
SmoothPath
UR5_collision_checking
UR5_collision_checking_Obs


=======================================================================


The two functions below are the ones you have to complete:

MPExtendRRT:  the main function of your RRT algorithm
MPExtendTree: the function will be called by MPExtendRRT when it is necessary






Enjoy and good luck!

Cheng



