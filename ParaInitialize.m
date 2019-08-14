function [  ] = ParaInitialize(C_ini, C_goal, Obs)

% this function is used to initialize the struct params

global params;

params.robot        = C_ini;
params.goal         = C_goal;
params.obstacles    = Obs;
params.distOneStep  = 0.075;            % the step size in MPExtendTree
params.goaltolerance= 0.1;              % the goal tolerance is used to check if the configuration is close enough to the goal configuration
params.maxiteration = 15000;            % the maximum iterations number for the algorithm
params.smoothiters  = 200;              % the maximum iternation number for the post-processing in SmoothPath
params.ur5_kin      = UR5Kinematics();

% Joint limits of all the six joints of UR5

% Important: please start with these joint limits, if experiencing
% self-collision or insufficient motion range, you can change the joint
% limits carefully.

% please note that there are hard joint limits for the real UR5 robot,
% which is from -360 to -360 degree for all the joints. 

% CAVEAT: using full motion ranges of all the joints may cause self-collision. 
% Self-collision is inplemented by the joint limits below, it is not
% included in the collision checking functions, UR5_collision_checking and UR5_collision_checking_Obs
%{
params.Q1min        = - (90 - 22.5) * pi / 180;
params.Q1max        = (22.5 + 90) * pi / 180;
params.Q2min        = (-90 - 50) * pi / 180;
params.Q2max        = (-90 + 50) * pi / 180;
params.Q3min        = -90 * pi / 180;
params.Q3max        = 90 * pi / 180;
params.Q4min        = -90 * pi / 180;
params.Q4max        = 90 * pi / 180;
params.Q5min        = -90 * pi / 180;
params.Q5max        = 90 * pi / 180;
params.Q6min        = -90 * pi / 180;
params.Q6max        = 90 * pi / 180; 
%}

params.Q1min        = - (180 - 22.5) * pi / 180;
params.Q1max        = (22.5 + 180) * pi / 180;
params.Q2min        = -pi;
params.Q2max        = pi;
params.Q3min        = -180 * pi / 180;
params.Q3max        = 180 * pi / 180;
params.Q4min        = -180 * pi / 180;
params.Q4max        = 180 * pi / 180;
params.Q5min        = -180 * pi / 180;
params.Q5max        = 180 * pi / 180;
params.Q6min        = -180 * pi / 180;
params.Q6max        = 180 * pi / 180;


%{
params.Q1min        = -3.14;
params.Q1max        = 3.14;
params.Q2min        = -pi;
params.Q2max        = -pi;
params.Q3min        = -pi;
params.Q3max        = pi;
params.Q4min        = -pi;
params.Q4max        = -pi;
params.Q5min        = -pi;
params.Q5max        = pi;
params.Q6min        = -pi;
params.Q6max        = pi;
%}

end

