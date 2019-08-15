function [JointTrajectory, JointTrajectory_smooth] = PotentialField(C_ini, C_goal, Obs)
% C_ini is the initial configuration of UR5 (dimensino: 1*6, unit: radian)
% C_goal is the goal configuration of UR5 (dimensino: 1*6, unit: radian)
% Obs represents all the capsule obstacles in the workspace (dimension: n*7)
% format of each row of Obs: [P_ini,  P_end,  r] where P_ini is the x-, y-
% and z- coordinates of the initial point of the interal line segment of
% the capsule (diemsion: 1*3 unit: meter), P_end is the x-, y-
% and z- coordinates of the end point of the interal line segment of
% the capsule (diemsion: 1*3 unit: meter), r is the radius of the capsule
% (scalar, unit: meter)

    
    % please define all these three inputs before runing this main function
    global mp;
    global params;

    JointTrajectory = []; 
    JointTrajectory_smooth = [];
    % JointTrajectory is the original solution path and JointTrajectory_smooth 
    % is the path after post-processing the path JointTrajectory

    % JointTrajectory_smooth is the joint trajectories of all the six joints of UR5, 
    % which you should transform them into the format of UR script and deploy them on the real robot.

    % JointTrajectory_smooth is a matrix whose dimension is n * 6 (unit: radian)

    MPInitialize(C_ini);
    ParaInitialize(C_ini, C_goal, Obs);

    p = [0 0 0.15 1]';
    %p2 = [0 0 0]
    
    
    iter      = 1;
    dstep     = params.distOneStep;
    vid       = 1;
    nrLinks   = 6;
    dhgoal = DHTransformation(C_goal, 6);
    dhgoal = dhgoal(1:3,4);
        
    while mp.vidAtGoal <= 0 && iter <= params.maxiteration
        for i = 1:nrLinks
            for j = 1:i
               %Jacobian... 
            end
        end
        
        
        dhvid  = DHTransformation(mp.nodes(vid, :), 6);
        pp = dhvid*p; %% 6x4 · 4x1
        
        
        
        syms Q1 Q2 Q3 Q4 Q5 Q6
        J = [diff(pp, Q1) diff(pp, Q2) diff(pp, Q3) diff(pp, Q4) diff(pp, Q5) diff(pp, Q6)];
        J = double(subs(J,[Q1 Q2 Q3 Q4 Q5 Q6], mp.nodes(vid, :)));
        
        J
        
        
        
        %J = Jacobian(pp, mp.nodes(vid, :));
        d = dhgoal - pp;
        d = d/norm(d);
        u = J'*d
        u = d/norm(d)*dstep;
        params.robot = mp.nodes(vid, :) + u; % New configuration
        
        %%%%%%%
        for k = 1:nrSteps
            params.robot = mp.nodes(vid, :) + u;
            if IsValidState() == 0
                return;
            end
            n                     = size(mp.nodes,1);
            mp.nchildren(vid)     = mp.nchildren(vid) + 1;
            mp.nodes(n + 1, :)    = params.robot;
            mp.parents(n + 1)     = vid;
            mp.nchildren(n + 1)   = 0;

            if HasRobotReachedGoal() == 1
                mp.vidAtGoal = n + 1;
                return;
            end
            vid = n + 1;
        end
        %%%%%%%%%%%
        iter = iter + 1;
    end

    if mp.vidAtGoal >= 1
        JointTrajectory  = MPGetPath();
        JointTrajectory_smooth = SmoothPath(JointTrajectory);
    end
    Draw(JointTrajectory_smooth);
    
    
    %function J = Jacobian(P, theta)
     %   syms Q1 Q2 Q3 Q4 Q5 Q6
      %  J = [diff(P, Q1) diff(P, Q2) diff(P, Q3) diff(P, Q4) diff(P, Q5) diff(P, Q6)];
      %  J = double(subs(J,[Q1 Q2 Q3 Q4 Q5 Q6], [theta(1) theta(2) theta(3) theta(4) theta(5) theta(6)]));
   % end
    
    
end
