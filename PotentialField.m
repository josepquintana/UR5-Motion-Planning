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

    p1 = [0 0 0.15 1]';
    %p2 = [0 0 0]
    
    iter = 0;
    dstep     = params.distOneStep;
    vid       = -1;
    nrLinks   = 6;
    d         = norm(C_curr);
    u         = dstep * C_curr / d;
    nrSteps = 10; %ceil(d / dstep);


    dhgoal = DHTransformation(C_goal, 6);
    
    
    while mp.vidAtGoal <= 0 && iter < params.maxiteration
        for i = 1:nrLinks
            for j = 1:i
               %Jacobian... 
            end
        end
        
        
        if i == nrLinks
            dhvid  = DHTransformation(mp.nodes(vid, :), 6);
            pp = dhvid*p;
            d = dhgoal - dhvid;
            
        end
        
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
        iter = iter + 1;
    end

    if mp.vidAtGoal >= 1
        JointTrajectory  = MPGetPath();
        JointTrajectory_smooth = SmoothPath(JointTrajectory);
    end
    Draw(JointTrajectory_smooth);
    
    function [jacx, jacy, jacz] = Jacobian(linkStart, linkEnd)
        [ex, ey] = simulator.GetLinkEnd(j);
        [sx, sy] = simulator.GetLinkStart(thetai);
        jacy =  ex - sx;
        jacx = -ey + sy;
    end
end
