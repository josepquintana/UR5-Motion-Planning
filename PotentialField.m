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

    syms Q1 Q2 Q3 Q4 Q5 Q6
    p = [0 0 0.15 1]';    
    
    iter      = 1;
    dstep     = params.distOneStep;
    vid       = 1;
    nrLinks   = 6;
    dhgoal = DHTransformation(C_goal, 6);
    G = subs(dhgoal, [Q1 Q2 Q3 Q4 Q5 Q6], params.goal);
    G = eval(G)*p;
    G = G(1:3,1);
        
    while mp.vidAtGoal <= 0 && iter <= params.maxiteration
        %for i = 1:nrLinks
          %  for j = 1:i
            % ...
          %  end
        %end
                
        dhvid  = DHTransformation(mp.nodes(vid, :), 6);
        pp = dhvid*p;
        pp = pp(1:3);
             
        J = [diff(pp(1), Q1) diff(pp(1), Q2) diff(pp(1), Q3) diff(pp(1), Q4) diff(pp(1), Q5) diff(pp(1), Q6); 
             diff(pp(2), Q1) diff(pp(2), Q2) diff(pp(2), Q3) diff(pp(2), Q4) diff(pp(2), Q5) diff(pp(2), Q6);
             diff(pp(3), Q1) diff(pp(3), Q2) diff(pp(3), Q3) diff(pp(3), Q4) diff(pp(3), Q5) diff(pp(3), Q6)];
        J = double(subs(J,[Q1 Q2 Q3 Q4 Q5 Q6], mp.nodes(vid, :)));
        
        Psub = subs(pp, [Q1 Q2 Q3 Q4 Q5 Q6], mp.nodes(vid, :));
        d = G - eval(Psub);
        d = d/norm(d);
        u = J'*d;
        u = u*dstep;
        params.robot = mp.nodes(vid, :) + u'; % New configuration
        
        n                     = size(mp.nodes,1);
        mp.nchildren(vid)     = mp.nchildren(vid) + 1;
        mp.nodes(n + 1, :)    = params.robot;
        mp.parents(n + 1)     = vid;
        mp.nchildren(n + 1)   = 0;

        if HasRobotReachedGoal() == 1
            mp.vidAtGoal = n + 1;
            return;
        end
        
        iter = iter + 1;
        vid = vid + 1;
        if mod(iter, 20) == 0
            fprintf('Iteration = %g\n', iter);
        end
    end

    if mp.vidAtGoal >= 1
        JointTrajectory  = MPGetPath();
        JointTrajectory_smooth = SmoothPath(JointTrajectory);
    end
    %output the output moves
    OutputMovesForUR(JointTrajectory_smooth, 'PotentialField.script');
    Draw(JointTrajectory_smooth);    
end
