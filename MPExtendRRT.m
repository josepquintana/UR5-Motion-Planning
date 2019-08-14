function [JointTrajectory, JointTrajectory_smooth] = MPExtendRRT(C_ini, C_goal, Obs)
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

    iter = 0;
    dmin   = Inf;
    vid    = -1;
    
    while mp.vidAtGoal <= 0 && iter < params.maxiteration

        % Implement the extension of the RRT algorithm inside while loop here ...
        if rand() < .2
            sto = params.goal;
        else
            sto = SampleState();
        end

        n      = size(mp.nodes,1);
        %n      = length(mp.xpts);
        for k = 1:n
            d = norm(sto - mp.nodes(k, :));
            norm(sto - C_ini);
            norm(C_goal - sto);
            
            %d = (sto(1) - mp.xpts(k))^2 + (sto(2) - mp.ypts(k))^2;
            if d < dmin
                dmin = d;
                vid  = k;
            end
        end 
        MPExtendTree(vid, sto);
        iter = iter + 1; 
        if mod(iter, 50) == 0
            fprintf('Iteration = %g\n', iter);
        end
    end

    if mp.vidAtGoal >= 1
        JointTrajectory  = MPGetPath();
        JointTrajectory_smooth = SmoothPath(JointTrajectory);
    end
    
    file = fopen('MPExtendRRT.script', 'w');
    for key=1:size(JointTrajectory_smooth,1)
        fprintf(file, 'movej([%g, %g, %g, %g, %g, %g], a=1, v=1, t=0, r=0)\n', JointTrajectory_smooth(key, 1), JointTrajectory_smooth(key, 2), JointTrajectory_smooth(key, 3), JointTrajectory_smooth(key, 4), JointTrajectory_smooth(key, 5), JointTrajectory_smooth(key, 6));
    end
    fprintf(file, 'sleep(2.0)\n');
    for key=size(JointTrajectory_smooth,1):-1:1
        fprintf(file, 'movej([%g, %g, %g, %g, %g, %g], a=1, v=1, t=0, r=0)\n', JointTrajectory_smooth(key, 1), JointTrajectory_smooth(key, 2), JointTrajectory_smooth(key, 3), JointTrajectory_smooth(key, 4), JointTrajectory_smooth(key, 5), JointTrajectory_smooth(key, 6));
    end
    fprintf(file, 'sleep(2.0)\n');
    fclose(file);
    
    Draw(JointTrajectory_smooth);
    
end

