function [] = MPExtendTree(vid, sto)
% Extend the tree from the state with index vid toward the state sto
% At each time, make a small step with magnitude params.distOneStep
% Add each intermediate valid state to the tree data structure.
% Stop as soon as an intermediate invalid state is encountered
%   - You can check state validity by first setting the robot position and
%   - then calling the function IsStateValid
% Also stop if an intermediate state reaches the goal
%   - You can check if robot has reached the goal by first setting the
%   - robot position and then calling the function HasRobotReachedGoal

    global mp;
    global params;

    mp.sto = sto;
    mp.vidNear = mp.nodes(vid,:);

    % You can make use of the global variables params and mp to access the
    % necessary information

    % Add your code here ...
    
    dstep     = params.distOneStep;
    C_curr    = sto - mp.nodes(vid, :);
    d         = norm(C_curr);
    u         = dstep * C_curr / d;
    nrSteps = 10; %ceil(d / dstep);

    for k = 1:nrSteps
        params.robot = mp.nodes(vid, :) + u;
        if IsValidState() == 0
            %UR robot configuration is not valid
            return;
        end
        
        %save this new node
        n                     = size(mp.nodes,1);
        mp.nchildren(vid)     = mp.nchildren(vid) + 1;
        mp.nodes(n + 1, :)    = params.robot;
        mp.parents(n + 1)     = vid;
        mp.nchildren(n + 1)   = 0;

        if HasRobotReachedGoal() == 1
            %robot has reached goal
            mp.vidAtGoal = n + 1;
            return;
        end
        vid = n + 1;
    end    
end

