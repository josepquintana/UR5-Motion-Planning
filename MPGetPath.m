function JointTrajectory = MPGetPath()

% retrieve the solution path if the goal point is reached and connected

global mp;

JointTrajectory = [];

vid   = mp.vidAtGoal;
while 1
    JointTrajectory =   [mp.nodes(vid,:); JointTrajectory];
    vid             =   mp.parents(vid);
    if vid <= 0
        return;
    end
end

end

