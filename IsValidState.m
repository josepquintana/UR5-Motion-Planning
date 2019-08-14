function status  = IsValidState()

% this function is used to check if a configuration is valid or not (including joint limit and collision constraints)

global params;


status = params.robot(1) >= params.Q1min && params.robot(1) <= params.Q1max && ...
    params.robot(2) >= params.Q2min && params.robot(2) <= params.Q2max && ...
    params.robot(3) >= params.Q3min && params.robot(3) <= params.Q3max && ...
    params.robot(4) >= params.Q4min && params.robot(4) <= params.Q4max && ...
    params.robot(5) >= params.Q5min && params.robot(5) <= params.Q5max && ...
    params.robot(6) >= params.Q6min && params.robot(6) <= params.Q6max;

if status == 1
    collision_point = UR5_collision_checking_Obs(params.ur5_kin, params.robot, params.obstacles);
    if collision_point == 1
        status = 0;
        return;
    end
end



end

