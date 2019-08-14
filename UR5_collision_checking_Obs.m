function collision_point = UR5_collision_checking_Obs(ur5_kin, P, Obstacles)
% this function is used to check if the UR5 configuration p is in
% collision with all the obstacles, Obstacles

% you do not have to modify this file

collision_point = 0;

[n, ~] = size(Obstacles);

for i = 1 : n
    collision_point_obs = UR5_collision_checking(ur5_kin, P, Obstacles(i,1:3)', Obstacles(i,4:6)', Obstacles(i,7));
    if collision_point_obs == 1
        collision_point = 1;
        return;
    end
end

end