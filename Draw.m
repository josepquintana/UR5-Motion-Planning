function Draw(JointTrajectory)

% this function is used to simulate the planned collision-free motion of UR5
% the number of simulation loops can be tuned by sim_num

global params;

f = figure();
axis equal;
grid on;
grid minor;
camlight;
material metal;
axis([-0.6 0.6 -0.6 0.6 0 1]*1.5);
ur5_disp = UR5Display(f);

[m ,~] = size(JointTrajectory);

JointTrajectory_ext_all = [];

for i = 1 : (m-1)

    dstep     = params.distOneStep;
    diff      = JointTrajectory(i+1,:) - JointTrajectory(i,:);
    nrSteps   = floor( norm(diff) / dstep);
    u         = dstep * diff / norm(diff);
    
    JointTrajectory_ext = [];
    for k = 0 : 1 : nrSteps
        JointTrajectory_ext = [JointTrajectory_ext; JointTrajectory(i,:) + k * u];
    end
    
    JointTrajectory_ext_all = [JointTrajectory_ext_all; JointTrajectory_ext];

end

JointTrajectory_ext_all = [JointTrajectory_ext_all; JointTrajectory(end,:)];


[m_all ,~] = size(JointTrajectory_ext_all);

sim_num = 1;

for loop = 1:sim_num
    
    for j = 1 : m_all
        ur5_disp.draw_configuration( JointTrajectory_ext_all(j,:) );
        hold on;
        [n_obs, ~] = size(params.obstacles);
        for j_obs = 1 : n_obs
            Obs = params.obstacles(j_obs,:);
            plot3( [ Obs(1); Obs(4)], [ Obs(2); Obs(5)], [ Obs(3); Obs(6)], 'r-' , 'LineWidth', 5  );
            hold on;
        end
        pause(0.5);
        refresh(f);
    end
    
end


end






















