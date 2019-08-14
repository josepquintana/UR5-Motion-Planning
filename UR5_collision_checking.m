function collision_point = UR5_collision_checking(ur5_kin, P, Obs_ini, Obs_end, Obs_r)

% this function is used to check if the UR5 configuration P is in collision with a specific capsule obstacle 
% Obs_ini is the starting point of its internal line segment
% Obs_end is the end point of its internal line segment
% Obs_r is the radius of the capsule obstacle

% you do not have to modify this function

collision_point = 0;

threshhold_obs = 0.01;  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UPdate the posture of UR5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

result = ur5_kin.forward_kinematics(P);

offset1 = 0.12;
offset2 = 0.115;
offset3 = 0.075;
offset4 = 0.025;

L1_r = 0.040;
L2_r = 0.035;
L3_r = 0.035;
L4_r = 0.035;
L5_r = 0.025;
L6_r = 0.025;
L7_r = 0.025;
L8_r = 0.025;

L1_ini = result.transform_matrices.T1(1:3,4) + offset1 * result.transform_matrices.T1(1:3,3);
L1_end = result.transform_matrices.T2(1:3,4) + offset2 * result.transform_matrices.T2(1:3,3);

L2_ini = result.transform_matrices.T2(1:3,4);
L2_end = result.transform_matrices.T3(1:3,4);

L3_ini = result.transform_matrices.T4(1:3,4);
L3_end = result.transform_matrices.T5(1:3,4);

L4_ini = result.transform_matrices.T5(1:3,4);
L4_end = result.transform_matrices.T6(1:3,4);

L5_ini = result.transform_matrices.T6(1:3,4) + offset3 * result.transform_matrices.T6(1:3,2) + offset4 * result.transform_matrices.T6(1:3,3);
L5_end = result.transform_matrices.T6(1:3,4) - offset3 * result.transform_matrices.T6(1:3,2) + offset4 * result.transform_matrices.T6(1:3,3);

L6_ini = L5_ini + 2 * offset4 * result.transform_matrices.T6(1:3,3);
L6_end = L5_end + 2 * offset4 * result.transform_matrices.T6(1:3,3);

L7_ini = L6_ini + 2 * offset4 * result.transform_matrices.T6(1:3,3);
L7_end = L6_end + 2 * offset4 * result.transform_matrices.T6(1:3,3);

L8_ini = L7_ini + 2 * offset4 * result.transform_matrices.T6(1:3,3);
L8_end = L7_end + 2 * offset4 * result.transform_matrices.T6(1:3,3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot3( [ L1_ini(1); L1_end(1)], [ L1_ini(2); L1_end(2)], [ L1_ini(3); L1_end(3)], 'r-' , 'LineWidth', 5  ); hold on;
% plot3( [ L2_ini(1); L2_end(1)], [ L2_ini(2); L2_end(2)], [ L2_ini(3); L2_end(3)], 'r-' , 'LineWidth', 5  ); hold on;
% plot3( [ L3_ini(1); L3_end(1)], [ L3_ini(2); L3_end(2)], [ L3_ini(3); L3_end(3)], 'r-' , 'LineWidth', 5  ); hold on;
% plot3( [ L4_ini(1); L4_end(1)], [ L4_ini(2); L4_end(2)], [ L4_ini(3); L4_end(3)], 'r-' , 'LineWidth', 5  ); hold on;
% plot3( [ L5_ini(1); L5_end(1)], [ L5_ini(2); L5_end(2)], [ L5_ini(3); L5_end(3)], 'r-' , 'LineWidth', 5  ); hold on;
% plot3( [ L6_ini(1); L6_end(1)], [ L6_ini(2); L6_end(2)], [ L6_ini(3); L6_end(3)], 'r-' , 'LineWidth', 5  ); hold on;
% plot3( [ L7_ini(1); L7_end(1)], [ L7_ini(2); L7_end(2)], [ L7_ini(3); L7_end(3)], 'r-' , 'LineWidth', 5  ); hold on;
% plot3( [ L8_ini(1); L8_end(1)], [ L8_ini(2); L8_end(2)], [ L8_ini(3); L8_end(3)], 'r-' , 'LineWidth', 5  ); hold on;
% 
% plot3( [ Obs_ini(1); Obs_end(1)], [ Obs_ini(2); Obs_end(2)], [ Obs_ini(3); Obs_end(3)], 'b-' , 'LineWidth', 5  ); hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Collision checking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L1_Obs = Dmin ( L1_ini, L1_end, Obs_ini, Obs_end ) - ( L1_r + Obs_r ) - threshhold_obs;
L2_Obs = Dmin ( L2_ini, L2_end, Obs_ini, Obs_end ) - ( L2_r + Obs_r ) - threshhold_obs;
L3_Obs = Dmin ( L3_ini, L3_end, Obs_ini, Obs_end ) - ( L3_r + Obs_r ) - threshhold_obs;
L4_Obs = Dmin ( L4_ini, L4_end, Obs_ini, Obs_end ) - ( L4_r + Obs_r ) - threshhold_obs;
L5_Obs = Dmin ( L5_ini, L5_end, Obs_ini, Obs_end ) - ( L5_r + Obs_r ) - threshhold_obs;
L6_Obs = Dmin ( L6_ini, L6_end, Obs_ini, Obs_end ) - ( L6_r + Obs_r ) - threshhold_obs;
L7_Obs = Dmin ( L7_ini, L7_end, Obs_ini, Obs_end ) - ( L7_r + Obs_r ) - threshhold_obs;
L8_Obs = Dmin ( L8_ini, L8_end, Obs_ini, Obs_end ) - ( L8_r + Obs_r ) - threshhold_obs;

if L1_Obs <= 0 || L2_Obs <= 0 || L3_Obs <= 0 || L4_Obs <= 0 || L5_Obs <= 0 || L6_Obs <= 0 || L7_Obs <= 0 || L8_Obs <= 0
    collision_point = 1;
    return;
end

