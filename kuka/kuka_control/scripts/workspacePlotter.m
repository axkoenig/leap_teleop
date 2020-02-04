% WORKSPACE PLOTTER TO TEST WORKSPACE DEFINITIONS

close all
addpath(strcat(pwd), 'functions')
addpath(getKSTPath())

% workspace definition
params.ip = '172.31.1.147';
params.robot = 'LBR7R800';
params.flange = 'MF_elektrisch';
params.center_line = [1, -1, 0];
params.opening_angle = deg2rad(50);
params.z_lower_limit = 0.2;
params.z_upper_limit = 0.6;
params.inner_sphere_limit = 0.5;
params.outer_sphere_limit = 0.7;
params.tool_length = 0.106;
params.home_pos = deg2rad([-45, 20, 0, -100, 0, 40, 0]);
params.use_home_pos = false;

iiwa = setupKUKA(params);

% check workspace validity
checkWorkspace(params, iiwa);

% check if we should calculate home pose or not
if params.use_home_pos
    
    % check given home position from parameter server
    [ws_center, joints_home, T_BT_h, T_BF_h] = checkCustomHomePosition(params, iiwa);
    
else
    % calculate workspace center
    ws_center = getWorkspaceCenter(params, iiwa);
    
    % calculate home pose
    [joints_home, T_BT_h, T_BF_h]= getJointsHome(params, ws_center, iiwa);
    
end

% create random positions
samples = 10000;
positions = randn(samples, 3);

% run check for every position
for i = 1:samples
    [positions(i,:), error, ~] = getConstrainedPosition(positions(i,:), params, iiwa);
end

% plot
figure('Name', 'Workspace Plot', 'NumberTitle', 'off', 'Position', [0 0 900 600]);
movegui(gcf, 'center');

hold on
grid on
axis equal

length_coosy = 0.2;
campos([5, 2.5, 2])
tool_x = T_BT_h(1:3,1:3) * [1;0;0];
tool_y = T_BT_h(1:3,1:3) * [0;1;0];
tool_z = T_BT_h(1:3,1:3) * [0;0;1];
flange_x = T_BF_h(1:3,1:3) * [1;0;0];
flange_y = T_BF_h(1:3,1:3) * [0;1;0];
flange_z = T_BF_h(1:3,1:3) * [0;0;1];
tool_x = length_coosy * tool_x / norm(tool_x);
tool_y = length_coosy * tool_y / norm(tool_y);
tool_z = length_coosy * tool_z / norm(tool_z);
flange_x = length_coosy * flange_x / norm(flange_x);
flange_y = length_coosy * flange_y / norm(flange_y);
flange_z = length_coosy * flange_z / norm(flange_z);
params.center_line = params.center_line / norm(params.center_line);

quiver3(0, 0, 0, length_coosy, 0, 0, 'color', 'r', 'DisplayName', 'KUKA X Axis')
quiver3(0, 0, 0, 0, length_coosy, 0, 'color', 'g', 'DisplayName', 'KUKA Y Axis')
quiver3(0, 0, 0, 0, 0, length_coosy, 'color', 'b', 'DisplayName', 'KUKA Z Axis')

quiver3(T_BT_h(1,4), T_BT_h(2,4), T_BT_h(3,4), tool_x(1), tool_x(2), tool_x(3), 'color', 'r', 'DisplayName', 'Tool X Axis')
quiver3(T_BT_h(1,4), T_BT_h(2,4), T_BT_h(3,4), tool_y(1), tool_y(2), tool_y(3),'color', 'g', 'DisplayName', 'Tool Y Axis')
quiver3(T_BT_h(1,4), T_BT_h(2,4), T_BT_h(3,4), tool_z(1), tool_z(2), tool_z(3), 'color', 'b', 'DisplayName', 'Tool Z Axis')

quiver3(T_BF_h(1,4), T_BF_h(2,4), T_BF_h(3,4), flange_x(1), flange_x(2), flange_x(3), 'color', 'r', 'DisplayName', 'Flange X Axis')
quiver3(T_BF_h(1,4), T_BF_h(2,4), T_BF_h(3,4), flange_y(1), flange_y(2), flange_y(3),'color', 'g', 'DisplayName', 'Flange Y Axis')
quiver3(T_BF_h(1,4), T_BF_h(2,4), T_BF_h(3,4), flange_z(1), flange_z(2), flange_z(3), 'color', 'b', 'DisplayName', 'Flange Z Axis')

quiver3(0, 0, 0, params.center_line(1), params.center_line(2), params.center_line(3), 'color', 'c', 'DisplayName', 'Center Line')
scatter3(positions(:,1), positions(:,2), positions(:,3), 'DisplayName', 'Constrainted Positions')
plot3(ws_center(1), ws_center(2), ws_center(3), '.', 'color', 'r', 'MarkerSize', 20, 'DisplayName', 'Workspace Center')

legend

hold off

disp("Workspace center is:")
disp(ws_center)
disp("Home pose is:")
for i=1:7
    disp(rad2deg(joints_home{i}))
end 