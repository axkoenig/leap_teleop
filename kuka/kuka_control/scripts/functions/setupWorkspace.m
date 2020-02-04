function [state, ws_center, joints_home] = setupWorkspace(params, state, p_tree, iiwa)
%SETUPWORKSPACE Get workspace center and home pose, prepare static tf ROS
%messages for home poses.

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

% upload parameters to server
set(p_tree, strcat(params.ns,'ws_center'), {ws_center(1), ws_center(2), ws_center(3)});
disp("Loaded workspace center to ROS parameter server.")

set(p_tree, strcat(params.ns,'joints_home'), joints_home);
disp("Loaded home pose to ROS parameter server.")

% setup tf messages for home poses
state.tf_BF_h = rosmessage('geometry_msgs/TransformStamped');
state.tf_BF_h.ChildFrameId = strcat(params.ns,'flange_frame/home');
state.tf_BF_h.Header.FrameId = strcat(params.ns,'base_frame');
q_BF_h = rotm2quat(T_BF_h(1:3,1:3));
state.tf_BF_h.Transform.Translation.X = T_BF_h(1,4);
state.tf_BF_h.Transform.Translation.Y = T_BF_h(2,4);
state.tf_BF_h.Transform.Translation.Z = T_BF_h(3,4);
state.tf_BF_h.Transform.Rotation.W = q_BF_h(1);
state.tf_BF_h.Transform.Rotation.X = q_BF_h(2);
state.tf_BF_h.Transform.Rotation.Y = q_BF_h(3);
state.tf_BF_h.Transform.Rotation.Z = q_BF_h(4);

state.tf_BT_h = rosmessage('geometry_msgs/TransformStamped');
state.tf_BT_h.ChildFrameId = strcat(params.ns,'tool_frame/home');
state.tf_BT_h.Header.FrameId = strcat(params.ns,'base_frame');
q_BT_h = rotm2quat(T_BT_h(1:3,1:3));
state.tf_BT_h.Transform.Translation.X = T_BT_h(1,4);
state.tf_BT_h.Transform.Translation.Y = T_BT_h(2,4);
state.tf_BT_h.Transform.Translation.Z = T_BT_h(3,4);
state.tf_BT_h.Transform.Rotation.W = q_BT_h(1);
state.tf_BT_h.Transform.Rotation.X = q_BT_h(2);
state.tf_BT_h.Transform.Rotation.Y = q_BT_h(3);
state.tf_BT_h.Transform.Rotation.Z = q_BT_h(4);

end

