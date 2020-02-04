function [params, state, tf_tree, p_tree, srv] = setupROS(name)
%SETUPROS Setup ROS services, parameters, publishers and messages
% params struct contains parameters loaded from ros parameter server
% state struct contrains publishers and messages for system state 

if robotics.ros.internal.Global.isNodeActive
    disp("Previous Matlab ROS node still running. Shutting ROS down.")
    rosshutdown
    error("ROS shutdown completed. Please rerun all ROS-related scripts.")
end

% initialize node
rosinit('NodeName', name);
tf_tree = rostf;
p_tree = rosparam;

% namespace
ns = 'kuka/';

% check if parameters exist
while ~has(p_tree, strcat(ns, 'ip'))
    disp("Waiting for KUKA parameters on ROS parameter server.")
    pause(1)
end

% define global variables for service callbacks
global exit_app_emergency
global exit_app
global kuka_control
global gripper_control
global scaling_factor

% initialise global variables
exit_app_emergency = false;
exit_app = false;
kuka_control = false;
gripper_control = false;
scaling_factor = 1;

% load parameters, append namespace
params = rosparam("get", ns);
params.ns = ns;
params.center_line = getArrayFromCell(params.center_line);
params.opening_angle = deg2rad(params.opening_angle);
params.home_pos = deg2rad(getArrayFromCell(params.home_pos));

% measured transformation from kuka base to flange frame
state.tf_BF_m = rosmessage('geometry_msgs/TransformStamped');
state.tf_BF_m.ChildFrameId = strcat(params.ns,'flange_frame/measured');
state.tf_BF_m.Header.FrameId = strcat(params.ns,'base_frame');

% transform from flange frame to kuka base (apriltag camera wants this)
state.tf_FB_m = rosmessage('geometry_msgs/TransformStamped');
state.tf_FB_m.ChildFrameId = strcat(params.ns,'base_frame');
state.tf_FB_m.Header.FrameId = strcat(params.ns,'flange_frame/measured');

% setup publishers and messages for kuka state
state.pub_joint_pos = rospublisher(strcat(params.ns,'joint_positions'), 'kuka_msgs/jointPositions');
state.pub_joint_torques_mes = rospublisher(strcat(params.ns,'joint_torques_measured'), 'kuka_msgs/jointTorquesMeasured');
state.pub_joint_torques_ext = rospublisher(strcat(params.ns,'joint_torques_external'), 'kuka_msgs/jointTorquesExternal');
state.pub_flange_force = rospublisher(strcat(params.ns,'flange_force'), 'geometry_msgs/Vector3Stamped');
state.pub_flange_torque = rospublisher(strcat(params.ns,'flange_torque'), 'geometry_msgs/Vector3Stamped');
state.msg_joint_pos = rosmessage('kuka_msgs/jointPositions');
state.msg_joint_torques_mes = rosmessage('kuka_msgs/jointTorquesMeasured');
state.msg_joint_torques_ext = rosmessage('kuka_msgs/jointTorquesExternal');
state.msg_flange_force = rosmessage('geometry_msgs/Vector3Stamped');
state.msg_flange_torque = rosmessage('geometry_msgs/Vector3Stamped');

% publisher and message for gripper control status
state.pub_gripper = rospublisher(strcat(params.ns, 'gripper_control'), 'std_msgs/Bool');
state.msg_gripper = rosmessage('std_msgs/Bool');

% publisher and message for rt and ptp motion flags (messages are false by default)
state.pub_rt = rospublisher(strcat(params.ns, 'moving_rt'), 'std_msgs/Bool');
state.pub_ptp = rospublisher(strcat(params.ns, 'moving_ptp'), 'std_msgs/Bool');
state.msg_rt = rosmessage('std_msgs/Bool');
state.msg_ptp = rosmessage('std_msgs/Bool');

% provide services
srv.exit_em = rossvcserver(strcat(params.ns,'exit_app_emergency'), 'kuka_msgs/exitAppEmergency', @callbacks.exitAppEmergency);
srv.exit = rossvcserver(strcat(params.ns,'exit_app'), 'kuka_msgs/exitApp', @callbacks.exitApp);
srv.kuka = rossvcserver(strcat(params.ns,'kuka_control'), 'kuka_msgs/setKukaControl', @callbacks.setKukaControl);
srv.gripper = rossvcserver(strcat(params.ns,'gripper_control'), 'kuka_msgs/setGripperControl', @callbacks.setGripperControl);
srv.scale = rossvcserver(strcat(params.ns,'scaling_factor'), 'kuka_msgs/setScalingFactor', @callbacks.setScalingFactor);

end

