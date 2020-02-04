function publishState(iiwa, state, tf_tree)
%PUBLISHSTATE Publishes state of the robot to ROS network.

% publish if gripper is active or not
global gripper_control

if gripper_control
    state.msg_gripper.Data = true;
else
    state.msg_gripper.Data = false;
end
send(state.pub_gripper, state.msg_gripper);

% publish if kuka is moving
send(state.pub_ptp, state.msg_ptp);
send(state.pub_rt, state.msg_rt);

% publish tool home pose to tf tree
state.tf_BT_h.Header.Stamp = rostime('now');
sendTransform(tf_tree, state.tf_BT_h);

% publish flange home pose to tf tree
state.tf_BF_h.Header.Stamp = rostime('now');
sendTransform(tf_tree, state.tf_BF_h);

% since status info getters of KST do not work in real-time mode
if state.msg_rt.Data == false
    
    % try catch since the KST's getter functions occasionally produce
    % annoying errors... (e.g. "Output argument "jPos" (and maybe others) 
    % not assigned during call to "getEEF_Moment>getDoubleFromString"").
    
    try
        % send current measured base to flange transform
        p_BF_m = cell2mat(iiwa.getEEFCartesianPosition());
        r_BF_m = cell2mat(iiwa.getEEFCartesianOrientation());
        T_BF_m = getTransformFromEuler(p_BF_m, r_BF_m);
        quats = rotm2quat(T_BF_m(1:3,1:3));
        state.tf_BF_m.Header.Stamp = rostime('now');
        state.tf_BF_m.Transform.Translation.X = T_BF_m(1,4)/1000;
        state.tf_BF_m.Transform.Translation.Y = T_BF_m(2,4)/1000;
        state.tf_BF_m.Transform.Translation.Z = T_BF_m(3,4)/1000;
        state.tf_BF_m.Transform.Rotation.W = quats(1);
        state.tf_BF_m.Transform.Rotation.X = quats(2);
        state.tf_BF_m.Transform.Rotation.Y = quats(3);
        state.tf_BF_m.Transform.Rotation.Z = quats(4);
        sendTransform(tf_tree, state.tf_BF_m);
        
        % send inverse of measured base to flange transform
        T_FB_m = getInverseTransform(T_BF_m);
        quats_inv = rotm2quat(T_FB_m(1:3,1:3));
        state.tf_FB_m.Header.Stamp = rostime('now');
        state.tf_FB_m.Transform.Translation.X = T_FB_m(1,4)/1000;
        state.tf_FB_m.Transform.Translation.Y = T_FB_m(2,4)/1000;
        state.tf_FB_m.Transform.Translation.Z = T_FB_m(3,4)/1000;
        state.tf_FB_m.Transform.Rotation.W = quats_inv(1);
        state.tf_FB_m.Transform.Rotation.X = quats_inv(2);
        state.tf_FB_m.Transform.Rotation.Y = quats_inv(3);
        state.tf_FB_m.Transform.Rotation.Z = quats_inv(4);
        sendTransform(tf_tree, state.tf_FB_m);
        
        % send current measured joint positions [rad]
        joints_pos = cell2mat(iiwa.getJointsPos());
        state.msg_joint_pos.Header.Stamp = rostime('now');
        state.msg_joint_pos.Joint1 = joints_pos(1);
        state.msg_joint_pos.Joint2 = joints_pos(2);
        state.msg_joint_pos.Joint3 = joints_pos(3);
        state.msg_joint_pos.Joint4 = joints_pos(4);
        state.msg_joint_pos.Joint5 = joints_pos(5);
        state.msg_joint_pos.Joint6 = joints_pos(6);
        state.msg_joint_pos.Joint7 = joints_pos(7);
        send(state.pub_joint_pos, state.msg_joint_pos);
        
        % send current measured joint torques [Nm]
        joint_torques_mes = cell2mat(iiwa.getJointsMeasuredTorques());
        state.msg_joint_torques_mes.Header.Stamp = rostime('now');
        state.msg_joint_torques_mes.Joint1 = joint_torques_mes(1);
        state.msg_joint_torques_mes.Joint2 = joint_torques_mes(2);
        state.msg_joint_torques_mes.Joint3 = joint_torques_mes(3);
        state.msg_joint_torques_mes.Joint4 = joint_torques_mes(4);
        state.msg_joint_torques_mes.Joint5 = joint_torques_mes(5);
        state.msg_joint_torques_mes.Joint6 = joint_torques_mes(6);
        state.msg_joint_torques_mes.Joint7 = joint_torques_mes(7);
        send(state.pub_joint_torques_mes, state.msg_joint_torques_mes);
        
        % send current external joint torques [Nm]
        joint_torques_ext = cell2mat(iiwa.getJointsExternalTorques());
        state.msg_joint_torques_ext.Header.Stamp = rostime('now');
        state.msg_joint_torques_ext.Joint1 = joint_torques_ext(1);
        state.msg_joint_torques_ext.Joint2 = joint_torques_ext(2);
        state.msg_joint_torques_ext.Joint3 = joint_torques_ext(3);
        state.msg_joint_torques_ext.Joint4 = joint_torques_ext(4);
        state.msg_joint_torques_ext.Joint5 = joint_torques_ext(5);
        state.msg_joint_torques_ext.Joint6 = joint_torques_ext(6);
        state.msg_joint_torques_ext.Joint7 = joint_torques_ext(7);
        send(state.pub_joint_torques_ext, state.msg_joint_torques_ext);
        
        % send current flange force [N]
        flange_force = cell2mat(iiwa.getEEF_Force());
        state.msg_flange_force.Header.Stamp = rostime('now');
        state.msg_flange_force.Vector.X = flange_force(1);
        state.msg_flange_force.Vector.Y = flange_force(2);
        state.msg_flange_force.Vector.Z = flange_force(3);
        send(state.pub_flange_force, state.msg_flange_force);
        
        % send current flange torque [Nm]
        flange_torque = cell2mat(iiwa.getEEF_Moment());
        state.msg_flange_torque.Header.Stamp = rostime('now');
        state.msg_flange_torque.Vector.X = flange_torque(1);
        state.msg_flange_torque.Vector.Y = flange_torque(2);
        state.msg_flange_torque.Vector.Z = flange_torque(3);
        send(state.pub_flange_torque, state.msg_flange_torque);
    
    catch e
        disp("A getter function in Kuka Sunrise Toolbox produced an error. Don't worry about this. Error message is below.")
        disp(e.message)
    end
end
end

