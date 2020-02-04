function [joints, T_base_tool, T_base_flange] = getJointsHome(params, ws_center, iiwa)
%GETJOINTSHOME Calculates home position of KUKA in joint space.
%   Definition of home position:
%   1. Tool position must coincide with workspace center
%   2. Tool z axis must point downwards
%   3. Tool x axis must align with workspace center line
%   4. When viewed from above KUKA links should all lie above center line

%   Approach for simplified inverse kinematics for KUKA home pose
%   A. Calculate flange pose such that tool pose satisfies the constraits
%   B. Joints 3, 5 and 7 are fixed (i.e. 0)
%   C. Calculate joint 1 such that rotation axis of joint 2 is
%      perpendicular to center line
%   D. Calculate joint 2 and joint 4 rotation such that joint 6 lies above
%      desired tool position
%   E. Calculate joint 6 rotation such that tool z axis points downwards

%   The reason for this rather complex script is that using a standard IK
%   solver for a redundant robot might not give repeatable results and
%   might return joint positions that do not adhere to defintion 4 from
%   above.

% access dh parameters
d_0_2 = cell2mat(iiwa.dh_data.d(1));
d_2_4 = cell2mat(iiwa.dh_data.d(3));
d_4_6 = cell2mat(iiwa.dh_data.d(5));
d_6_F = cell2mat(iiwa.dh_data.d(7));

%% STEP A. CALCULATE FLANGE POSE

% basis vectors that define tool orientation (in kuka base frame)
if params.center_line(1) >= 0
    % positive half, x points opposite to center line
    tool_basis_x = - transpose(params.center_line);
else
    % negative hald, x points along center line
    tool_basis_x = transpose(params.center_line);
end
tool_basis_z = [0; 0; -1];
tool_basis_y = cross(tool_basis_z, tool_basis_x);

% get full home pose of tool (in kuka base frame)
p_base_tool = [ws_center(1); ws_center(2); ws_center(3)];
R_base_tool = getRotationFromBasis(tool_basis_x, tool_basis_y, tool_basis_z);
T_base_tool = [R_base_tool, p_base_tool; 0 0 0 1];

% define flange transform (in tool frame)
T_tool_flange = eye(4);
T_tool_flange(3,4) = - params.tool_length;

% calculate desired flange pose (in kuka base frame)
T_base_flange = T_base_tool * T_tool_flange;
p_base_flange = T_base_flange(1:3,4);

%% STEP B. FIXED JOINTS

joint_3 = 0;
joint_5 = 0;
joint_7 = 0;

%% STEP C. CALCULATE JOINT 1 ROTATION

% check if robot should bend over towards positive or negative half
if params.center_line(1) >= 0
    % get rotation axis (always z) and angle
    rot = vrrotvec([1; 0; 0], params.center_line);
else
    % get rotation axis (always z) and angle
    rot = vrrotvec([-1; 0; 0], params.center_line);
end 

% rotation is product of z axis sign (+1 or -1) and angle
joint_1 = rot(3) * rot(4);

%% STEP D. CALCULATE JOINT 2 AND 4 ROTATION

% joint 6 should lie directly above flange
joint_6_pos = p_base_flange + [0; 0; d_6_F];

% simplify KUKA to 2R robot by only moving joint 2 and 4. Joint 6 is
% "virtual" end effector. As we are only moving in the plane that is
% spanned by the KUKA z axis and the direction vector, this now is a 2D
% problem that can be solved easily.

% make joint 2 origin, throw error if joint 6 position not reachable
r = joint_6_pos - [0; 0; d_0_2];
dist = norm(r);
if dist > (d_2_4 + d_4_6)
    error ("Trying to make tool tip and workspace center coincide, but given parameters would lead to non-reachable home pose. Your tool might be too long.")
end

% get desired joint 6 position in plane coordinates
z = r(3);
x = sqrt(norm(r)^2 - z^2);

% calculate joint angles (from CS W4733 Notes - Inverse Kinematics Columbia 
% University, page 5&6) http://www.cs.columbia.edu/~allen/F15/NOTES/invkin.pdf
c = (x^2 + z^2 - d_2_4^2 - d_4_6^2) / 2 * d_2_4 * d_4_6;
s = sqrt(1 - c^2);
angle_1 = atan2(d_4_6*s*x + (d_2_4+d_4_6*c)*z, (d_2_4 + d_4_6*c)*x - d_4_6*s*z);
angle_2 = acos(c);
    
% adjust solution by Columbia University to KUKA joint direction convention
joint_2 = pi/2 - angle_1;        
joint_4 = -angle_2;

%% STEP E. CALCULATE JOINT 6 ROTATION

% find angle such that flange points downwards
joint_6 = pi - joint_2 + joint_4;
   
% mirror results if should be facing in negative x direction
if params.center_line(1) < 0
    joint_2 = -joint_2;
    joint_4 = -joint_4;
    joint_6 = -joint_6;
end 

joints = {joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7};

disp("Calculated home position in degrees is:")
for i = 1:7
    string = strcat({'Joint '}, num2str(i), {': '}, num2str(rad2deg(joints{i})));
    disp(string)
end

% check if solution exceeds joints limits 
in_bounds = checkJointLimits(joints);
if ~in_bounds
    error("Calculated home pose exceeds joint limits.")
end
end 

