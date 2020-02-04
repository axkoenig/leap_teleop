function [ws_center, joints_home, T_BT_h, T_BF_h] = checkCustomHomePosition(params, iiwa)
%CHECKCUSTOMHOMEPOSITION Checks given home position and returns needed
%parameters.

% calculate tool home pos with KST's direct kinematics function
T_BT_h = iiwa.gen_DirectKinematics(params.home_pos);

% workspace center is tool home position
ws_center = T_BT_h(1:3,4).';

% check if tool pose lies within specified workspace
[~,~, inside] = getConstrainedPosition(ws_center, params, iiwa);

if ~inside
    error("Provided home position lies outside defined workspace.")
end

% calculate home flange pose
T_TF = eye(4);
T_TF(3,4) = - params.tool_length;
T_BF_h = T_BT_h * T_TF;

% home joints must be cell array
joints_home = num2cell(params.home_pos);

end

