function [joints_cur] = checkSolutionValidity(joints_cur, joints_prev)
%CHECKSOLUTIONVALIDITY Checks if inverse kinematics solution is valid.

if any(isnan(joints_cur(:)))
    disp("No IK solution found. Staying in previous pose.")
    joints_cur = joints_prev;
else
    in_bounds = checkJointLimits(joints_cur);
    if ~in_bounds
        disp("Joint limit violated. Staying in previous pose.")
        joints_cur = joints_prev;
    end
end

end

