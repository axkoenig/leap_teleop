function [in_bounds] = checkJointLimits(joints)
%CHECKJOINTLIMITS Returns false if KUKA joint limits exceeded.

% make this function usable for both cell and arrays
if iscell(joints)
    joints = cell2mat(joints);
end 

if length(joints) ~= 7
    error("Expecting array of 7 joint values.")
end
    
% do not change this
clearance = deg2rad(7);
limits = deg2rad([170,120,170,120,170,120,175]) - clearance * ones(1,7);

in_bounds = true;

for i=1:7
    if (abs(joints(i)) > limits(i))
        in_bounds = false;
        fprintf("Joint limit %d exceeded! Given %d, allowed +/- %d", i, joints(i), limits(i));
        return;
    end
end

end

