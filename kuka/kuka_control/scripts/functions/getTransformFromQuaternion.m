function [T] = getTransformFromQuaternion(position, quaternions)
% GETTRANSFORMFROMQUATERNION Returns transformation matrix.
%   calculates rotation matrix from quaternions and then assembles 
%   the full 4 by 4 transformation including position.

x_pos = position(1);
y_pos = position(2);
z_pos = position(3);

R = quat2rotm(quaternions);

T = eye(4,4);
T(1:3, 1:3) = R;
T(1:3, 4) = [x_pos; y_pos; z_pos];

end

