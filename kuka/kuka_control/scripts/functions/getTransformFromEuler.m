function [T] = getTransformFromEuler(position, euler)
% GETTRANSFORMFROMEULER Returns transformation matrix.
%   calculates rotation matrix from ZYX euler angles and then assembles 
%   the full 4 by 4 transformation including position.

x_pos = position(1);
y_pos = position(2);
z_pos = position(3);

z_rot = euler(1);
y_rot = euler(2);
x_rot = euler(3);

R_z = [cos(z_rot) -sin(z_rot) 0; sin(z_rot) cos(z_rot) 0; 0 0 1];
R_y = [cos(y_rot) 0 sin(y_rot); 0 1 0; -sin(y_rot) 0 cos(y_rot)];
R_x = [1 0 0; 0 cos(x_rot) -sin(x_rot); 0 sin(x_rot) cos(x_rot)];

R = R_z * R_y * R_x;

T = eye(4,4);
T(1:3, 1:3) = R;
T(1:3, 4) = [x_pos; y_pos; z_pos];

end

