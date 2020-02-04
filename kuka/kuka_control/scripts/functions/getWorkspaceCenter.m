function [center] = getWorkspaceCenter(params, iiwa)
%GETWORKSPACECENTER Calculates workspace center as midpoint of workspace.
%   Workspace center lies between z limits, directly above center line and 
%   between sphere limits.

sphere_origin = [0, 0, cell2mat(iiwa.dh_data.d(1))];

% center lies in middle of upper and lower z limit
z = (params.z_upper_limit + params.z_lower_limit) / 2;

% alpha is angle from joint 2 origin to desired workspace center
mean_sphere_rad = (params.inner_sphere_limit + params.outer_sphere_limit) / 2;
alpha = asin((z - sphere_origin(3)) / mean_sphere_rad);

% offset of center from KUKA z axis
z_offset = cos(alpha) * mean_sphere_rad;

% normalize center line vector for scaling
direction = params.center_line / norm(params.center_line);

% scale direction vector by z offset to obtain x,y coordinates of center 
center = direction * z_offset;
center(3) = z;

end

