function [position, error, inside] = getConstrainedPosition(position, params, iiwa)
%GETCONSTRAINEDPOSITION Enforces workspace boundary constraints.

error = false;
inside = true; 

sphere_origin = [0, 0, cell2mat(iiwa.dh_data.d(1))];

%% 1. FEASIBILITY CHECKS 

% point must lie in "positive half" which is defined by center line
if dot(position, params.center_line) <= 0
    disp("Error. Desired position not in positive half of robot.")
    position = NaN(1,3);
    error = true;
    inside = false; 
    return
end

if position(3) < 0
    disp("Error. Desired position lies below KUKA XY plane.")
    position = NaN(1,3);
    error = true;
    inside = false; 
    return
end

%% 2. ENFORCE OPENING ANGLE PLANE CONSTRAINTS

% find direction vector that span workspace planes with KUKA z axis
R_z_r = [cos(-params.opening_angle/2) -sin(-params.opening_angle/2) 0; sin(-params.opening_angle/2) cos(-params.opening_angle/2) 0; 0 0 1];
R_z_l = [cos(params.opening_angle/2) -sin(params.opening_angle/2) 0; sin(params.opening_angle/2) cos(params.opening_angle/2) 0; 0 0 1];
plane_r_dir = params.center_line * R_z_r.';
plane_l_dir = params.center_line * R_z_l.';
plane_r_dir = plane_r_dir/norm(plane_r_dir);
plane_l_dir = plane_l_dir/norm(plane_l_dir);

% normals are pointing outwards from valid workspace
plane_r_normal = cross(plane_r_dir, [0, 0, 1]);
plane_l_normal = -cross(plane_l_dir, [0, 0, 1]);

if dot(position, plane_r_normal) > 0
    disp("Enforcing right opening angle constraint,")
    % vector projection
    a1 = dot(position(1:2), plane_r_dir(1:2)) * plane_r_dir(1:2);
    a2 = position(1:2) - a1;
    position(1:2) = position(1:2) - a2;
    inside = false; 
    
elseif dot(position, plane_l_normal) > 0
    disp("Enforcing left opening angle constraint,")
    % vector projection
    a1 = dot(position(1:2), plane_l_dir(1:2)) * plane_l_dir(1:2);
    a2 = position(1:2) - a1;
    position(1:2) = position(1:2) - a2;
    inside = false; 
end

%% 3. ENFORCE Z PLANE CONSTRAINTS

if position(3) < params.z_lower_limit
    position(3) = params.z_lower_limit;
    disp("Enforcing lower z limit.")
    inside = false; 
elseif position(3) > params.z_upper_limit
    position(3) = params.z_upper_limit;
    disp("Enforcing upper z limit.")
    inside = false; 
end

%% 4. ENFORCE SPHERE CONSTRAINTS

% translate vector from sphere origin to KUKA origin
position = position - sphere_origin;
z = position(3);

% get euclidean distance between origin and desired position
dist = norm(position);

if dist < params.inner_sphere_limit
    
    % find desired offset from z axis with pythagoras
    offset = sqrt(params.inner_sphere_limit^2 - z^2);
    
    % find unit vector in x,y plane and translate accordingly
    direction = position - [0, 0, z];
    unit_vector = direction/norm(direction);
    position = offset * unit_vector + [0, 0, z];
    disp("Enforcing inner sphere constraint.")
    inside = false; 
    
elseif dist > params.outer_sphere_limit
    
    % find desired offset from z axis with pythagoras
    offset = sqrt(params.outer_sphere_limit^2 - z^2);
    
    % find unit vector in x,y plane and translate accordingly
    direction = position - [0, 0, z];
    unit_vector = direction/norm(direction);
    position = offset * unit_vector + [0, 0, z];
    disp("Enforcing outer sphere constraint.")
    inside = false; 
end

% translate vector back to sphere origin from KUKA origin
position = position + sphere_origin;

end

