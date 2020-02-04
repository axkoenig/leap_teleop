function checkWorkspace(params, iiwa)
%CHECKWORKSPACE Checks if given workspace parameters are valid.
%   Throws error when any of the conditions is violated. 

% workspace parameter limits
max_opening_angle = deg2rad(180);
min_opening_angle = deg2rad(0);
min_z = 0.1;
max_z = 1.14;
min_inner_sphere = 0.4;
max_outer_sphere = 0.8;
sphere_origin = [0, 0, cell2mat(iiwa.dh_data.d(1))];

if params.center_line(3) ~= 0
    error("Z component of center line not zero. Center line must lie in xy-plane.")
    
elseif params.opening_angle <= min_opening_angle
    error("Workspace opening angle must be larger than %d degrees.", rad2deg(min_opening_angle))

elseif params.opening_angle >= max_opening_angle
    error("Workspace opening angle must be smaller than %d degrees.", rad2deg(max_opening_angle))
    
elseif params.z_lower_limit <= min_z
    error("Workspace lower z limit must be larger than %d meters.", min_z)
    
elseif params.z_upper_limit >= max_z
    error("Workspace upper z limit must be smaller than %d meters.", max_z)

elseif params.z_upper_limit <= params.z_lower_limit
    error("Workspace upper z limit must be larger than lower z limit.")

elseif params.z_upper_limit > sphere_origin(3) + params.outer_sphere_limit
    error("Workspace upper z limit must not be larger than sum of specified outer sphere radius (%d m) and z position of joint 2 (%d m). The maximum allowed value for upper z limit is %d m.", params.outer_sphere_limit, sphere_origin(3), params.outer_sphere_limit + sphere_origin(3))

elseif params.inner_sphere_limit <= min_inner_sphere
    error("Workspace inner sphere limit must be larger than %d meters.", min_inner_sphere)
    
elseif params.outer_sphere_limit >= max_outer_sphere
    error("Workspace outer sphere limit must be smaller than %d meters.", max_outer_sphere)
    
elseif params.outer_sphere_limit <= params.inner_sphere_limit
    error("Outer sphere limit must be larger than inner sphere limit.")
    
elseif params.tool_length < 0
    error("Tool length must not be negative.")
    
end

end

