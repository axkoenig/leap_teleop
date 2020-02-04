function [position] = getScaledPosition(position, ws_center, scaling_factor)
%GETSCALEDPOSITION Scales tool position around workspace center. 

% uniform scaling matrix
scaling_matrix = eye(3,3) * scaling_factor;

% make workspace center new origin
position = position - ws_center;

% scaling around workspace center
position = position * scaling_matrix;

% translate back to KUKA origin
position = position + ws_center;

end

