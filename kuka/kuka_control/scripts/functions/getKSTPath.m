function [path] = getKSTPath()
%GETKSTPATH Returns path of KST Matlab client

% truncate current path
src = extractBefore(pwd, 'kuka/kuka_control/scripts');

% go down src path
path = strcat(src, 'modules/KST-Kuka-Sunrise-Toolbox/Matlab_client');

end

