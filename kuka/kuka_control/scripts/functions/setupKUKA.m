function [iiwa] = setupKUKA(params)
%SETUPKUKA Sets up KUKA robot object.

% transformation from flange to tool
T_FT = eye(4,4);
T_FT(3,4) = params.tool_length;

% check which robot model
if strcmp(params.robot, 'LBR7R800')
    model = KST.LBR7R800;
elseif strcmp(params.robot, 'LBR14R820')
    model = KST.LBR14R820;
else 
    error("Robot type must either be 'LBR7R800' or 'LBR14R820'.")
end 

% check which flange model
if strcmp(params.flange, 'MF_elektrisch')
    flange = KST.Medien_Flansch_elektrisch;
elseif strcmp(params.flange, 'MF_pneumatisch')
    flange = KST.Medien_Flansch_pneumatisch;
elseif  strcmp(params.flange, 'MF_IO_pneumatisch')
    flange = KST.Medien_Flansch_IO_pneumatisch;
elseif strcmp(params.flange, 'MF_touch_pneumatisch')
    flange = KST.Medien_Flansch_Touch_pneumatisch;
else 
    error("Flange type must either be 'MF_elektrisch', 'MF_pneumatisch', 'MF_IO_pneumatisch' or 'MF_touch_pneumatisch'.")
end

% create robot object (with correct DH parameters)
ip = params.ip;
iiwa = KST(ip, model, flange, T_FT);

end

