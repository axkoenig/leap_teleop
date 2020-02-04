function setUIProperties(handles)
%SETUIPROPERTIES Updates toggle colours and state according to global variables.

green = [0 1 0];
gray = [0.94 0.94 0.94];

global kuka_control
global gripper_control

% kuka control toggle
if kuka_control
    handles.KUKAControlToggle.BackgroundColor = green;
    handles.KUKAControlToggle.Value = 1; 
else
    handles.KUKAControlToggle.BackgroundColor = gray;
    handles.KUKAControlToggle.Value = 0; 
end

% gripper control toggle
if gripper_control
    handles.GripperControlToggle.BackgroundColor = green;
    handles.GripperControlToggle.Value = 1;
else
    handles.GripperControlToggle.BackgroundColor = gray;
    handles.GripperControlToggle.Value = 0;
end

end

