function [GUI] = setupGUI()
%SETUPGUI Setup of GUI

GUI = openfig('GUI');
movegui(GUI, 'center');

global handles
handles = guihandles(GUI);

min_scale = 0.1;
max_scale = 1.5;
handles.ScalingSlider.Min = min_scale;
handles.ScalingSlider.Max = max_scale;

end

