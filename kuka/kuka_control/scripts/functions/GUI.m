function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 03-Sep-2019 15:05:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)

% Choose default command line output for GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in ExitApplicationToggle.
function ExitApplicationToggle_Callback(hObject, eventdata, handles)
% hObject    handle to ExitApplicationToggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global exit_app
exit_app = true;       

% Hint: get(hObject,'Value') returns toggle state of ExitApplicationToggle


% --- Executes on button press in EmergencyStopToggle.
function EmergencyStopToggle_Callback(hObject, eventdata, handles)
% hObject    handle to EmergencyStopToggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global exit_app_emergency
exit_app_emergency = true;
    
% Hint: get(hObject,'Value') returns toggle state of EmergencyStopToggle


% --- Executes on button press in GripperControlToggle.
function GripperControlToggle_Callback(hObject, eventdata, handles)
% hObject    handle to GripperControlToggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global gripper_control

if gripper_control == true 
    gripper_control = false;
else 
    gripper_control = true;
end 

% Hint: get(hObject,'Value') returns toggle state of GripperControlToggle


% --- Executes on button press in KUKAControlToggle.
function KUKAControlToggle_Callback(hObject, eventdata, handles)
% hObject    handle to KUKAControlToggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global kuka_control

if kuka_control == true 
    kuka_control = false;
else 
    kuka_control = true;
end

handles = guihandles(gcf);
if get(hObject,'Value') == 1
    handles.ScalingSlider.Enable = 'off';
else 
    handles.ScalingSlider.Enable = 'on';
end 

% Hint: get(hObject,'Value') returns toggle state of KUKAControlToggle


% --- Executes on slider movement.
function ScalingSlider_Callback(hObject, eventdata, handles)
% hObject    handle to ScalingSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global scaling_factor
scaling_factor = get(hObject,'Value');

handles = guihandles(gcf);
handles.ScalingFactorDisplay.String = "Scaling Factor: " + num2str(get(hObject,'Value'));
    
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function ScalingSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ScalingSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in ResetScalingToggle.
function ResetScalingToggle_Callback(hObject, eventdata, handles)
% hObject    handle to ResetScalingToggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global scaling_factor
scaling_factor = 1;

handles = guihandles(gcf);

if handles.KUKAControlToggle.Value == 0
    handles.ScalingSlider.Value = 1;
    handles.ScalingFactorDisplay.String = "Scaling Factor: 1.0";
end

handles.ResetScalingToggle.Value = 0;

% Hint: get(hObject,'Value') returns toggle state of ResetScalingToggle
