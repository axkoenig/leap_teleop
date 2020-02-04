% SIMPLE CONTROLLER TO MOVE THE ROBOT IN A JOINT CONFIGURATION

close all
addpath(strcat(pwd), 'functions')
addpath(getKSTPath())

% specify robot IP, model and flange, create robot object
ip = '172.31.1.147';
model = KST.LBR7R800;
flange = KST.Medien_Flansch_elektrisch;
iiwa = KST(ip, model, flange);

% connect to server
iiwa.net_establishConnection();

% positions in joint space
pos_zero = {0, 0, 0, 0, 0, 0, 0};
pos_hand_mounting = {0, 0, 0, deg2rad(-110), 0, deg2rad(-50), 0};
pos_camera_calibr = {0, deg2rad(20), deg2rad(10), deg2rad(-110), 0, deg2rad(90), deg2rad(-45)};

velocity_ptp_slow = 0.15;

% move robot to desired position
iiwa.movePTPJointSpace(pos_hand_mounting, velocity_ptp_slow);

% close connection
iiwa.net_turnOffServer();