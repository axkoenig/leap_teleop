% ROS INTERFACE FOR KUKA SUNRISE TOOLBOX - REAL TIME MOTIONS

warning('off')
addpath(strcat(pwd), 'functions')
addpath(getKSTPath())
debug_mode = false;

%% SETUP

[params, state, tf_tree, p_tree, srv] = setupROS('kuka_real_time_node');
iiwa = setupKUKA(params);
[state, ws_center, joints_home] = setupWorkspace(params, state, p_tree, iiwa);
timing = setupTiming(params);

if ~debug_mode
    % connect to server
    disp("Connecting to the robot. Make sure MatlabToolboxServer is running and connected via ethernet.")
    iiwa.net_establishConnection();
end
GUI = setupGUI();

% access global variables needed in main thread
global exit_app_emergency
global exit_app
global kuka_control
global scaling_factor
global handles

%% SET VARIABLES FOR CONTROL LOOP

fps_counter = 0;
mirroring_time = 0;
pick_up_thresh = 0.01;
mirroring = false;
joints_zero = {0, 0, 0, 0, 0, 0, 0};
% max allow 2 degree real-time jump at pickup
j_max = deg2rad(2);

% IK solver parameters
iterations = 10;
lambda = 0.1;

% position and rotation from base to tool
p_BT_d = [0,0,0];
q_BT_d = [0,0,0,0];

%% CONTROL LOOP

% move to home position
if ~debug_mode
    state = movePTP(iiwa, state, joints_home, params.velocity_ptp_slow);
end
at_home = true;
joints_prev = cell2mat(joints_home);

while true
    
    if exit_app_emergency
        return
    elseif exit_app
        % stop gripper control and exit script
        state.msg_gripper.Data = false;
        send(state.pub_gripper, state.msg_gripper);
        break
    end
    
    setUIProperties(handles);
    drawnow;
    
    % check when tool transform becomes available for first time
    tool_available = canTransform(tf_tree, strcat(params.ns, 'base_frame'), strcat(params.ns, 'tool_frame/desired'));
    
    if tool_available
        % get current transform from tf tree
        tf_BT_d = getTransform(tf_tree, strcat(params.ns, 'base_frame'), strcat(params.ns, 'tool_frame/desired'));
        % check if we receive new data
        timing = getTimings(timing, tf_BT_d);
    end
    
    if kuka_control && tool_available
        
        if timing.timeout && ~at_home
            % not receiving new data for longer than threshold allows
            disp("Time-out occurred. Moving to home position.")
            if ~debug_mode
                iiwa.realTime_stopDirectServoJoints;
                state = movePTP(iiwa, state, joints_home, params.velocity_ptp_fast);
            end
            joints_prev = cell2mat(joints_home);
            at_home = true;
            mirroring = false;
        end
        
        if ~timing.timeout
            % receiving new data, access desired tool position and rotation
            p_BT_d(1) = tf_BT_d.Transform.Translation.X;
            p_BT_d(2) = tf_BT_d.Transform.Translation.Y;
            p_BT_d(3) = tf_BT_d.Transform.Translation.Z;
            q_BT_d(1) = tf_BT_d.Transform.Rotation.W;
            q_BT_d(2) = tf_BT_d.Transform.Rotation.X;
            q_BT_d(3) = tf_BT_d.Transform.Rotation.Y;
            q_BT_d(4) = tf_BT_d.Transform.Rotation.Z;
            
            % scale around workspace center
            p_BT_d = getScaledPosition(p_BT_d, ws_center, scaling_factor);
            
            % enforce workspace boundary constraints if neccessary
            [p_BT_d, error, ~] = getConstrainedPosition(p_BT_d, params, iiwa);
            if error
                disp("Shutting down application.");
                break
            end
            
            % construct transformation matrix for IK solver
            T_BT_d = getTransformFromQuaternion(p_BT_d, q_BT_d);
            
            % do inverse kinematics and check validity
            joints_cur = iiwa.gen_InverseKinematics(joints_prev.', T_BT_d, iterations, lambda).';
            joints_cur = checkSolutionValidity(joints_cur, joints_prev);
            joints_prev = joints_cur;
            joints_cmd = num2cell(joints_cur);
            
            if ~mirroring && at_home
                % receiving new data and should start mirroring
                disp("Started receiving tool transformation stream.")
                prev_mirroring_time = toc;
                
                disp("Checking joint differences at pickup.")
                
                % check if ptp pickup or not
                for i = 1:7
                    diff = joints_home{i} - joints_cmd{i};
                    if abs(diff) > j_max
                        ptp_pickup = true;
                    end 
                end 
                
                if ptp_pickup
                    % jump too big, do ptp movement to pickup pose
                    disp("Executing PTP movement to pickup pose.")
                    if ~debug_mode
                        state = movePTP(iiwa, state, joints_cmd, params.velocity_ptp_fast);
                        iiwa.realTime_startDirectServoJoints();
                        state.msg_rt.Data = true;
                    end
                else
                    % jump small, start real-time control directly
                    disp("Executing real-time movement.")
                    if ~debug_mode
                        iiwa.realTime_startDirectServoJoints();
                        state = moveRT(iiwa, state, joints_cmd);
                    end
                end
                at_home = false;
                mirroring = true;
                ptp_pickup = false;
            end
            
            if mirroring
                
                fps_counter = fps_counter + 1;
                mirroring_time = mirroring_time + (toc - prev_mirroring_time);
                prev_mirroring_time = toc;
                
                if ~debug_mode
                    state = moveRT(iiwa, state, joints_cmd);
                end
            end
        end
        
    elseif ~kuka_control && ~at_home
        % real time control deactivated, send to home pose
        if ~debug_mode
            disp("Deactivating real-time control. Moving home.")
            iiwa.realTime_stopDirectServoJoints;
            state = movePTP(iiwa, state, joints_home, params.velocity_ptp_slow);
        end
        joints_prev = cell2mat(joints_home);
        at_home = true;
        mirroring = false;
    end
    
    % publish info about robot state
    if ~debug_mode
        publishState(iiwa, state, tf_tree);
    end
    
end

rate = fps_counter/mirroring_time;
fprintf("Update rate during mirroring was %f Hz.\n", rate);

if ~debug_mode
    iiwa.realTime_stopDirectServoJoints();
    disp("Moving to zero position. Have a nice day!")
    state = movePTP(iiwa, state, joints_zero, params.velocity_ptp_slow);
    iiwa.net_turnOffServer();
end

close(GUI)
rosshutdown
warning('on')