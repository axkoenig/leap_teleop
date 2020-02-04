classdef callbacks
    %CALLBACKS Class with callbacks for each ROS service
    
    methods (Static)
        
        function resp = exitAppEmergency(~,~,resp)
            
            global exit_app_emergency
            exit_app_emergency = true;
            
            msg = "Performing emergency stop. Terminating all processes immediately.";
            resp.Success = true;
            resp.Message = msg;
            disp(msg)
            
        end
        
        function resp = exitApp(~,~,resp)
            
            global exit_app
            exit_app = true;
            
            msg = "Shutting down robot connection. Exiting the application.";
            resp.Success = true;
            resp.Message = msg;
            disp(msg)
            
        end
        
        function resp = setKukaControl(~,req,resp)
            
            global kuka_control
            
            if req.Data == 1
                kuka_control = true;
                msg = "Activated KUKA control. Listening to /tf topic.";
                resp.Success = true;
                resp.Message = msg;
                disp(msg)
            elseif req.Data == 0
                kuka_control = false;
                msg = "Deactivated KUKA control. Not listening to /tf topic.";
                resp.Success = true;
                resp.Message = msg;
                disp(msg)
            end
            
        end
        
        function resp = setGripperControl(~,req,resp)
            
            global gripper_control
            
            if req.Data == 1
                gripper_control = true;
                msg = "Activated gripper control.";
                resp.Success = true;
                resp.Message = msg;
                disp(msg)
            elseif req.Data == 0
                gripper_control = false;
                msg = "Deactivated gripper control.";
                resp.Success = true;
                resp.Message = msg;
                disp(msg)
            end
            
        end
        
        function resp = setScalingFactor(~,req,resp)
            
            global scaling_factor
            global kuka_control
            global handles
            
            max_scale = handles.ScalingSlider.Max;
            min_scale = handles.ScalingSlider.Min;
            
            if kuka_control
                msg = "Real-time control must be switched off to set scaling factor.";
                resp.Success = false;
                resp.Message = msg;
                disp(msg)
            
            elseif req.Data > max_scale
                msg = strcat("Scaling factor must not be larger than ", string(max_scale), ".");
                resp.Success = false;
                resp.Message = msg;
                disp(msg)
                
            elseif req.Data < min_scale
                msg = strcat("Scaling factor must not be smaller than ", string(min_scale), ".");
                resp.Success = false;
                resp.Message = msg;
                disp(msg)
                
            elseif (req.Data <= max_scale) && (req.Data >= min_scale)
                scaling_factor = req.Data;
                msg = strcat("Setting scaling factor to ", string(req.Data), ".");
                resp.Success = true;
                resp.Message = msg;
                disp(msg)
                
                % update GUI
                handles.ScalingSlider.Value = scaling_factor;
                handles.ScalingFactorDisplay.String = "Scaling Factor: " + num2str(scaling_factor);
            else
                msg = "An error occurred.";
                resp.Success = false;
                resp.Message = msg;
                disp(msg)
            end
            
        end
        
    end
end