function [state] = moveRT(iiwa, state, joints)
%MOVERT Real-time movement of KUKA.

% set PTP mode flag to off
state.msg_ptp.Data = false;

% set real-time mode flag to on
state.msg_rt.Data = true;

% move KUKA in real-time
iiwa.sendJointsPositionsf(joints);

end

