function [state] = movePTP(iiwa, state, joints, rel_vel)
%MOVEPTP PTP movement of KUKA.

% publish that real-time mode is off
state.msg_rt.Data = false;
send(state.pub_rt, state.msg_rt);

% publish flag, that KUKA is moving in PTP mode
state.msg_ptp.Data = true;
send(state.pub_ptp, state.msg_ptp);

% move KUKA (this function waits until PTP movement is executed)
iiwa.movePTPJointSpace(joints, rel_vel);

% publish flag, that KUKA is not moving in PTP mode anymore
state.msg_ptp.Data = false;
send(state.pub_ptp, state.msg_ptp);

end

