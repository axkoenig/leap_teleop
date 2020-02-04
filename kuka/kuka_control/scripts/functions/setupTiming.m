function [timing] = setupTiming(params)
%SETUPTIMING Setup of timing variables
tic
timing.time = 0;
timing.thresh = params.time_out;   % if not seen new data for [s] move home
timing.delta = 0;
timing.prev = 0;
timing.timeout = true;
timing.stamps.cur.sec = 0;
timing.stamps.cur.nsec = 0;
timing.stamps.prev.sec = 0;
timing.stamps.prev.nsec = 0;
end

