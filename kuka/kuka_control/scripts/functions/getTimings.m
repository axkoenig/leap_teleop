function [timing] = getTimings(timing,tf)
%GETTIMINGS Handles all timing related functionalities.

% get current time stamps from tf tree
timing.stamps.cur.sec = tf.Header.Stamp.Sec;
timing.stamps.cur.nsec = tf.Header.Stamp.Nsec;

if isequal(timing.stamps.cur, timing.stamps.prev)
    % equal time stamps (received same tf message), count duration
    cur_time = toc;
    timing.delta = cur_time - timing.prev;
    timing.prev = cur_time;
else
    % different time stamps (received new tf message), reset variables
    timing.prev = toc;
    timing.time = 0;
end

timing.time = timing.time + timing.delta;

% check if time above threshold
if timing.time > timing.thresh
    % timout occurred
    timing.timeout = true;
else
    % receiving new data
    timing.timeout = false;
end

timing.stamps.prev.sec = timing.stamps.cur.sec;
timing.stamps.prev.nsec = timing.stamps.cur.nsec;

end

