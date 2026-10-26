function lg = load_log(path, skip)
% LOAD_LOG  Read a CSV written by log_serial (or log_serial.py).
%
%   lg = load_log(path, skip)  returns a struct of column vectors / matrices:
%     t [s from first sample], acc [Nx3], gyr [Nx3], temp, marker, dt, fs
if nargin < 2, skip = 0; end

d = readmatrix(path, 'FileType', 'text', 'Delimiter', ',', 'NumHeaderLines', 1, 'CommentStyle', '#');
d = d(~any(isnan(d(:, 1:9)), 2), :);  % drop partial lines

step = diff(d(:, 1));
step(step < 0) = step(step < 0) + 2^32;  % micros() wraps every ~71 minutes
t = [0; cumsum(step)] * 1e-6;

lg.t = t;
lg.acc = d(:, 2:4);
lg.gyr = d(:, 5:7);
lg.temp = d(:, 8);
lg.marker = round(d(:, 9));

if skip > 0
    keep = lg.t >= skip;
    lg.t = lg.t(keep);
    lg.acc = lg.acc(keep, :);
    lg.gyr = lg.gyr(keep, :);
    lg.temp = lg.temp(keep);
    lg.marker = lg.marker(keep);
    lg.t = lg.t - lg.t(1);
end
lg.dt = [median(diff(lg.t)); diff(lg.t)];
lg.fs = 1.0 / median(diff(lg.t));
end
