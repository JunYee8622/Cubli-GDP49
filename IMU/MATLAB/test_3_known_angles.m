function test_3_known_angles(file, varargin)
% TEST_3_KNOWN_ANGLES  Static accuracy at known angles.
%
%   test_3_known_angles('data/known_*.csv', 'Angles', [0 90 45 30 0])
%
% Protocol (one recording): place the cube at each reference angle in turn, hold still ~5 s,
% press Enter, move to the next. References from the cube itself: flat on a face = 0,
% rolled onto the next face = 90, a phone inclinometer / digital protractor for anything between.
% Then pass the angles in the same order via 'Angles'.
%
% Output: table of reference vs accel / complementary / Kalman, both absolute and with the
% first pose used as zero (removes IMU mounting offset).
%
% Options: Angles (required, reference angles in order [deg]), Window (still window [s]).
p = common_args();
p.addParameter('Angles', [], @isnumeric);
p.addParameter('Window', 2.0, @isscalar);
p.parse(file, varargin{:});
args = p.Results;
if isempty(args.Angles)
    error('test_3_known_angles:noAngles', "Pass the reference angles, e.g. 'Angles', [0 90 45 30 0]");
end
angles = args.Angles(:);

lg = load_log(args.file, args.Skip);
est = run_filters(lg, load_cal(args.Cal), args);
segs = marker_segments(lg.marker);
if size(segs, 1) < numel(angles)
    fprintf('%d marker segments but %d angles given.\n', size(segs, 1), numel(angles));
    return
end
n = floor(args.Window * lg.fs);

vals = zeros(numel(angles), 5);  % ref, accel, comp, kf, centre index of the window
for i = 1:numel(angles)
    [a, b] = quietest_window(est.rate, segs(i, 1), segs(i, 2), n);
    vals(i, :) = [angles(i), mean(est.accel(a:b)), mean(est.comp(a:b)), mean(est.kf(a:b)), floor((a + b) / 2)];
end

rows = zeros(numel(angles), 8);
for i = 1:numel(angles)
    v = vals(i, :);
    rel_ref = v(1) - vals(1, 1);
    rel = v(2:4) - vals(1, 2:4);
    rows(i, :) = [v(1:4), v(2) - v(1), rel - rel_ref];
end
fprintf('\nStatic angles [deg]. ''rel err'' uses the first pose as zero (mounting offset removed).\n');
print_table({'ref', 'accel', 'comp', 'kf', 'abs err', 'rel accel', 'rel comp', 'rel kf'}, rows);
if size(rows, 1) > 1
    rel = rows(2:end, 6:8);
    m = max(abs(rel), [], 1);
    fprintf('max |rel err|: accel %.2f, comp %.2f, kf %.2f deg\n', m(1), m(2), m(3));
end

fig = new_fig(args, 1000, 400);
ax = axes(fig);
hold(ax, 'on');
plot(ax, lg.t, est.accel, 'LineWidth', 0.5, 'Color', [0.12 0.47 0.71 0.6], 'DisplayName', 'accel');
plot(ax, lg.t, est.comp, 'DisplayName', 'complementary');
plot(ax, lg.t, est.kf, 'DisplayName', 'Kalman');
for i = 1:size(vals, 1)
    plot(ax, lg.t(vals(i, 5)), vals(i, 1) + (vals(1, 2) - vals(1, 1)), 'kx', 'MarkerSize', 10, ...
         'HandleVisibility', 'off');
end
xlabel(ax, 'time [s]');
ylabel(ax, 'angle [deg]');
title(ax, 'Known angles (x = reference, shifted by first-pose offset)');
legend(ax, 'show');
save_fig(fig, args, 'test3_known_angles.png');
end
