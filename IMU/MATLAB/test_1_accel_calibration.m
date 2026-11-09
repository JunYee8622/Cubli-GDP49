function test_1_accel_calibration(file, varargin)
% TEST_1_ACCEL_CALIBRATION  Accelerometer six-position calibration + gravity magnitude check.
%
%   test_1_accel_calibration('data/six_pose_*.csv', 'Window', 2.0)
%
% Protocol (one recording, ~1 minute):
%   1. Start log_serial with Label 'six_pose'.
%   2. Hold the IMU still with +Z up for ~5 s, press Enter (or the Marker button).
%   3. Move to the next pose, hold ~5 s, press Enter. Repeat for -Z, +X, -X, +Y, -Y (any order).
%   4. Optional: add 3-6 extra random still orientations (Enter after each) for the magnitude check.
% Use the cube faces / a flat table so each axis is close to vertical.
%
% Output: accelerometer offset + scale written to calibration.json, a C++ snippet for config.h,
% and |a| in every still pose before vs after calibration (should all be 1 g).
%
% Options: Window (still window per pose [s]) plus the common ones (see common_args).
p = common_args();
p.addParameter('Window', 2.0, @isscalar);
p.parse(file, varargin{:});
args = p.Results;

c = imu_const();
G = c.G;
AXES = 'XYZ';
SIGNS = '+-';

lg = load_log(args.file, args.Skip);
nrm = vecnorm(lg.acc, 2, 2);
n = floor(args.Window * lg.fs);

segs = marker_segments(lg.marker);
npose = size(segs, 1);
pose_mean = zeros(npose, 3);
pose_std = zeros(npose, 1);
for i = 1:npose
    [a, b] = quietest_window(nrm, segs(i, 1), segs(i, 2), n);
    pose_mean(i, :) = mean(lg.acc(a:b, :), 1);
    pose_std(i) = max(std(lg.acc(a:b, :), 1, 1));
end

% Pick, for each of the six directions, the pose where that axis is most vertical.
best_mean = cell(3, 2);   % column 1 = +axis up, column 2 = -axis up
best_dom = zeros(3, 2);
for i = 1:npose
    m = pose_mean(i, :);
    [~, k] = max(abs(m));
    col = 1 + (m(k) < 0);
    dominance = abs(m(k)) / norm(m);
    if dominance > 0.95 && (isempty(best_mean{k, col}) || dominance > best_dom(k, col))
        best_mean{k, col} = m;
        best_dom(k, col) = dominance;
    end
end

missing = {};
for k = 1:3
    for col = 1:2
        if isempty(best_mean{k, col})
            missing{end + 1} = sprintf('%s%s', SIGNS(col), AXES(k)); %#ok<AGROW>
        end
    end
end
if ~isempty(missing)
    fprintf('Missing still poses for: %s. Found %d segments - did you press Enter after each pose?\n', ...
            strjoin(missing, ', '), npose);
    return
end

offset = zeros(1, 3);
scale = ones(1, 3);
for k = 1:3
    up = best_mean{k, 1}(k);
    down = best_mean{k, 2}(k);
    offset(k) = (up + down) / 2;
    scale(k) = (up - down) / (2 * G);
end

fprintf('\nAccelerometer calibration\n');
rows = cell(3, 3);
for k = 1:3
    rows(k, :) = {AXES(k), offset(k), scale(k)};
end
print_table({'axis', 'offset', 'scale'}, rows);

rows = cell(npose, 4);
before = zeros(1, npose);
after = zeros(1, npose);
for i = 1:npose
    b = norm(pose_mean(i, :)) / G;
    a = norm((pose_mean(i, :) - offset) ./ scale) / G;
    flag = '';
    if pose_std(i) > 0.2, flag = '  <- not still'; end
    rows(i, :) = {sprintf('%d', i), b, a, sprintf('%+.2f%%%s', (a - 1) * 100, flag)};
    before(i) = b;
    after(i) = a;
end
fprintf('\nGravity magnitude in each still pose (target 1.000 g)\n');
print_table({'pose', '|a| raw', '|a| cal', 'error'}, rows);
fprintf('\nworst error: raw %.2f%%  -> calibrated %.2f%%\n', ...
        max(abs(before - 1)) * 100, max(abs(after - 1)) * 100);

cal = save_cal(args.Cal, struct('acc_offset', offset, 'acc_scale', scale));
fprintf('\nsaved to %s. Paste into config.h:\n%s\n', args.Cal, cpp_snippet(cal));

fig = new_fig(args, 800, 400);
ax = axes(fig);
bar(ax, 1:npose, [before(:) after(:)], 'grouped');
hold(ax, 'on');
yline(ax, 1.0, 'k-');
ylim(ax, [min([before after]) - 0.02, max([before after]) + 0.02]);
xlabel(ax, 'still pose');
ylabel(ax, '|a| / g');
title(ax, 'Gravity magnitude check');
legend(ax, {'raw', 'calibrated'});
save_fig(fig, args, 'test1_gravity_magnitude.png');
end
