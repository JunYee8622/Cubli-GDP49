function test_4_gyro_scale_roll(file, varargin)
% TEST_4_GYRO_SCALE_ROLL  Gyro scale factor from 90 degree rolls.
%
%   test_4_gyro_scale_roll('data/roll_*.csv')
%
% Protocol: cube flat on a face, still >= 2 s. Roll it onto the adjacent face about the tilt axis,
% hold still >= 2 s. Roll back, hold. Repeat ~6 times, mixing slow and fast rolls. No markers needed:
% rolls are detected automatically from the gyro rate.
%
% For each roll, the gyro-integrated angle is compared with (a) the accelerometer angle change
% between the still periods and (b) the nominal 90 deg. A gyro scale error shows as the same
% percentage error on every roll; random errors point to timing (dt) problems.
%
% Options: Expected (nominal roll angle [deg]), Thr (rate threshold for 'moving' [deg/s]),
% Still (still window before/after each roll [s]).
p = common_args();
p.addParameter('Expected', 90.0, @isscalar);
p.addParameter('Thr', 15.0, @isscalar);
p.addParameter('Still', 0.5, @isscalar);
p.parse(file, varargin{:});
args = p.Results;

lg = load_log(args.file, args.Skip);
est = run_filters(lg, load_cal(args.Cal), args);
rate = est.rate;
N = numel(rate);

moving = abs(rate) > args.Thr;
pad = floor(0.15 * lg.fs);
moving = conv(double(moving), ones(2 * pad + 1, 1), 'same') > 0;  % merge brief pauses
edges = find(diff(double(moving)) ~= 0);
% starts = first moving sample, ends = first sample after the move (1-based, end exclusive)
if moving(1)
    starts = edges(2:2:end) + 1;
    ends = edges(3:2:end) + 1;
else
    starts = edges(1:2:end) + 1;
    ends = edges(2:2:end) + 1;
end
nroll = min(numel(starts), numel(ends));
starts = starts(1:nroll);
ends = ends(1:nroll);
n = floor(args.Still * lg.fs);

rows = {};
ratios = [];
for j = 1:nroll
    a = starts(j);
    b = ends(j);
    if a - n < 1 || b - 1 + n > N || (b - a) < 0.1 * lg.fs
        continue
    end
    pre = a - n:a - 1;
    post = b:b + n - 1;
    ref = mean(est.accel(post)) - mean(est.accel(pre));
    i0 = a - floor(n / 2);
    i1 = b + floor(n / 2) - 1;
    gyro = sum(rate(i0:i1) .* lg.dt(i0:i1));
    peak = max(abs(rate(a:b - 1)));
    ratios(end + 1) = ref / gyro; %#ok<AGROW>
    rows(end + 1, :) = {sprintf('%d', size(rows, 1) + 1), peak, (b - a) / lg.fs, ref, gyro, ...
                        sprintf('%+.2f%%', (abs(gyro) / abs(ref) - 1) * 100), ...
                        sprintf('%+.2f', abs(gyro) - args.Expected)}; %#ok<AGROW>
end

if isempty(rows)
    fprintf('No rolls detected. Lower Thr or check the tilt Axis.\n');
    return
end
fprintf('\n%d rolls detected\n', size(rows, 1));
print_table({'roll', 'peak dps', 'dur s', 'accel deg', 'gyro deg', 'gyro vs acc', 'vs nominal'}, rows, 11);
r = ratios;
if mean(r) < 1, dirn = 'high'; else, dirn = 'low'; end
fprintf('\nscale correction (accel/gyro): mean %.4f, std %.4f  -> gyro reads %+.2f%% %s\n', ...
        mean(r), std(r, 1), (1 / mean(r) - 1) * 100, dirn);
fprintf('Consistent %% across slow and fast rolls = scale factor. Scatter = timing or a wobbly roll.\n');

fig = new_fig(args, 1000, 600);
ax1 = subplot(2, 1, 1, 'Parent', fig);
hold(ax1, 'on');
plot(ax1, lg.t, est.accel, 'DisplayName', 'accel angle');
plot(ax1, lg.t, est.gyro, 'DisplayName', 'gyro only');
plot(ax1, lg.t, est.kf, 'DisplayName', 'Kalman');
ylabel(ax1, 'angle [deg]');
legend(ax1, 'show');
ax2 = subplot(2, 1, 2, 'Parent', fig);
plot(ax2, lg.t, rate);
hold(ax2, 'on');
yl = ylim(ax2);
for j = 1:nroll
    x0 = lg.t(starts(j));
    x1 = lg.t(min(ends(j), N));
    patch(ax2, [x0 x1 x1 x0], [yl(1) yl(1) yl(2) yl(2)], [0.12 0.47 0.71], ...
          'FaceAlpha', 0.15, 'EdgeColor', 'none');
end
ylabel(ax2, 'rate [dps]');
xlabel(ax2, 'time [s]');
linkaxes([ax1 ax2], 'x');
save_fig(fig, args, 'test4_roll.png');
end
