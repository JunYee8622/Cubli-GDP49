function visualise_graph(file, varargin)
% VISUALISE_GRAPH  Visualise a recorded IMU log - quick look at everything before running the tests.
%
%   visualise_graph('data/known_*.csv')
%   visualise_graph('fake_data/t5_return.csv', 'Axis', 0, 'T0', 5, 'T1', 20, 'Show', true)
%
% Four stacked panels on one time axis:
%   1. accelerometer, calibrated [m/s^2]
%   2. gyroscope, calibrated [deg/s]
%   3. tilt angle: accelerometer-only, gyro-only, complementary, Kalman [deg]
%   4. temperature [C]
% Dashed vertical lines are the markers inserted with Enter while logging.
%
% Options: T0, T1 (plotted window [s]) plus the common ones.
p = common_args();
p.addParameter('T0', [], @(x) isempty(x) || isscalar(x));
p.addParameter('T1', [], @(x) isempty(x) || isscalar(x));
p.parse(file, varargin{:});
args = p.Results;

lg = load_log(args.file, args.Skip);
cal = load_cal(args.Cal);
[acc, gyr] = apply_cal(lg, cal);
est = run_filters(lg, cal, args);

t = lg.t;
lo = t(1); if ~isempty(args.T0), lo = args.T0; end
hi = t(end); if ~isempty(args.T1), hi = args.T1; end
tilt_axes = 'XY';
axis_name = tilt_axes(args.Axis + 1);

fig = new_fig(args, 1200, 1000);
ax = gobjects(4, 1);
for k = 1:4
    ax(k) = subplot(4, 1, k, 'Parent', fig);
    hold(ax(k), 'on');
end

names = 'XYZ';
for k = 1:3
    plot(ax(1), t, acc(:, k), 'LineWidth', 0.8, 'DisplayName', names(k));
    plot(ax(2), t, gyr(:, k), 'LineWidth', 0.8, 'DisplayName', names(k));
end
ylabel(ax(1), 'accel [m/s^2]');
ylabel(ax(2), 'gyro [deg/s]');

plot(ax(3), t, est.accel, 'LineWidth', 0.6, 'Color', [0.5 0.5 0.5], 'DisplayName', 'accel only');
plot(ax(3), t, est.gyro, ':', 'LineWidth', 0.8, 'DisplayName', 'gyro only');
plot(ax(3), t, est.comp, 'LineWidth', 1.2, 'DisplayName', sprintf('complementary (alpha=%g)', args.Alpha));
plot(ax(3), t, est.kf, 'LineWidth', 1.2, 'DisplayName', 'Kalman');
ylabel(ax(3), sprintf('tilt about %s [deg]', axis_name));

plot(ax(4), t, lg.temp, 'LineWidth', 1.0, 'Color', [0.84 0.15 0.16]);
ylabel(ax(4), 'temp [C]');
xlabel(ax(4), 'time [s]');

% marker boundaries (one dashed line per marker change)
segs = marker_segments(lg.marker);
for j = 2:size(segs, 1)
    for k = 1:4
        xline(ax(k), t(segs(j, 1)), '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.6, 'HandleVisibility', 'off');
    end
end

for k = 1:4
    grid(ax(k), 'on');
    if k < 4
        legend(ax(k), 'show', 'Location', 'northeast', 'NumColumns', 4, 'FontSize', 8);
    end
end
linkaxes(ax, 'x');
xlim(ax(1), [lo hi]);

[~, fname] = fileparts(char(args.file));
title(ax(1), sprintf('%s - %.1f s at %.0f Hz, %d marker segment(s)', strrep(fname, '_', '\_'), t(end), lg.fs, size(segs, 1)));
save_fig(fig, args, [fname '_graph.png']);
end
