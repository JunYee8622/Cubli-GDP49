function test_5_return_to_start(file, varargin)
% TEST_5_RETURN_TO_START  Does the filter come back to the same angle after motion?
%
%   test_5_return_to_start('data/return_*.csv')
%
% Protocol: cube against a fixed stop (e.g. the corner of a wall / a clamped block), still >= 3 s.
% Pick it up, swing and tumble it about the tilt axis for 20-30 s, put it back exactly against
% the stop, still >= 3 s. No markers needed.
%
% A good estimator ends where it started. Gyro-only shows the drift the filter has to cancel.
%
% Options: Still (still window at start and end [s]) plus the common ones.
p = common_args();
p.addParameter('Still', 2.0, @isscalar);
p.parse(file, varargin{:});
args = p.Results;

lg = load_log(args.file, args.Skip);
est = run_filters(lg, load_cal(args.Cal), args);
n = floor(args.Still * lg.fs);

keys = {'gyro', 'accel', 'comp', 'kf'};
names = {'gyro only', 'accel only', 'complementary', 'Kalman'};
rows = cell(4, 4);
for j = 1:4
    x = est.(keys{j});
    start_ang = mean(x(1:n));
    end_ang = mean(x(end - n + 1:end));
    rows(j, :) = {names{j}, start_ang, end_ang, end_ang - start_ang};
end
fprintf('\nReturn-to-start over %.0f s (peak rate %.0f dps)\n', lg.t(end), max(abs(est.rate)));
print_table({'estimator', 'start', 'end', 'error'}, rows, 14);
fprintf('Kalman residual gyro-bias estimate at the end: %+.3f dps\n', est.kf_bias(end));

fig = new_fig(args, 1000, 400);
ax = axes(fig);
hold(ax, 'on');
plot(ax, lg.t, est.accel, 'LineWidth', 0.5, 'Color', [0.12 0.47 0.71 0.5], 'DisplayName', 'accel');
plot(ax, lg.t, est.gyro, 'DisplayName', 'gyro only');
plot(ax, lg.t, est.comp, 'DisplayName', 'complementary');
plot(ax, lg.t, est.kf, 'DisplayName', 'Kalman');
yline(ax, mean(est.accel(1:n)), 'k--', 'HandleVisibility', 'off');
xlabel(ax, 'time [s]');
ylabel(ax, 'angle [deg]');
title(ax, 'Return to start');
legend(ax, 'show');
save_fig(fig, args, 'test5_return_to_start.png');
end
