function test_2_gyro_still(file, varargin)
% TEST_2_GYRO_STILL  Gyro bias, noise, drift and warm-up (IMU completely still).
%
%   test_2_gyro_still('data/still_*.csv', 'Skip', 60)
%
% Protocol: power the board, leave it untouched on a solid surface, record 5+ minutes:
%   log_serial('COM5', 'Label', 'still', 'Seconds', 300)
% Don't touch the table. Use Skip = 60 to ignore the warm-up minute if you want bias after warm-up.
%
% Output: gyro bias (saved to calibration.json), noise, drift with vs without bias removal,
% accelerometer-angle noise (a starting value for Kalman R), Allan deviation plot.
p = common_args();
p.parse(file, varargin{:});
args = p.Results;

AXES = 'XYZ';
lg = load_log(args.file, args.Skip);
cal = load_cal(args.Cal);
acc = apply_cal(lg, cal);
raw = lg.gyr;
T = lg.t(end);
N = size(raw, 1);

bias = mean(raw, 1);
noise = std(raw, 1, 1);
half = floor(N / 2);
bias_first_half = mean(raw(1:half, :), 1);
t2_end = lg.t(end) - lg.t(half + 1);
rows = cell(3, 6);
for k = 1:3
    drift_raw = sum(raw(:, k) .* lg.dt);                                          % no correction at all
    drift_cal = sum((raw(half + 1:end, k) - bias_first_half(k)) .* lg.dt(half + 1:end));  % bias from 1st half
    rows(k, :) = {AXES(k), bias(k), noise(k), drift_raw, drift_raw / T * 60, drift_cal / t2_end * 60};
end
fprintf('\nStill recording: %.0f s at %.0f Hz\n', T, lg.fs);
print_table({'axis', 'bias dps', 'noise dps', 'drift deg', 'raw deg/min', 'cal deg/min'}, rows, 12);
fprintf('  raw = integrating without bias removal; cal = bias from first half applied to second half\n');

ang = accel_angle(acc, args.Axis);
ang_std = std(ang, 1);
fprintf('\nAccelerometer tilt-angle noise (axis %s): %.3f deg rms -> static Kalman R ~ %.4f deg^2\n', ...
        AXES(args.Axis + 1), ang_std, ang_std^2);

[taus, adev] = allan_dev(raw(:, args.Axis + 1) - bias(args.Axis + 1), lg.fs);
arw = interp1(taus, adev, min(max(1.0, taus(1)), taus(end)));
bias_inst = min(adev) / 0.664;
fprintf('Allan (axis %s): angle random walk ~ %.4f deg/sqrt(s) (%.2f deg/sqrt(h)), bias instability ~ %.1f deg/h\n', ...
        AXES(args.Axis + 1), arw, arw * 60, bias_inst * 3600);
fprintf(['  data-driven Kalman start: Q_angle ~ %.2e, R ~ %.4f. In motion the accelerometer is much ' ...
         'worse than at rest, so R usually ends up larger.\n'], arw^2, ang_std^2);
fprintf('temperature: %.1f -> %.1f C\n', lg.temp(1), lg.temp(end));

cal = save_cal(args.Cal, struct('gyro_bias', bias, 'acc_angle_noise_deg', ang_std));
fprintf('\nsaved gyro bias to %s. Paste into config.h:\n%s\n', args.Cal, cpp_snippet(cal));

fig = new_fig(args, 900, 900);
ax1 = subplot(3, 1, 1, 'Parent', fig);
win = max(1, floor(10 * lg.fs));
hold(ax1, 'on');
for k = 1:3
    plot(ax1, lg.t, conv(raw(:, k), ones(win, 1) / win, 'same'), 'DisplayName', sprintf('g%s', lower(AXES(k))));
end
ylabel(ax1, '10 s mean rate [dps]');
title(ax1, 'Gyro bias vs time (warm-up shows as a slope)');
legend(ax1, 'show');
yyaxis(ax1, 'right');
plot(ax1, lg.t, lg.temp, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
ylabel(ax1, 'temp [C]');
ax1.YAxis(2).Color = 'k';

ax2 = subplot(3, 1, 2, 'Parent', fig);
hold(ax2, 'on');
for k = 1:3
    plot(ax2, lg.t, cumsum(raw(:, k) .* lg.dt), 'DisplayName', sprintf('%s raw', AXES(k)));
end
ylabel(ax2, 'integrated angle [deg]');
xlabel(ax2, 'time [s]');
title(ax2, 'Drift if you integrate the gyro with no correction');
legend(ax2, 'show');

ax3 = subplot(3, 1, 3, 'Parent', fig);
loglog(ax3, taus, adev);
xlabel(ax3, 'tau [s]');
ylabel(ax3, 'Allan dev [dps]');
title(ax3, sprintf('Allan deviation, axis %s', AXES(args.Axis + 1)));
grid(ax3, 'on');
save_fig(fig, args, 'test2_gyro_still.png');
end

function [taus, adev] = allan_dev(x, fs)
% Overlapping Allan deviation of a rate signal.
n_taus = 40;
N = numel(x);
theta = cumsum(x(:)) / fs;
ms = unique(floor(logspace(0, log10(floor(N / 10)), n_taus)));
taus = zeros(numel(ms), 1);
adev = zeros(numel(ms), 1);
for j = 1:numel(ms)
    m = ms(j);
    d = theta(2 * m + 1:N) - 2 * theta(m + 1:N - m) + theta(1:N - 2 * m);
    tau = m / fs;
    taus(j) = tau;
    adev(j) = sqrt(sum(d .^ 2) / (2 * tau^2 * (N - 2 * m)));
end
end
