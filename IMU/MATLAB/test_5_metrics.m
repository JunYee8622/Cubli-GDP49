function test_5_metrics(file, varargin)
% TEST_5_METRICS  Compare tilt filters DURING motion, on a test 5 (return-to-start) recording.
%
%   test_5_metrics('data/return_1.csv', 'Axis', 1)
%   test_5_metrics('data/return_1.csv', 'Axis', 1, 'Alphas', [0.98 0.989 0.995 0.998], ...
%                  'QAngles', [1e-3 1e-4])
%
% Same recording as test_5_return_to_start: cube still against a fixed stop at the start and at
% the end (>= 2 s each), swung about the tilt axis in between.
%
% Reference ("truth") during the motion: the gyro-only angle, with a straight-line drift
% correction so it matches the accelerometer in the still windows at both ends. This is valid
% because, after calibration, the gyro drifts < 0.3 deg/min and its scale error is < 0.1 %
% (tests 2 and 4), so over a ~30 s recording it is accurate to ~0.1 deg.
%
% Note: because the reference is built from the gyro, filters that trust the gyro more always
% look better during motion. The 'bias test' column shows the price of that: the angle error
% each filter would settle at if the gyro bias shifted by BiasStep deg/s (e.g. after the motor
% warms the board). The Kalman filter estimates the bias, so it should stay close to 0 there.
%
% Columns:
%   motion RMS / max   error vs reference while the cube is moving [deg]  (lower = better)
%   still noise        std of the estimate in the first still window [deg]
%   settle s           time after the last movement until |error| stays < Tol
%   end err            end - start angle, as in test 5 [deg]
%   bias test          extra end error with the gyro bias shifted by BiasStep [deg]
%
% Options: Alphas, QAngles, Still (still window [s]), Thr (moving threshold [deg/s]),
%          Tol (settling tolerance [deg]), BiasStep [deg/s], plus the common ones
%          (Axis, Cal, Skip, Out, QBias, R, Show).
p = common_args();
p.addParameter('Alphas', [0.98 0.989 0.995 0.998], @isnumeric);
p.addParameter('QAngles', [1e-3 1e-4], @isnumeric);
p.addParameter('Still', 2.0, @isscalar);
p.addParameter('Thr', 20.0, @isscalar);
p.addParameter('Tol', 0.2, @isscalar);
p.addParameter('BiasStep', 0.5, @isscalar);
p.parse(file, varargin{:});
args = p.Results;

% ---------------- data ----------------
lg = load_log(args.file, args.Skip);
cal = load_cal(args.Cal);
[acc, gyr] = apply_cal(lg, cal);
rate = gyr(:, args.Axis + 1);
acc_ang = accel_angle(acc, args.Axis);
t = lg.t;
dt = lg.dt;
N = numel(rate);
n = floor(args.Still * lg.fs);
if 2 * n >= N
    error('test_5_metrics:short', 'Recording too short for %.1f s still windows.', args.Still);
end
w0 = 1:n;              % start still window
w1 = N - n + 1:N;      % end still window

% ---------------- motion period ----------------
moving = abs(rate) > args.Thr;
i_first = find(moving, 1, 'first');
i_last = find(moving, 1, 'last');
if isempty(i_first)
    fprintf('No motion above %.0f deg/s found. Lower Thr or check the Axis.\n', args.Thr);
    return
end
if i_first <= n || i_last > N - n
    fprintf(['WARNING: the motion overlaps the %.1f s still windows. Start logging with the cube ' ...
             'still, and keep it still for a few seconds before Ctrl+C.\n'], args.Still);
end
act = i_first:i_last;
after = i_last:N;

% ---------------- reference: drift-corrected gyro ----------------
g = gyro_only(rate, dt, 0);
e0 = mean(wrap180(acc_ang(w0) - g(w0)));
e1 = e0 + wrap180(mean(wrap180(acc_ang(w1) - g(w1))) - e0);
t0 = mean(t(w0));
t1 = mean(t(w1));
ref = g + e0 + (e1 - e0) * (t - t0) / (t1 - t0);
drift = e1 - e0;

% ---------------- estimators ----------------
labels = {'accel only'};
ests = {acc_ang};
ests_b = {acc_ang};                      % accel does not use the gyro: unaffected by bias
rate_b = rate + args.BiasStep;
taus = NaN;
for a = args.Alphas(:)'
    labels{end + 1} = sprintf('comp %.3f', a); %#ok<AGROW>
    ests{end + 1} = complementary_filter(rate, acc_ang, dt, a); %#ok<AGROW>
    ests_b{end + 1} = complementary_filter(rate_b, acc_ang, dt, a); %#ok<AGROW>
    taus(end + 1) = a * median(dt) / (1 - a); %#ok<AGROW>
end
for q = args.QAngles(:)'
    labels{end + 1} = sprintf('KF Q %.0e', q); %#ok<AGROW>
    ests{end + 1} = kalman_tilt(rate, acc_ang, dt, q, args.QBias, args.R); %#ok<AGROW>
    ests_b{end + 1} = kalman_tilt(rate_b, acc_ang, dt, q, args.QBias, args.R); %#ok<AGROW>
    taus(end + 1) = NaN; %#ok<AGROW>
end

% ---------------- metrics ----------------
nest = numel(ests);
rows = cell(nest, 8);
rms_m = zeros(nest, 1);
settle = zeros(nest, 1);
for j = 1:nest
    e = wrap180(ests{j} - ref);
    rms_m(j) = sqrt(mean(e(act) .^ 2));
    max_m = max(abs(e(act)));
    noise = std(ests{j}(w0));
    bad = find(abs(e(after)) > args.Tol, 1, 'last');
    if isempty(bad)
        settle(j) = 0;
    elseif bad == numel(after)
        settle(j) = NaN;                 % never settles within Tol before the end
    else
        settle(j) = t(after(bad + 1)) - t(i_last);
    end
    end_err = wrap180(mean(ests{j}(w1)) - mean(ests{j}(w0)));
    bias_err = mean(wrap180(ests_b{j}(w1) - ref(w1))) - mean(e(w1));
    if isnan(taus(j)), tau_txt = '-'; else, tau_txt = sprintf('%.2f', taus(j)); end
    rows(j, :) = {labels{j}, tau_txt, rms_m(j), max_m, noise, settle(j), end_err, bias_err};
end

% ---------------- print ----------------
fprintf('\nFilter comparison during motion: %.1f s recording, moving %.1f-%.1f s, peak rate %.0f deg/s\n', ...
        t(end), t(i_first), t(i_last), max(abs(rate)));
fprintf('Reference = gyro-only angle with %+.2f deg linear drift correction over %.0f s.\n', drift, t1 - t0);
if abs(drift) > 1.0
    fprintf(['WARNING: drift correction > 1 deg. Either the cube was not put back exactly against the ' ...
             'stop, or it was twisted a lot about other axes. Treat the motion numbers with care.\n']);
end
print_table({'estimator', 'tau s', 'motion RMS', 'motion max', 'still noise', 'settle s', ...
             'end err', 'bias test'}, rows, 12);
fprintf('(tau = complementary time constant; bias test = extra end error with the gyro bias off by %.1f deg/s)\n', ...
        args.BiasStep);

cand = 2:nest;
ok = cand(~isnan(settle(cand)) & settle(cand) <= 1.0);
if ~isempty(ok)
    [~, k] = min(rms_m(ok));
    fprintf('Lowest motion RMS that settles within 1 s: %s (%.2f deg)\n', labels{ok(k)}, rms_m(ok(k)));
end

% ---------------- plot ----------------
fig = new_fig(args, 1100, 700);
ax1 = subplot(2, 1, 1, 'Parent', fig);
hold(ax1, 'on');
plot(ax1, t, acc_ang, 'Color', [0.65 0.65 0.65], 'LineWidth', 0.5, 'DisplayName', 'accel');
plot(ax1, t, ref, 'k', 'LineWidth', 1.5, 'DisplayName', 'reference (drift-corrected gyro)');
for j = 2:nest
    plot(ax1, t, ests{j}, 'LineWidth', 1.0, 'DisplayName', labels{j});
end
ylabel(ax1, 'angle [deg]');
title(ax1, 'Tilt estimates during motion');
legend(ax1, 'show', 'Location', 'best', 'Interpreter', 'none');

ax2 = subplot(2, 1, 2, 'Parent', fig);
hold(ax2, 'on');
for j = 2:nest
    plot(ax2, t, wrap180(ests{j} - ref), 'LineWidth', 1.0, 'DisplayName', labels{j});
end
yline(ax2, args.Tol, 'k:', 'HandleVisibility', 'off');
yline(ax2, -args.Tol, 'k:', 'HandleVisibility', 'off');
ylabel(ax2, 'error vs reference [deg]');
xlabel(ax2, 'time [s]');
legend(ax2, 'show', 'Location', 'best', 'Interpreter', 'none');

linkaxes([ax1 ax2], 'x');
xlim(ax1, [t(max(1, i_first - n)), t(min(N, i_last + 2 * n))]);
save_fig(fig, args, 'test5_metrics.png');
end
