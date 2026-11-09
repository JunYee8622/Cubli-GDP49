function motor_analysis(csvFile)
% MOTOR_ANALYSIS  Analyse a cubli_motor_test_wifi log saved by log_udp.py.
%
%   motor_analysis                 % opens a file picker
%   motor_analysis('imu_2026....csv')
%
% Columns used: t_us, marker, en (state), rpm, cmd_A, ref_rpm
% Produces: speed vs reference, current command and PWM duty, tracking error,
%           acceleration-vs-current fit (checks Km/J and ESCON current scaling),
%           stop-time metrics, and sampling-quality statistics.

% ---------------- constants: keep identical to balance_config.h ----------------
IA_MAX_ESCON = 6.0;        % A   PWM 10 % = -IA_MAX, 50 % = 0 A, 90 % = +IA_MAX
PWM_BITS     = 12;         % ESP32 PWM resolution
KM           = 0.0396;     % Nm/A
J            = 5.934e-4;   % kg m^2
SMOOTH_S     = 0.10;       % smoothing window for the acceleration estimate [s]
STOP_RPM     = 50;         % |rpm| below this counts as "stopped"
% ------------------------------------------------------------------------------

if nargin < 1 || isempty(csvFile)
    [f, p] = uigetfile('*.csv', 'Select the log CSV');
    if isequal(f, 0), return; end
    csvFile = fullfile(p, f);
end

T = readtable(csvFile, 'VariableNamingRule', 'preserve');
need = {'t_us', 'rpm', 'cmd_A'};
for k = 1:numel(need)
    if ~ismember(need{k}, T.Properties.VariableNames)
        error('Column "%s" not found. Columns: %s', need{k}, strjoin(T.Properties.VariableNames, ', '));
    end
end

% ---------------- extract + time axis (handles the 32-bit microsecond wrap) -----
tus = double(T.t_us);
wrap = [0; cumsum(diff(tus) < 0)] * 2^32;
t    = (tus + wrap - tus(1)) * 1e-6;                 % s
rpm  = T.rpm;
cmdA = T.cmd_A;
if ismember('ref_rpm', T.Properties.VariableNames), ref = T.ref_rpm; else, ref = nan(size(rpm)); end
if ismember('en', T.Properties.VariableNames),      st  = T.en;      else, st  = nan(size(rpm)); end
N = numel(t);

% ---------------- PWM reconstruction ------------------------------------------
% Same mapping as writeCurrent() in the firmware.
cmdC = max(min(cmdA, IA_MAX_ESCON), -IA_MAX_ESCON);
duty = 0.5 + cmdC * (0.8 / (2 * IA_MAX_ESCON));      % 0..1 (0.1..0.9)
dutyPct = duty * 100;
counts  = floor(duty * (2^PWM_BITS - 1));

% ---------------- sampling quality --------------------------------------------
dt = diff(t);
Ts = median(dt);
fs = 1 / Ts;
missed = sum(round(dt(dt > 1.5 * Ts) / Ts) - 1);
fprintf('\n==== %s ====\n', csvFile);
fprintf('Rows: %d, duration %.1f s, median sample time %.2f ms (%.0f Hz)\n', N, t(end), Ts * 1e3, fs);
fprintf('Gaps (lost samples, UDP/loop): ~%d (%.2f %%)\n', missed, 100 * missed / (N + missed));

% ---------------- smooth speed and estimate acceleration ----------------------
win   = max(3, round(SMOOTH_S * fs));
rpmS  = movmean(rpm, win);
w     = rpmS * (2 * pi / 60);                        % rad/s
alpha = gradient(w, t);                              % rad/s^2
alphaS = movmean(alpha, win);

% ---------------- tracking error ----------------------------------------------
if any(~isnan(ref))
    err = ref - rpm;
    maxRef = max(abs(ref));
    rampM = abs(ref) > 0.10 * maxRef & abs(ref) < 0.95 * maxRef;
    if any(~isnan(st)), rampM = rampM & st == 1; end
    holdM = abs(ref) >= 0.95 * maxRef;
    fprintf('\n-- Tracking (reference vs measured) --\n');
    if any(rampM), fprintf('Ramp: RMSE %.1f rpm, max |err| %.1f rpm, mean err %+.1f rpm\n', ...
            rms(err(rampM)), max(abs(err(rampM))), mean(err(rampM))); end
    if any(holdM), fprintf('Hold: RMSE %.1f rpm, mean err %+.1f rpm, ripple (std of rpm) %.1f rpm\n', ...
            rms(err(holdM)), mean(err(holdM)), std(rpm(holdM))); end
else
    err = nan(size(rpm)); maxRef = NaN;
end

% ---------------- sudden-stop metrics -----------------------------------------
if any(~isnan(ref)) && maxRef > 0
    idx = find(abs(ref(1:end-1)) > 0.5 * maxRef & abs(ref(2:end)) < 1) + 1;
    fprintf('\n-- Sudden stops: %d found --\n', numel(idx));
    for k = 1:numel(idx)
        i0 = idx(k);
        v0 = rpm(i0);
        seg = i0:min(N, i0 + round(10 * fs));
        j = find(abs(rpm(seg)) < STOP_RPM, 1);
        if isempty(j), tStop = NaN; else, tStop = t(seg(j)) - t(i0); end
        [iPk, kPk] = max(abs(cmdA(seg)));
        decel = NaN;
        if ~isnan(tStop) && tStop > 0, decel = abs(v0) / tStop; end
        fprintf('Stop %d at t=%.2f s: from %+.0f rpm, to <%d rpm in %.2f s (avg %.0f rpm/s), peak |cmd| %.2f A at +%.2f s\n', ...
            k, t(i0), v0, STOP_RPM, tStop, decel, iPk, t(seg(kPk)) - t(i0));
    end
end

% ---------------- acceleration vs current fit ---------------------------------
% Model: alpha = a1*i + a2*w + a3*sign(w)   (current torque, viscous drag, Coulomb friction)
% a1 should equal Km/J if the ESCON delivers the commanded current.
mask = abs(w) > 5 & isfinite(alphaS) & (isnan(st) | st ~= 0);
a1 = NaN;
if nnz(mask) > 50
    X = [cmdA(mask), w(mask), sign(w(mask))];
    coef = X \ alphaS(mask);
    a1 = coef(1);
    KmJ = KM / J;
    fprintf('\n-- Acceleration vs current fit --\n');
    fprintf('a1 (rad/s^2 per A)  measured %.1f   expected Km/J %.1f   ratio %.2f\n', a1, KmJ, a1 / KmJ);
    fprintf('a2 viscous drag     %.4f 1/s   (b/J)\n', coef(2));
    fprintf('a3 Coulomb friction %.2f rad/s^2  (= %.4f A equivalent)\n', coef(3), coef(3) / a1);
    fprintf('Implied J if Km is right: %.3e kg m^2 (config: %.3e)\n', KM / a1, J);
    fprintf('Ratio ~1: ESCON scaling and J/Km OK. Ratio ~0.33: ESCON set to +/-2 A while firmware assumes +/-6 A.\n');
end

% ---------------- plots -------------------------------------------------------
figure('Name', 'Motor speed', 'Color', 'w');
ax1 = subplot(4, 1, 1); hold on; grid on;
plot(t, rpm, 'LineWidth', 0.8, 'DisplayName', 'measured');
if any(~isnan(ref)), plot(t, ref, '--', 'LineWidth', 1.2, 'DisplayName', 'reference'); end
ylabel('speed [rpm]'); legend('Location', 'best'); title('Wheel speed');

ax2 = subplot(4, 1, 2); hold on; grid on;
plot(t, cmdA, 'LineWidth', 0.8);
ylabel('current cmd [A]'); title('Current command (signal input to ESCON)');

ax3 = subplot(4, 1, 3); hold on; grid on;
plot(t, dutyPct, 'LineWidth', 0.8);
yline(50, ':'); yline(10, 'r:'); yline(90, 'r:');
ylabel('PWM duty [%]'); title('PWM duty sent to ESCON (50 % = 0 A, 10/90 % = limits)');
ylim([0 100]);

ax4 = subplot(4, 1, 4); hold on; grid on;
if any(~isnan(err)), plot(t, err, 'LineWidth', 0.8); end
if any(~isnan(st)), stairs(t, st * max(1, 0.1 * max(abs(err(~isnan(err)) ))), 'k:', 'DisplayName', 'state'); end
ylabel('error [rpm]'); xlabel('time [s]'); title('Tracking error (reference - measured); dotted = state');
linkaxes([ax1 ax2 ax3 ax4], 'x');

figure('Name', 'Accel vs current', 'Color', 'w');
subplot(1, 2, 1); hold on; grid on;
if nnz(mask) > 50
    scatter(cmdA(mask), alphaS(mask), 6, t(mask), 'filled');
    xx = linspace(min(cmdA(mask)), max(cmdA(mask)), 50);
    plot(xx, a1 * xx, 'r', 'LineWidth', 1.5, 'DisplayName', 'fit a1*i');
    plot(xx, (KM / J) * xx, 'k--', 'LineWidth', 1.2, 'DisplayName', 'ideal Km/J');
    colorbar; legend('Location', 'best');
end
xlabel('current cmd [A]'); ylabel('acceleration [rad/s^2]'); title('Acceleration vs current command');

subplot(1, 2, 2); hold on; grid on;
histogram(dt * 1e3, 60);
xlabel('sample interval [ms]'); ylabel('count'); title(sprintf('Sampling: median %.2f ms, %d gaps', Ts * 1e3, missed));

figure('Name', 'PWM counts', 'Color', 'w');
plot(t, counts); grid on; xlabel('time [s]'); ylabel(sprintf('PWM counts (0..%d)', 2^PWM_BITS - 1));
title('Reconstructed PWM compare value');
end
