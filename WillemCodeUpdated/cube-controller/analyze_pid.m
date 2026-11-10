function R = analyze_pid(csvFile, varargin)
%ANALYZE_PID  Analyse a cube-controller telemetry CSV (from log_udp.py) and tune kp, kd, kpw, kdw.
%
%   analyze_pid                          newest log in logs/ that has data
%   analyze_pid('logs/cube_xxx.csv')
%   analyze_pid(file, 'Optimize', false) skip the gain search (much faster)
%   analyze_pid(file, 'SaveDir', 'figs') also save the figures as PNG
%   analyze_pid(file, 'Th0Deg', [1 2])   initial tilts (deg) used for the gain search
%   analyze_pid(file, 'LogGains', [300 40 0.009 0.02])
%                                        the [kp kd kpw kdw] that were in the firmware when the log was recorded
%                                        (default: the values in parameters.h now)
%
% What it does
%   1. Plots the recorded response: tilt, rate, wheel speed, torque, with the arming window, the torque limit
%      (Km*ia_max) and the wheel speed limit marked.
%   2. Splits the torque into the four controller terms (kp, kd, kpw, kdw) so you can see which one dominates.
%      The firmware has no integral term: the controller is a PD on the cube plus a PD on the wheel.
%   3. Fits the 1D plant (gravity stiffness and 1/inertia) from the data and checks the sign of the wheel torque.
%   4. Simulates the closed loop with the real limits (torque saturation, wheel speed limit, 250 Hz) and searches
%      for the gains that recover from a small tilt best. An LQR design is shown for reference.
%
% Gains, motor and cube parameters are read from src/definitions/parameters.h and cube-controller.ino, so the
% analysis always uses what is currently in the firmware.
%
% The tuning is model based: it is only as good as the 1D model (see the printed fit). Treat the suggested gains
% as a starting point and test with a hand on the cube and the ESCON power within reach.

if nargin >= 1 && ischar(csvFile) && any(strcmpi(csvFile, {'Optimize', 'SaveDir', 'Visible', 'Th0Deg', 'LogGains'}))
    varargin = [{csvFile}, varargin];     % called with options only: use the newest log
    csvFile = '';
end
p = inputParser;
addParameter(p, 'Optimize', true, @islogical);
addParameter(p, 'SaveDir', '', @ischar);
addParameter(p, 'Visible', true, @islogical);
addParameter(p, 'Th0Deg', [], @isnumeric);
addParameter(p, 'LogGains', [], @isnumeric);
parse(p, varargin{:});
opt = p.Results;

thisDir = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(csvFile)
    csvFile = newest_log(thisDir);
end
fprintf('Log file : %s\n', csvFile);

P = read_params(thisDir);
fprintf(['Gains in firmware: kp=%g  kd=%g  kpw=%g  kdw=%g   (%s controller)\n' ...
         'Motor limit      : ia_max=%g A -> max torque %.3f Nm | wheel speed limit %d rpm\n'], ...
        P.kp, P.kd, P.kpw, P.kdw, tern(P.nonlin, 'nonlinear', 'linear'), P.iaMax, P.tauMax, P.maxRpm);

%% ---- load and prepare the data ----------------------------------------------------------------------------
T = readtable(csvFile);
need = {'t_us','qw','qx','qy','qz','gx_dps','phi_deg','wheel_rpm','wheel_rad','tau_Nm','armed','halted'};
missing = setdiff(need, T.Properties.VariableNames);
if ~isempty(missing)
    error('analyze_pid:columns', 'CSV is missing columns: %s', strjoin(missing, ', '));
end
T = T(all(isfinite(T{:, need}), 2), :);
reset = find(diff(T.t_us) <= 0, 1, 'last');
if ~isempty(reset)
    warning('t_us went backwards (ESP32 rebooted). Using the data after the last reboot.');
    T = T(reset+1:end, :);
end
if height(T) < 50, error('analyze_pid:short', 'Not enough rows in %s', csvFile); end

t     = (T.t_us - T.t_us(1)) / 1e6;
armed = T.armed > 0.5;
wx    = T.gx_dps * pi/180;                    % cube rate about x [rad/s]
wRad  = T.wheel_rad;                          % wheel angle estimate [rad]
wW    = T.wheel_rpm * pi/30;                  % wheel speed estimate [rad/s]
tau   = T.tau_Nm;                             % torque the controller asked for (not clamped) [Nm]
tauA  = max(min(tau, P.tauMax), -P.tauMax);   % torque the motor can actually deliver [Nm]

% Signed tilt from the logged quaternion, with the same error quaternion as the firmware
qr = P.qr;
q0 = T.qw; q1 = T.qx; q2 = T.qy; q3 = T.qz;
qe0 = q0*qr(1) + q1*qr(2) + q2*qr(3) + q3*qr(4);
qe1 = q0*qr(2) - q1*qr(1) - q2*qr(4) + q3*qr(3);
qe2 = q0*qr(3) - q2*qr(1) + q1*qr(4) - q3*qr(2);
qe3 = q0*qr(4) - q1*qr(3) + q2*qr(2) - q3*qr(1);
nrm = sqrt(qe0.^2 + qe1.^2 + qe2.^2 + qe3.^2);
qe0 = qe0./nrm; qe1 = qe1./nrm;
sgn = sign(qe0); sgn(sgn == 0) = 1;
eX  = 2*asin(max(min(qe1.*sgn, 1), -1));      % error angle about x (reference - actual) [rad], like the firmware
theta = -eX;                                  % tilt relative to the balance point [rad]
phiRec = 2*acosd(min(abs(qe0), 1));
ok = T.phi_deg < 90;
dphi = max(abs(phiRec(ok) - T.phi_deg(ok)));
if dphi > 1
    warning(['Reconstructed error angle differs from the logged phi_deg by up to %.1f deg. The reference ' ...
             'quaternion in parameters.h may not match the one used when this log was recorded.'], dphi);
end

%% ---- armed episodes ---------------------------------------------------------------------------------------
ep = find_runs(armed, t, 0.25);
fprintf('\nArmed episodes: %d\n', size(ep, 1));
if isempty(ep)
    fprintf('The controller never armed in this log (phi never went below %g deg), so there is no controlled\n', P.phiMinDeg);
    fprintf('response to analyse. Showing the recording only.\n');
else
    fprintf('  #   start[s]  length[s]  max|tilt|[deg]  peak|tau|[Nm]  >limit[%%]  peak|rpm|  ended\n');
    for k = 1:size(ep, 1)
        r = ep(k,1):ep(k,2);
        endTxt = tern(T.halted(min(ep(k,2)+5, height(T))) > 0.5, 'halted (tilt > limit)', 'disarmed/ongoing');
        fprintf('  %-3d %8.2f  %9.2f  %14.1f  %13.3f  %9.0f  %9.0f  %s\n', k, t(ep(k,1)), t(ep(k,2)) - t(ep(k,1)), ...
            max(abs(theta(r)))*180/pi, max(abs(tau(r))), 100*mean(abs(tau(r)) > P.tauMax), max(abs(T.wheel_rpm(r))), endTxt);
    end
    [~, longest] = max(diff(ep, 1, 2));
    sel = ep(longest, 1):ep(longest, 2);
end

%% ---- torque breakdown (which gain produces the torque) ----------------------------------------------------
Kcur = [P.kp P.kd P.kpw P.kdw];
if isempty(opt.LogGains), Lg = Kcur; else, Lg = opt.LogGains(:).'; end
uP  =  Lg(1) * eX;                 % firmware: u = kp*theta_e + kd*omega_e - kpw*theta_w - kdw*omega_w
uD  =  Lg(2) * (-wx);
uWt = -Lg(3) * wRad;
uWw = -Lg(4) * wW;
tauP = -P.Ib*uP; tauD = -P.Ib*uD; tauWt = -P.Ib*uWt; tauWw = -P.Ib*uWw;
tauSum = tauP + tauD + tauWt + tauWw;
% what the firmware adds on top in the nonlinear controller: gravity and wheel friction compensation
tauComp = P.nonlin * (P.G*theta + sign(wW) .* (P.tauC + P.bw*abs(wW)));
tauRec = tauSum + tauComp;         % reconstructed controller output
tauOther = tau - tauSum;           % everything that is not one of the four terms
if ~isempty(ep)
    ia = false(height(T), 1);
    for k = 1:size(ep, 1), ia(ep(k,1):ep(k,2)) = true; end
    relErr = norm(tau(ia) - tauRec(ia)) / max(norm(tau(ia)), eps);
    fprintf('\nGains assumed for this log: kp=%g kd=%g kpw=%g kdw=%g%s\n', Lg, ...
        tern(isempty(opt.LogGains), '  (from parameters.h)', '  (LogGains)'));
    fprintf('Reconstructed torque vs logged torque: %.0f%% relative error\n', 100*relErr);
    if relErr > 0.3
        fprintf(['>> They do not match: this log was probably recorded with different gains than the ones assumed.\n' ...
                 '   The torque breakdown below is only meaningful with the right gains: pass them as\n' ...
                 '   analyze_pid(file, ''LogGains'', [kp kd kpw kdw]).\n']);
    end
end

%% ---- plant fit (is the torque sign right? what is the gravity stiffness?) ---------------------------------
fit = struct('ok', false);
if ~isempty(ep)
    idx = false(height(T), 1);
    for k = 1:size(ep, 1), idx(ep(k,1):ep(k,2)) = true; end
    acc = gradient(movmean(wx, 5), t);
    if nnz(idx) > 100
        A = [theta(idx), tauA(idx)];
        y = acc(idx);
        c = A \ y;
        yhat = A*c;
        fit.ok = true; fit.a = c(1); fit.b = c(2);
        fit.R2 = 1 - sum((y - yhat).^2)/sum((y - mean(y)).^2);
        fit.cond = cond(A ./ vecnorm(A));
        fit.Ib = -1/c(2); fit.G = c(1)*fit.Ib;
        fit.y = y; fit.yhat = yhat;
        fprintf('\nPlant fit on armed data:  theta_ddot = a*theta + b*tau\n');
        fprintf('  a = %.1f 1/s^2   (model: %.1f)   b = %.1f 1/(kg m^2)   (model: %.1f)   R^2 = %.2f   cond = %.0f\n', ...
            c(1), P.G/P.Ib, c(2), -1/P.Ib, fit.R2, fit.cond);
        if fit.R2 < 0.3 || fit.cond > 30
            fprintf(['  Inconclusive: the fit explains too little of the motion (the torque is saturated or tied to the\n' ...
                     '  tilt), so it says nothing about the torque sign. Use a log with a clean, unsaturated response.\n']);
        elseif c(2) > 0
            fprintf('  >> b is POSITIVE: a positive tau accelerates the cube in the same direction as the tilt. That is\n');
            fprintf('     the opposite of the model, so the torque (or the IMU x axis) sign may be inverted.\n');
        else
            fprintf('  b is negative as the model expects: the torque sign looks consistent.\n');
        end
    end
end

%% ---- figures: recorded response ---------------------------------------------------------------------------
vis = tern(opt.Visible, 'on', 'off');
f1 = figure('Name', 'Recorded response', 'Visible', vis, 'Position', [40 40 1000 820]);
tl = tiledlayout(f1, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('%s   (gains assumed: kp=%g kd=%g kpw=%g kdw=%g)', strrep(csvName(csvFile), '_', '\_'), Lg));
ax(1) = nexttile(tl); hold on;
plot(t, theta*180/pi, 'b', 'DisplayName', 'tilt (signed)'); plot(t, T.phi_deg, 'Color', [.6 .6 .6], 'DisplayName', '|error angle| phi');
yline(P.phiMinDeg, 'g--', 'DisplayName', 'arming window'); yline(-P.phiMinDeg, 'g--', 'HandleVisibility', 'off');
yline(P.phiMaxDeg, 'r--', 'DisplayName', 'trip limit'); yline(-P.phiMaxDeg, 'r--', 'HandleVisibility', 'off');
ylabel('tilt [deg]'); grid on; shade(ax(1), t, armed); legend('Location', 'best');
ax(2) = nexttile(tl); hold on; plot(t, T.gx_dps, 'b'); ylabel('gyro x [deg/s]'); grid on; shade(ax(2), t, armed);
ax(3) = nexttile(tl); hold on; plot(t, T.wheel_rpm, 'm');
yline(P.maxRpm, 'r--'); yline(-P.maxRpm, 'r--'); ylabel('wheel speed [rpm]'); grid on; shade(ax(3), t, armed);
ax(4) = nexttile(tl); hold on;
plot(t, tau, 'Color', [.8 .4 .4], 'DisplayName', 'controller output'); plot(t, tauA, 'k', 'DisplayName', 'motor can deliver');
yline(P.tauMax, 'r--', 'DisplayName', 'torque limit'); yline(-P.tauMax, 'r--', 'HandleVisibility', 'off');
ylabel('torque [Nm]'); xlabel('time [s]'); grid on; shade(ax(4), t, armed); legend('Location', 'best');
linkaxes(ax, 'x'); xlim(ax(1), [t(1) t(end)]);

if ~isempty(ep)
    f2 = figure('Name', 'Torque by gain', 'Visible', vis, 'Position', [60 60 1000 600]);
    tl2 = tiledlayout(f2, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl2, sprintf('Where the torque comes from (armed episode %d)', longest));
    a1 = nexttile(tl2); hold on;
    plot(t(sel), tauP(sel), 'DisplayName', 'kp  (tilt)'); plot(t(sel), tauD(sel), 'DisplayName', 'kd  (tilt rate)');
    plot(t(sel), tauWt(sel), 'DisplayName', 'kpw (wheel angle)'); plot(t(sel), tauWw(sel), 'DisplayName', 'kdw (wheel speed)');
    ylabel('torque term [Nm]'); legend('Location', 'best'); grid on;
    a2 = nexttile(tl2); hold on;
    plot(t(sel), tau(sel), 'k', 'LineWidth', 1.2, 'DisplayName', 'logged torque');
    plot(t(sel), tauRec(sel), 'r--', 'DisplayName', 'reconstructed from the gains');
    plot(t(sel), tauSum(sel), 'c:', 'DisplayName', 'sum of the 4 terms only');
    yline(P.tauMax, 'r:', 'HandleVisibility', 'off'); yline(-P.tauMax, 'r:', 'HandleVisibility', 'off');
    ylabel('torque [Nm]'); xlabel('time [s]'); legend('Location', 'best'); grid on;
    linkaxes([a1 a2], 'x');
    fprintf('\nPeak torque contribution in episode %d [Nm]:  kp %.3f | kd %.3f | kpw %.3f | kdw %.3f   (limit %.3f)\n', ...
        longest, max(abs(tauP(sel))), max(abs(tauD(sel))), max(abs(tauWt(sel))), max(abs(tauWw(sel))), P.tauMax);

    if fit.ok
        f3 = figure('Name', 'Plant fit', 'Visible', vis, 'Position', [80 80 560 480]);
        plot(fit.yhat, fit.y, '.', 'MarkerSize', 4); hold on; grid on;
        lim = max(abs([fit.y; fit.yhat])); plot([-lim lim], [-lim lim], 'r--');
        xlabel('model-predicted theta\_ddot [rad/s^2]'); ylabel('measured theta\_ddot [rad/s^2]');
        title(sprintf('R^2 = %.2f,  b = %.1f (model %.1f)', fit.R2, fit.b, -1/P.Ib));
    end
end

%% ---- model-based gain tuning ------------------------------------------------------------------------------
R = struct('params', P, 'episodes', ep, 'fit', fit);
fprintf('\n--- Model based tuning (1D reaction-wheel pendulum, with torque and speed limits) ---\n');
fprintf('Gravity stiffness G = %.3f Nm/rad  (open-loop instability sqrt(G/Ib) = %.1f rad/s)\n', P.G, sqrt(P.G/P.Ib));
fprintf('Largest tilt the motor can hold even in the best case: tau_max/G = %.1f deg\n', P.tauMax/P.G*180/pi);
for ia = [2 4 6]
    fprintf('    with ia_max = %g A: %.1f deg\n', ia, P.Km*ia/P.G*180/pi);
end
tiltRec = P.tauMax/P.G*180/pi;
if tiltRec < P.phiMinDeg
    fprintf('>> The controller arms at %g deg but the motor can only hold about %.1f deg. Expect it to saturate and\n', P.phiMinDeg, tiltRec);
    fprintf('   fall after arming, whatever the gains. Raise ia_max (ESCON + parameters.h) or arm closer to the balance point.\n');
end

[Am, Bm] = plant_matrices(P);
pc = eig(Am - Bm*Kcur);
fprintf('Closed-loop poles with the current gains (unsaturated): %s\n', mat2str(round(pc.', 2)));
if any(real(pc) >= 0), fprintf('>> The current gains are UNSTABLE in the model even before saturation.\n'); end

thMax = 1*pi/180; thdMax = 10*pi/180; thwMax = 5*2*pi; wwMax = P.maxRpm*pi/30; uMax = P.tauMax/P.Ib;
Q = diag(1./[thMax thdMax thwMax wwMax].^2); Rr = 1/uMax^2;
Klqr = lqr_gains(Am, Bm, Q, Rr);
fprintf('LQR reference (Bryson weights): kp=%.3g kd=%.3g kpw=%.3g kdw=%.3g\n', Klqr);

cfg.T = 3;
if isempty(opt.Th0Deg)
    th0 = unique(round(min(3, tiltRec) * [0.2 0.4 0.6], 2));
else
    th0 = opt.Th0Deg(:).';
end
fprintf('Simulated initial tilts: %s deg (from rest, %g s)\n', mat2str(th0), cfg.T);

Kbest = Kcur; Jbest = cost_fn(log(Kcur), th0, P, cfg);
Jcur = Jbest; Jl = cost_fn(log(Klqr), th0, P, cfg);
if opt.Optimize
    fprintf('Searching for the best gains (this takes a minute)...\n');
    starts = [Kcur; Klqr; Kcur.*[2 2 1 1]; Kcur.*[0.5 0.5 1 1]; Kcur.*[1 1 5 5]];
    starts = max(starts, 1e-5);
    fo = optimset('Display', 'off', 'MaxFunEvals', 1200, 'MaxIter', 600, 'TolX', 1e-3, 'TolFun', 1e-4);
    for s = 1:size(starts, 1)
        [lg, J] = fminsearch(@(x) cost_fn(x, th0, P, cfg), log(starts(s, :)), fo);
        if J < Jbest, Jbest = J; Kbest = exp(lg); end
    end
end

names = {'current gains', 'LQR reference', 'best found'};
Ks = [Kcur; Klqr; Kbest]; Js = [Jcur; Jl; Jbest];
fprintf('\n%-14s %9s %9s %9s %9s %9s   %s\n', '', 'kp', 'kd', 'kpw', 'kdw', 'cost', 'balances all tilts?');
for k = 1:3
    fell = false;
    for th = th0
        [~, ~, ~, inf_] = sim_closed_loop(Ks(k,:), th*pi/180, P, cfg);
        fell = fell || inf_.fell;
    end
    fprintf('%-14s %9.4g %9.4g %9.4g %9.4g %9.3g   %s\n', names{k}, Ks(k,:), Js(k), tern(fell, 'NO (falls)', 'yes'));
end

f4 = figure('Name', 'Simulated recovery', 'Visible', vis, 'Position', [100 100 340*numel(th0)+60 780]);
tl4 = tiledlayout(f4, 3, numel(th0), 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl4, 'Simulated recovery from a tilt (model, with torque and speed limits)');
cols = {[0.85 0.33 0.10], [0.47 0.67 0.19], [0 0.45 0.74]};
for j = 1:numel(th0)
    a = nexttile(tl4, j); hold on; b = nexttile(tl4, numel(th0)+j); hold on; c = nexttile(tl4, 2*numel(th0)+j); hold on;
    for k = 1:3
        [ts, X, ta] = sim_closed_loop(Ks(k,:), th0(j)*pi/180, P, cfg);
        tt = (0:size(X,1)-1)'/P.f;
        plot(a, tt, X(:,1)*180/pi, 'Color', cols{k}, 'DisplayName', names{k});
        plot(b, tt, X(:,4)*30/pi, 'Color', cols{k});
        plot(c, tt, ta, 'Color', cols{k});
    end
    yline(b, P.maxRpm, 'r--'); yline(b, -P.maxRpm, 'r--'); yline(c, P.tauMax, 'r--'); yline(c, -P.tauMax, 'r--');
    title(a, sprintf('start %.2g deg', th0(j))); ylabel(a, 'tilt [deg]'); ylabel(b, 'wheel [rpm]'); ylabel(c, 'torque [Nm]');
    xlabel(c, 'time [s]'); grid(a, 'on'); grid(b, 'on'); grid(c, 'on');
    if j == 1, legend(a, 'Location', 'best'); end
end

R.Kcurrent = Kcur; R.Klqr = Klqr; R.Kbest = Kbest; R.cost = Js; R.th0 = th0;
fprintf('\nSuggested values for src/definitions/parameters.h (model based, test carefully):\n');
fprintf('  const float kp  = %.6g;\n  const float kd  = %.6g;\n  const float kpw = %.6g;\n  const float kdw = %.6g;\n', Kbest);
fprintf(['Caveats: 1D model, linear gravity, one sample of delay, no sensor noise. High gains amplify IMU noise, so\n' ...
         'move toward them in steps. The wheel speed signal must be trustworthy\n' ...
         'first (see wheel_rpm at rest in the log), and the ESCON current range must match ia_max.\n']);

if ~isempty(opt.SaveDir)
    if ~exist(opt.SaveDir, 'dir'), mkdir(opt.SaveDir); end
    figs = findobj('Type', 'figure');
    for k = 1:numel(figs)
        exportgraphics(figs(k), fullfile(opt.SaveDir, [regexprep(figs(k).Name, '\W', '_') '.png']), 'Resolution', 110);
    end
    fprintf('Figures saved in %s\n', opt.SaveDir);
end
end

%% =========================================================================================================
%% helpers
%% =========================================================================================================
function s = tern(c, a, b)
if c, s = a; else, s = b; end
end

function n = csvName(f)
[~, n, e] = fileparts(f); n = [n e];
end

function f = newest_log(thisDir)
d = dir(fullfile(thisDir, 'logs', 'cube_*.csv'));
d = d([d.bytes] > 20000);
if isempty(d), error('analyze_pid:nolog', 'No log with data found in %s. Pass a CSV path.', fullfile(thisDir, 'logs')); end
[~, i] = max([d.datenum]);
f = fullfile(d(i).folder, d(i).name);
end

function ep = find_runs(mask, t, gap)
% [first last] row indices of runs of true, merging runs separated by less than gap seconds
mask = mask(:);
d = diff([0; mask; 0]);
s = find(d == 1); e = find(d == -1) - 1;
ep = zeros(0, 2);
for k = 1:numel(s)
    if ~isempty(ep) && t(s(k)) - t(ep(end,2)) < gap
        ep(end,2) = e(k);
    else
        ep(end+1, :) = [s(k) e(k)]; %#ok<AGROW>
    end
end
end

function shade(ax, t, mask)
% light green background where the controller is armed (drawn after the data so the y range is the data's)
ep = find_runs(mask, t, 0);
yl = ylim(ax);
for k = 1:size(ep, 1)
    pt = patch(ax, t(ep(k,[1 2 2 1])), yl([1 1 2 2]), [0.85 1 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.6, ...
        'HandleVisibility', tern(k == 1, 'on', 'off'), 'DisplayName', 'armed');
    uistack(pt, 'bottom');
end
ylim(ax, yl);
end

function v = tok_num(tok, def)
if isempty(tok), v = def; else, v = str2double(tok{1}); end
end

function P = read_params(thisDir)
txt = fileread(fullfile(thisDir, 'src', 'definitions', 'parameters.h'));
txt = regexprep(txt, '//[^\n]*', '');
num = @(name, def) get_num(txt, name, def);
P.kp = num('kp', 6); P.kd = num('kd', 0.8); P.kpw = num('kpw', 2e-4); P.kdw = num('kdw', 4e-4);
P.Km = num('Km', 36e-3); P.iaMax = num('ia_max', 2);
P.tauC = num('tau_c', 3.4e-3); P.bw = num('bw', 1.513e-5);
P.f = num('f', 250);
ms = num('m_s', 1); mw = num('m_w', 0.225); l = num('l', 0.15);
Isxx = num('I_s_xx', 3e-3); Iwxx = num('I_w_xx', 4.855e-4); Iwyy = num('I_w_yy', 2.439e-4);
P.Iw = Iwxx;
P.Ib = Isxx + 2*Iwyy + (ms + 2*mw)*l^2/2;               % I_c_xx_bar in parameters.h
P.mcbarGl = (ms + 3*mw - mw) * 9.80665 * l;             % m_c_bar_g_l in parameters.h
P.tauMax = P.Km * P.iaMax;                              % Motor::set_current clamps the current to +/- ia_max

phiE = regexp(txt, 'const\s+float\s+phi_e\s*=\s*([-+]?[0-9.]+)\s*\*\s*pi\s*/\s*180', 'tokens', 'once');
phiE = tok_num(phiE, -3) * pi/180;
P.qr = [cos(phiE/2 - pi/8), cos(phiE/2 + 3*pi/8), 0, 0];      % x-edge reference used when generate() is commented out
if ~isempty(regexp(txt, 'const\s+float\s+qu1\s*=\s*0\.0\s*;', 'once'))
    warning('parameters.h has the y-edge reference active; this script assumes the x-edge reference.');
end
phiMin = regexp(txt, 'phi_min\s*=\s*([0-9.]+)\s*\*\s*pi', 'tokens', 'once');
phiMax = regexp(txt, 'phi_max\s*=\s*([0-9.]+)\s*\*\s*pi', 'tokens', 'once');
P.phiMinDeg = tok_num(phiMin, 5);
P.phiMaxDeg = tok_num(phiMax, 40);

% Gravity stiffness: derivative of the firmware's gravity term m_c_bar_g_l*(0.5 - q0^2 + q0*q1) for a rotation about x
g1 = @(d) P.mcbarGl * (0.5 - cos(phiE/2 - pi/8 + d/2).^2 + cos(phiE/2 - pi/8 + d/2) .* (-sin(phiE/2 - pi/8 + d/2)));
h = 1e-4;
P.G = abs((g1(h) - g1(-h)) / (2*h));

ino = fileread(fullfile(thisDir, 'cube-controller.ino'));
ino = regexprep(ino, '//[^\n]*', '');
m = regexp(ino, '#define\s+MAX_WHEEL_RPM\s+(\d+)', 'tokens', 'once');
P.maxRpm = tok_num(m, 3000);
m = regexp(ino, '#define\s+USE_NONLINEAR_CONTROLLER\s+(\d)', 'tokens', 'once');
P.nonlin = tok_num(m, 1) == 1;
end

function v = get_num(txt, name, def)
tok = regexp(txt, ['const\s+float\s+' name '\s*=\s*([-+]?[0-9.]+(?:[eE][-+]?\d+)?)\s*;'], 'tokens', 'once');
if isempty(tok)
    warning('Could not read %s from parameters.h, using %g.', name, def);
    v = def;
else
    v = str2double(tok{1});
end
end

function [A, B] = plant_matrices(P)
% state [theta; theta_dot; theta_w; omega_w], input u (desired cube acceleration), u = -K*x with K = [kp kd kpw kdw]
if P.nonlin
    A = [0 1 0 0; 0 0 0 0; 0 0 0 1; P.G/P.Iw 0 0 0];
else
    A = [0 1 0 0; P.G/P.Ib 0 0 0; 0 0 0 1; 0 0 0 0];
end
B = [0; 1; 0; -P.Ib/P.Iw];
end

function K = lqr_gains(A, B, Q, R)
try
    K = lqr(A, B, Q, R);
    return
catch
end
n = size(A, 1);
Hm = [A, -B*(R\B'); -Q, -A'];
[V, D] = eig(Hm);
sel = real(diag(D)) < 0;
V1 = V(1:n, sel); V2 = V(n+1:end, sel);
Pm = real(V2 / V1);
K = real(R \ (B' * Pm));
end

function [t, X, tauA, info] = sim_closed_loop(g, th0, P, cfg)
% Closed loop of the firmware controller (250 Hz) with the 1D plant, torque clamp and wheel speed limit.
dt = 1/P.f; n = round(cfg.T * P.f); sub = 2; h = dt/sub;
X = zeros(n, 4); tauA = zeros(n, 1);
x = [th0; 0; 0; 0];
info = struct('fell', false, 'tFall', cfg.T, 'limitFrac', 0, 'satFrac', 0);
limitCnt = 0; satCnt = 0; last = n; tauPrev = 0;
for k = 1:n
    u = -(g(1)*x(1) + g(2)*x(2) + g(3)*x(3) + g(4)*x(4));
    w = x(4);
    tf = sign(w) * (P.tauC + P.bw*abs(w));
    tauCmd = -P.Ib*u + P.nonlin * (P.G*x(1) + tf);        % feedback linearization + friction compensation
    tau = min(max(tauCmd, -P.tauMax), P.tauMax);
    if abs(tauCmd) > P.tauMax, satCnt = satCnt + 1; end
    if abs(w*30/pi) > P.maxRpm && tau*w > 0               % wheel speed limit in the firmware
        tau = 0; limitCnt = limitCnt + 1;
    end
    tauUse = tauPrev; tauPrev = tau;                      % one control period of delay (computation + ESCON)
    for s = 1:sub
        acc  = (P.G*x(1) - tauUse) / P.Ib;
        wdot = (tauUse - sign(x(4)) * (P.tauC + P.bw*abs(x(4)))) / P.Iw;
        x = x + h * [x(2); acc; x(4); wdot];
    end
    X(k, :) = x.'; tauA(k) = tauUse;
    if abs(x(1)) > P.phiMaxDeg*pi/180
        info.fell = true; info.tFall = k*dt; last = k; break
    end
end
X = X(1:last, :); tauA = tauA(1:last); t = (1:last)' * dt;
info.limitFrac = limitCnt / n; info.satFrac = satCnt / n;
end

function J = cost_fn(logg, th0list, P, cfg)
g = exp(logg(:)).';
if any(~isfinite(g)) || any(g > 1e5), J = 1e6; return; end
dt = 1/P.f; J = 0;
for th0 = th0list(:).'
    [~, X, tau, info] = sim_closed_loop(g, th0*pi/180, P, cfg);
    thd = X(:,1)*180/pi;
    rel = min(abs(thd)/th0, 20);                         % capped so a fall does not swamp the cost
    J = J + sum(rel.^2)*dt ...                           % tilt error (normalised ISE)
          + 0.5*sum((tau/P.tauMax).^2)*dt ...            % control effort
          + 0.5*sum((X(:,4)*30/pi/P.maxRpm).^2)*dt ...   % wheel speed
          + 20*info.limitFrac + 50*min(abs(thd(end))/th0, 10)^2;   % speed limit hits, not settled at the end
    if info.fell, J = J + 200 + 100*(cfg.T - info.tFall); end
end
end
