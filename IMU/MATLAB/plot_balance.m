function plot_balance(file, varargin)
% PLOT_BALANCE  Diagnose an edge-balancing run recorded by log_udp.py from edge_balance.ino.
%
%   plot_balance('data/bal_1.csv')
%   plot_balance('data/bal_1.csv', 'Axis', 1, 'Imax', 2.0, 'Show', true)
%
% Plots tilt error, body rate, wheel speed and motor current against time, shades the
% balancing periods, and prints one line of numbers per balancing attempt:
%   dur s       how long it balanced
%   end         how it ended: fell / wheel limit / stopped / still balancing
%   rms / max   tilt error while balancing [deg]
%   sat %       share of time the current sat at the clamp (lots = gains too high or wrong theta_eq)
%   rpm drift   average wheel acceleration [rpm/s] (steady drift = theta_eq slightly off)
%   osc Hz      dominant oscillation frequency of the tilt error (buzz vs slow wobble)
%
% Options: Axis (0 = X, 1 = Y, must match TILT_AXIS), Imax (I_MAX_BAL), Out (folder), Show.
p = inputParser;
p.addRequired('file', @(x) ischar(x) || isstring(x));
p.addParameter('Axis', 1, @isscalar);
p.addParameter('Imax', 2.0, @isscalar);
p.addParameter('Out', 'results', @(x) ischar(x) || isstring(x));
p.addParameter('Show', false, @islogical);
p.parse(file, varargin{:});
a = p.Results;

T = readtable(a.file);
tus = double(T.t_us);
d = diff(tus);
d(d < 0) = d(d < 0) + 2^32;                   % micros() wrap
t = [0; cumsum(d)] * 1e-6;
gyrCols = {'gx', 'gy', 'gz'};
rate = T.(gyrCols{a.Axis + 1});
if isfile('calibration.json')
    cal = jsondecode(fileread('calibration.json'));
    if isfield(cal, 'gyro_bias'), rate = rate - cal.gyro_bias(a.Axis + 1); end
end
st = T.en; th = T.th_err; rpm = T.rpm; cmd = T.cmd_A;
fs = 1 / median(diff(t));

% ---------------- balancing episodes (state 2) ----------------
bal = st == 2;
edges = diff([0; bal; 0]);
s0 = find(edges == 1);
s1 = find(edges == -1) - 1;
fprintf('\n%s: %.1f s recorded, %d balancing attempt(s)\n', a.file, t(end), numel(s0));
fprintf('%4s %8s %8s %12s %8s %8s %8s %11s %8s\n', '#', 'start s', 'dur s', 'end', 'rms deg', ...
        'max deg', 'sat %', 'rpm drift', 'osc Hz');
for k = 1:numel(s0)
    i = s0(k):s1(k);
    if s1(k) < numel(st)
        nxt = st(s1(k) + 1);
        if nxt == 4 && abs(th(s1(k) + 1)) > 15, how = 'fell';
        elseif nxt == 4 && abs(rpm(s1(k) + 1)) > 2500, how = 'wheel limit';
        else, how = 'stopped';
        end
    else
        how = 'running';
    end
    dur = t(s1(k)) - t(s0(k));
    sat = 100 * mean(abs(cmd(i)) >= 0.98 * a.Imax);
    if numel(i) > 10
        pf = polyfit(t(i), rpm(i), 1); drift = pf(1);
    else
        drift = NaN;
    end
    fosc = NaN;
    if dur > 2
        x = th(i) - mean(th(i));
        M = numel(x);
        X = abs(fft(x .* (0.5 - 0.5 * cos(2 * pi * (0:M - 1)' / (M - 1)))));
        f = (0:M - 1)' * fs / M;
        m = f > 0.3 & f < fs / 2;
        [~, j] = max(X(m)); fm = f(m); fosc = fm(j);
    end
    fprintf('%4d %8.1f %8.2f %12s %8.2f %8.2f %8.1f %11.0f %8.2f\n', k, t(s0(k)), dur, how, ...
            sqrt(mean(th(i) .^ 2)), max(abs(th(i))), sat, drift, fosc);
end
fprintf(['\nReading it:\n' ...
         '  falls within ~0.5 s, wheel spinning the "wrong" way   -> TILT_SIGN / sign test\n' ...
         '  sat %% high and osc > ~5 Hz (buzz)                     -> gains too high: scale K by 0.7-0.8\n' ...
         '  slow growing wobble, osc ~0.5-3 Hz                     -> more damping: K_RATE x1.3-1.5\n' ...
         '  steady rpm drift, ends at the wheel limit              -> theta_eq off: + or - (sign of drift)\n' ...
         '  balances but leans / th_err mean not 0                 -> fine, the wheel-speed term handles it\n']);

% ---------------- plot ----------------
fig = figure('Name', 'Balance run', 'Visible', 'on', 'Position', [100 100 1100 800]);
labels = {'tilt error [deg]', 'body rate [deg/s]', 'wheel [rpm]', 'current [A]'};
data = {th, rate, rpm, cmd};
ax = gobjects(4, 1);
for k = 1:4
    ax(k) = subplot(4, 1, k);
    hold(ax(k), 'on');
    yl = [min(data{k}) max(data{k})];
    if yl(1) == yl(2), yl = yl + [-1 1]; end
    for e = 1:numel(s0)
        patch(ax(k), [t(s0(e)) t(s1(e)) t(s1(e)) t(s0(e))], [yl(1) yl(1) yl(2) yl(2)], ...
              [0.85 0.95 0.85], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    plot(ax(k), t, data{k}, 'LineWidth', 0.8);
    ylabel(ax(k), labels{k});
    grid(ax(k), 'on');
end
yline(ax(4), a.Imax, 'r:'); yline(ax(4), -a.Imax, 'r:');
yline(ax(1), 0, 'k:');
title(ax(1), 'Edge balancing (green = BALANCING)');
xlabel(ax(4), 'time [s]');
linkaxes(ax, 'x');
xlim(ax(1), [t(1) t(end)]);
if ~isfolder(a.Out), mkdir(a.Out); end
[~, name] = fileparts(char(a.file));
out = fullfile(a.Out, [name '_balance.png']);
exportgraphics(fig, out, 'Resolution', 150);
fprintf('plot saved: %s\n', out);
if ~a.Show, close(fig); end
end
