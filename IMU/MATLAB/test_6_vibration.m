function test_6_vibration(file, varargin)
% TEST_6_VIBRATION  Reaction-wheel vibration.
%
%   test_6_vibration('data/vib_*.csv', 'Speeds', [0 1000 2000 3000 4000])
%
% Protocol: clamp the cube still (or rest it on an edge against a stop). Step the wheel speed up,
% holding each speed ~5 s, and press Enter at every speed change. Note the speeds in order and
% pass them with 'Speeds' (optional, just for the table labels).
%
% The cube isn't moving, so any change in the angle estimates is vibration getting through.
% Output: per-speed noise of each estimate, dominant vibration frequency, and a spectrogram.
%
% Options: Speeds (label per segment), Trim (seconds trimmed after each speed change).
p = common_args();
p.addParameter('Speeds', [], @(x) true);
p.addParameter('Trim', 0.5, @isscalar);
p.parse(file, varargin{:});
args = p.Results;

lg = load_log(args.file, args.Skip);
cal = load_cal(args.Cal);
est = run_filters(lg, cal, args);
acc = apply_cal(lg, cal);
anorm = vecnorm(acc, 2, 2);
trim = floor(args.Trim * lg.fs);
speeds = string(args.Speeds);

segs = marker_segments(lg.marker);
rows = {};
for i = 1:size(segs, 1)
    i0 = segs(i, 1);
    i1 = segs(i, 2);
    if (i1 - i0 + 1) - trim < lg.fs  % under 1 s
        continue
    end
    s = i0 + trim:i1;
    if i <= numel(speeds), label = char(speeds(i)); else, label = sprintf('%d', i); end
    rows(end + 1, :) = {label, std(est.accel(s), 1), std(est.comp(s), 1), std(est.kf(s), 1), ...
                        std(est.rate(s), 1), std(anorm(s), 1), dominant_freq(anorm(s), lg.fs)}; %#ok<AGROW>
end
fprintf('\nNoise (std) per wheel speed - the cube is still, so this is all vibration\n');
print_table({'speed', 'accel deg', 'comp deg', 'kf deg', 'rate dps', '|a| m/s2', 'peak Hz'}, rows, 10);
fprintf(['If comp/kf std jumps at certain speeds, look at the peak frequency: lower the ICM-20948 DLPF,\n' ...
         'add a notch at that frequency, or fix the mechanical source (wheel balance, loose mount).\n']);

fig = new_fig(args, 1000, 700);
ax1 = subplot(2, 1, 1, 'Parent', fig);
hold(ax1, 'on');
plot(ax1, lg.t, est.accel, 'LineWidth', 0.5, 'Color', [0.12 0.47 0.71 0.6], 'DisplayName', 'accel');
plot(ax1, lg.t, est.comp, 'DisplayName', 'complementary');
plot(ax1, lg.t, est.kf, 'DisplayName', 'Kalman');
ylabel(ax1, 'angle [deg]');
legend(ax1, 'show');
ax2 = subplot(2, 1, 2, 'Parent', fig);
[S, f, tt] = stft_db(anorm - mean(anorm), lg.fs, 256, 128);
imagesc(ax2, tt, f, S);
set(ax2, 'YDir', 'normal');
colormap(ax2, 'parula');
ylabel(ax2, 'freq [Hz]');
xlabel(ax2, 'time [s]');
title(ax2, '|a| spectrogram - lines that rise with wheel speed are wheel vibration');
linkaxes([ax1 ax2], 'x');
xlim(ax1, [lg.t(1) lg.t(end)]);
save_fig(fig, args, 'test6_vibration.png');
end

function fpk = dominant_freq(x, fs)
fmin = 2.0;
x = x(:) - mean(x);
M = numel(x);
w = 0.5 - 0.5 * cos(2 * pi * (0:M - 1)' / (M - 1));  % np.hanning
spec = abs(fft(x .* w));
spec = spec(1:floor(M / 2) + 1);
f = (0:floor(M / 2))' * fs / M;
m = f > fmin;
if any(m)
    fm = f(m);
    [~, k] = max(spec(m));
    fpk = fm(k);
else
    fpk = NaN;
end
end

function [S, f, tt] = stft_db(x, fs, nfft, nover)
% Short-time power spectrum in dB (Hann window), no toolbox needed.
x = x(:);
step = nfft - nover;
nseg = floor((numel(x) - nfft) / step) + 1;
w = 0.5 - 0.5 * cos(2 * pi * (0:nfft - 1)' / (nfft - 1));
nf = nfft / 2 + 1;
S = zeros(nf, nseg);
for k = 1:nseg
    seg = x((k - 1) * step + (1:nfft)) .* w;
    X = fft(seg);
    P = abs(X(1:nf)) .^ 2 / (fs * sum(w .^ 2));
    P(2:end - 1) = 2 * P(2:end - 1);
    S(:, k) = 10 * log10(P + eps);
end
f = (0:nf - 1)' * fs / nfft;
tt = ((0:nseg - 1) * step + nfft / 2) / fs;
end
