"""Test 6 — reaction-wheel vibration.

Protocol: clamp the cube still (or rest it on an edge against a stop). Step the wheel speed up,
holding each speed ~5 s, and press Enter at every speed change. Note the speeds in order and
pass them with --speeds (optional, just for the table labels):
    python test_6_vibration.py data/vib_*.csv --speeds 0 1000 2000 3000 4000

The cube isn't moving, so any change in the angle estimates is vibration getting through.
Output: per-speed noise of each estimate, dominant vibration frequency, and a spectrogram.
"""
import numpy as np

from imu_common import common_args, load_log, load_cal, apply_cal, run_filters, marker_segments, save_fig, plt, print_table


def dominant_freq(x, fs, fmin=2.0):
    x = x - x.mean()
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    f = np.fft.rfftfreq(len(x), 1 / fs)
    m = f > fmin
    return f[m][np.argmax(spec[m])] if m.any() else np.nan


def main():
    p = common_args(__doc__)
    p.add_argument("--speeds", nargs="*", default=None, help="wheel speed label per segment")
    p.add_argument("--trim", type=float, default=0.5, help="seconds trimmed after each speed change")
    args = p.parse_args()

    log = load_log(args.file, args.skip)
    cal = load_cal(args.cal)
    est = run_filters(log, cal, args)
    acc, _ = apply_cal(log, cal)
    anorm = np.linalg.norm(acc, axis=1)
    trim = int(args.trim * log.fs)

    rows = []
    for i, (i0, i1) in enumerate(marker_segments(log.marker)):
        s = slice(i0 + trim, i1)
        if i1 - i0 - trim < log.fs:  # under 1 s
            continue
        label = args.speeds[i] if args.speeds and i < len(args.speeds) else str(i + 1)
        rows.append([label, est["accel"][s].std(), est["comp"][s].std(), est["kf"][s].std(),
                     est["rate"][s].std(), anorm[s].std(), dominant_freq(anorm[s], log.fs)])
    print("\nNoise (std) per wheel speed — the cube is still, so this is all vibration")
    print_table(["speed", "accel deg", "comp deg", "kf deg", "rate dps", "|a| m/s2", "peak Hz"], rows, "{:>10}")
    print("If comp/kf std jumps at certain speeds, look at the peak frequency: lower the ICM-20948 DLPF,\n"
          "add a notch at that frequency, or fix the mechanical source (wheel balance, loose mount).")

    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axs[0].plot(log.t, est["accel"], lw=0.5, alpha=0.6, label="accel")
    axs[0].plot(log.t, est["comp"], label="complementary")
    axs[0].plot(log.t, est["kf"], label="Kalman")
    axs[0].set_ylabel("angle [deg]")
    axs[0].legend()
    axs[1].specgram(anorm - anorm.mean(), NFFT=256, Fs=log.fs, noverlap=128, cmap="viridis")
    axs[1].set_ylabel("freq [Hz]")
    axs[1].set_xlabel("time [s]")
    axs[1].set_title("|a| spectrogram — lines that rise with wheel speed are wheel vibration")
    save_fig(fig, args.out, "test6_vibration.png")


if __name__ == "__main__":
    main()
