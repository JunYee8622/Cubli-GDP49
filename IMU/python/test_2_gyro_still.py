"""Test 2 — gyro bias, noise, drift and warm-up (IMU completely still).

Protocol: power the board, leave it untouched on a solid surface, record 5+ minutes:
    python log_serial.py COM5 --label still --seconds 300
Don't touch the table. Use --skip 60 to ignore the warm-up minute if you want bias after warm-up.

Output: gyro bias (saved to calibration.json), noise, drift with vs without bias removal,
accelerometer-angle noise (a starting value for Kalman R), Allan deviation plot.
"""
import numpy as np

from imu_common import common_args, load_log, load_cal, apply_cal, accel_angle, save_cal, cpp_snippet, save_fig, plt, print_table

AXES = "XYZ"


def allan_dev(x, fs, n_taus=40):
    """Overlapping Allan deviation of a rate signal."""
    N = len(x)
    theta = np.cumsum(x) / fs
    ms = np.unique(np.logspace(0, np.log10(N // 10), n_taus).astype(int))
    taus, adev = [], []
    for m in ms:
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        tau = m / fs
        taus.append(tau)
        adev.append(np.sqrt(np.sum(d**2) / (2 * tau**2 * (N - 2 * m))))
    return np.array(taus), np.array(adev)


def main():
    args = common_args(__doc__).parse_args()
    log = load_log(args.file, args.skip)
    cal = load_cal(args.cal)
    acc, _ = apply_cal(log, cal)
    raw = log.gyr
    T = log.t[-1]

    bias = raw.mean(axis=0)
    noise = raw.std(axis=0)
    half = len(raw) // 2
    bias_first_half = raw[:half].mean(axis=0)
    t2 = log.t[half:] - log.t[half]
    rows = []
    for k in range(3):
        drift_raw = np.sum(raw[:, k] * log.dt)                               # no correction at all
        drift_cal = np.sum((raw[half:, k] - bias_first_half[k]) * log.dt[half:])  # bias from 1st half
        rows.append([AXES[k], bias[k], noise[k], drift_raw, drift_raw / T * 60, drift_cal / t2[-1] * 60])
    print(f"\nStill recording: {T:.0f} s at {log.fs:.0f} Hz")
    print_table(["axis", "bias dps", "noise dps", "drift deg", "raw deg/min", "cal deg/min"], rows, "{:>12}")
    print("  raw = integrating without bias removal; cal = bias from first half applied to second half")

    ang = accel_angle(acc, args.axis)
    ang_std = ang.std()
    print(f"\nAccelerometer tilt-angle noise (axis {AXES[args.axis]}): {ang_std:.3f} deg rms "
          f"-> static Kalman R ~ {ang_std**2:.4f} deg^2")

    taus, adev = allan_dev(raw[:, args.axis] - bias[args.axis], log.fs)
    arw = np.interp(1.0, taus, adev)
    bias_inst = adev.min() / 0.664
    print(f"Allan (axis {AXES[args.axis]}): angle random walk ~ {arw:.4f} deg/sqrt(s) ({arw * 60:.2f} deg/sqrt(h)), "
          f"bias instability ~ {bias_inst * 3600:.1f} deg/h")
    print(f"  data-driven Kalman start: Q_angle ~ {arw**2:.2e}, R ~ {ang_std**2:.4f}. "
          "In motion the accelerometer is much worse than at rest, so R usually ends up larger.")
    print(f"temperature: {log.temp[0]:.1f} -> {log.temp[-1]:.1f} C")

    cal = save_cal(args.cal, {"gyro_bias": bias.tolist(), "acc_angle_noise_deg": float(ang_std)})
    print(f"\nsaved gyro bias to {args.cal}. Paste into config.h:\n{cpp_snippet(cal)}")

    fig, axs = plt.subplots(3, 1, figsize=(9, 9))
    win = max(1, int(10 * log.fs))
    for k in range(3):
        axs[0].plot(log.t, np.convolve(raw[:, k], np.ones(win) / win, mode="same"), label=f"g{AXES[k].lower()}")
    axs[0].set_ylabel("10 s mean rate [dps]")
    axs[0].set_title("Gyro bias vs time (warm-up shows as a slope)")
    axs[0].legend()
    ax2 = axs[0].twinx()
    ax2.plot(log.t, log.temp, "k:", lw=0.8)
    ax2.set_ylabel("temp [C]")
    for k in range(3):
        axs[1].plot(log.t, np.cumsum(raw[:, k] * log.dt), label=f"{AXES[k]} raw")
    axs[1].set_ylabel("integrated angle [deg]")
    axs[1].set_xlabel("time [s]")
    axs[1].set_title("Drift if you integrate the gyro with no correction")
    axs[1].legend()
    axs[2].loglog(taus, adev)
    axs[2].set_xlabel("tau [s]")
    axs[2].set_ylabel("Allan dev [dps]")
    axs[2].set_title(f"Allan deviation, axis {AXES[args.axis]}")
    axs[2].grid(True, which="both", alpha=0.3)
    save_fig(fig, args.out, "test2_gyro_still.png")


if __name__ == "__main__":
    main()
