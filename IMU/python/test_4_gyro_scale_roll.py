"""Test 4 — gyro scale factor from 90 degree rolls.

Protocol: cube flat on a face, still >= 2 s. Roll it onto the adjacent face about the tilt axis,
hold still >= 2 s. Roll back, hold. Repeat ~6 times, mixing slow and fast rolls. No markers needed:
rolls are detected automatically from the gyro rate.

For each roll, the gyro-integrated angle is compared with (a) the accelerometer angle change
between the still periods and (b) the nominal 90 deg. A gyro scale error shows as the same
percentage error on every roll; random errors point to timing (dt) problems.
"""
import numpy as np

from imu_common import common_args, load_log, load_cal, run_filters, save_fig, plt, print_table


def main():
    p = common_args(__doc__)
    p.add_argument("--expected", type=float, default=90.0, help="nominal roll angle [deg]")
    p.add_argument("--thr", type=float, default=15.0, help="rate threshold for 'moving' [deg/s]")
    p.add_argument("--still", type=float, default=0.5, help="still window before/after each roll [s]")
    args = p.parse_args()

    log = load_log(args.file, args.skip)
    est = run_filters(log, load_cal(args.cal), args)
    rate = est["rate"]

    moving = np.abs(rate) > args.thr
    pad = int(0.15 * log.fs)
    moving = np.convolve(moving, np.ones(2 * pad + 1), mode="same") > 0  # merge brief pauses
    edges = np.flatnonzero(np.diff(moving.astype(int)))
    starts, ends = edges[::2] + 1, edges[1::2] + 1
    if moving[0]:
        starts, ends = edges[1::2] + 1, edges[2::2] + 1
    n = int(args.still * log.fs)

    rows, ratios = [], []
    for a, b in zip(starts, ends):
        if a - n < 0 or b + n > len(rate) or (b - a) < 0.1 * log.fs:
            continue
        pre, post = slice(a - n, a), slice(b, b + n)
        ref = est["accel"][post].mean() - est["accel"][pre].mean()
        i0, i1 = a - n // 2, b + n // 2
        gyro = np.sum(rate[i0:i1] * log.dt[i0:i1])
        peak = np.abs(rate[a:b]).max()
        ratios.append(ref / gyro)
        rows.append([len(rows) + 1, peak, (b - a) / log.fs, ref, gyro,
                     f"{(abs(gyro) / abs(ref) - 1) * 100:+.2f}%", f"{(abs(gyro) - args.expected):+.2f}"])

    if not rows:
        print("No rolls detected. Lower --thr or check the tilt --axis.")
        return
    print(f"\n{len(rows)} rolls detected")
    print_table(["roll", "peak dps", "dur s", "accel deg", "gyro deg", "gyro vs acc", "vs nominal"], rows, "{:>11}")
    r = np.array(ratios)
    print(f"\nscale correction (accel/gyro): mean {r.mean():.4f}, std {r.std():.4f}  "
          f"-> gyro reads {(1 / r.mean() - 1) * 100:+.2f}% {'high' if r.mean() < 1 else 'low'}")
    print("Consistent % across slow and fast rolls = scale factor. Scatter = timing or a wobbly roll.")

    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axs[0].plot(log.t, est["accel"], label="accel angle")
    axs[0].plot(log.t, est["gyro"], label="gyro only")
    axs[0].plot(log.t, est["kf"], label="Kalman")
    axs[0].set_ylabel("angle [deg]")
    axs[0].legend()
    axs[1].plot(log.t, rate)
    for a, b in zip(starts, ends):
        axs[1].axvspan(log.t[a], log.t[min(b, len(log.t) - 1)], alpha=0.15)
    axs[1].set_ylabel("rate [dps]")
    axs[1].set_xlabel("time [s]")
    save_fig(fig, args.out, "test4_roll.png")


if __name__ == "__main__":
    main()
