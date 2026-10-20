"""Visualise a recorded IMU log — quick look at everything before running the tests.

    python visualise_graph.py data/known_*.csv
    python visualise_graph.py fake_data/t5_return.csv --axis 0 --t0 5 --t1 20

Four stacked panels on one time axis:
    1. accelerometer, calibrated [m/s^2]
    2. gyroscope, calibrated [deg/s]
    3. tilt angle: accelerometer-only, gyro-only, complementary, Kalman [deg]
    4. temperature [C]
Dashed vertical lines are the markers inserted with Enter while logging.
"""
import os

import numpy as np

from imu_common import common_args, load_log, load_cal, apply_cal, run_filters, marker_segments, save_fig, plt


def main():
    p = common_args(__doc__)
    p.add_argument("--t0", type=float, default=None, help="start of the plotted window [s]")
    p.add_argument("--t1", type=float, default=None, help="end of the plotted window [s]")
    args = p.parse_args()

    log = load_log(args.file, args.skip)
    cal = load_cal(args.cal)
    acc, gyr = apply_cal(log, cal)
    est = run_filters(log, cal, args)

    t = log.t
    lo = args.t0 if args.t0 is not None else t[0]
    hi = args.t1 if args.t1 is not None else t[-1]
    axis_name = "XY"[args.axis]

    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    for k, name in enumerate("XYZ"):
        ax[0].plot(t, acc[:, k], lw=0.8, label=name)
        ax[1].plot(t, gyr[:, k], lw=0.8, label=name)
    ax[0].set_ylabel("accel [m/s²]")
    ax[1].set_ylabel("gyro [deg/s]")

    ax[2].plot(t, est["accel"], lw=0.6, alpha=0.5, color="grey", label="accel only")
    ax[2].plot(t, est["gyro"], lw=0.8, ls=":", label="gyro only")
    ax[2].plot(t, est["comp"], lw=1.2, label=f"complementary (α={args.alpha})")
    ax[2].plot(t, est["kf"], lw=1.2, label="Kalman")
    ax[2].set_ylabel(f"tilt about {axis_name} [deg]")

    ax[3].plot(t, log.temp, lw=1.0, color="tab:red")
    ax[3].set_ylabel("temp [°C]")
    ax[3].set_xlabel("time [s]")

    # marker boundaries (one dashed line per marker change)
    for start, _ in marker_segments(log.marker)[1:]:
        for a in ax:
            a.axvline(t[start], color="k", ls="--", lw=0.6, alpha=0.4)

    for a in ax:
        a.grid(alpha=0.3)
        if a is not ax[3]:
            a.legend(loc="upper right", ncol=4, fontsize=8)
    ax[0].set_xlim(lo, hi)

    ax[0].set_title(f"{os.path.basename(args.file)} — {t[-1]:.1f} s at {log.fs:.0f} Hz, "
                    f"{len(marker_segments(log.marker))} marker segment(s)")

    name = os.path.splitext(os.path.basename(args.file))[0] + "_graph.png"
    save_fig(fig, args.out, name)


if __name__ == "__main__":
    main()
