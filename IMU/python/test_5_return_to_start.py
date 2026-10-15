"""Test 5 — return-to-start (does the filter come back to the same angle after motion?).

Protocol: cube against a fixed stop (e.g. the corner of a wall / a clamped block), still >= 3 s.
Pick it up, swing and tumble it about the tilt axis for 20-30 s, put it back exactly against
the stop, still >= 3 s. No markers needed.

A good estimator ends where it started. Gyro-only shows the drift the filter has to cancel.
"""
import numpy as np

from imu_common import common_args, load_log, load_cal, run_filters, save_fig, plt, print_table


def main():
    p = common_args(__doc__)
    p.add_argument("--still", type=float, default=2.0, help="still window at start and end [s]")
    args = p.parse_args()

    log = load_log(args.file, args.skip)
    est = run_filters(log, load_cal(args.cal), args)
    n = int(args.still * log.fs)

    rows = []
    for k, name in (("gyro", "gyro only"), ("accel", "accel only"), ("comp", "complementary"), ("kf", "Kalman")):
        start, end = est[k][:n].mean(), est[k][-n:].mean()
        rows.append([name, start, end, end - start])
    print(f"\nReturn-to-start over {log.t[-1]:.0f} s (peak rate {np.abs(est['rate']).max():.0f} dps)")
    print_table(["estimator", "start", "end", "error"], rows, "{:>14}")
    print(f"Kalman residual gyro-bias estimate at the end: {est['kf_bias'][-1]:+.3f} dps")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(log.t, est["accel"], lw=0.5, alpha=0.5, label="accel")
    ax.plot(log.t, est["gyro"], label="gyro only")
    ax.plot(log.t, est["comp"], label="complementary")
    ax.plot(log.t, est["kf"], label="Kalman")
    ax.axhline(est["accel"][:n].mean(), color="k", lw=0.6, ls="--")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("angle [deg]")
    ax.set_title("Return to start")
    ax.legend()
    save_fig(fig, args.out, "test5_return_to_start.png")


if __name__ == "__main__":
    main()
