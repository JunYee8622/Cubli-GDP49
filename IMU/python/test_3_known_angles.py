"""Test 3 — static accuracy at known angles.

Protocol (one recording): place the cube at each reference angle in turn, hold still ~5 s,
press Enter, move to the next. References from the cube itself: flat on a face = 0,
rolled onto the next face = 90, a phone inclinometer / digital protractor for anything between.
Then pass the angles in the same order:
    python test_3_known_angles.py data/known_*.csv --angles 0 90 45 30 0

Output: table of reference vs accel / complementary / Kalman, both absolute and with the
first pose used as zero (removes IMU mounting offset).
"""
import numpy as np

from imu_common import common_args, load_log, load_cal, run_filters, marker_segments, quietest_window, save_fig, plt, print_table


def main():
    p = common_args(__doc__)
    p.add_argument("--angles", type=float, nargs="+", required=True, help="reference angles in order [deg]")
    p.add_argument("--window", type=float, default=2.0)
    args = p.parse_args()

    log = load_log(args.file, args.skip)
    est = run_filters(log, load_cal(args.cal), args)
    segs = marker_segments(log.marker)
    if len(segs) < len(args.angles):
        print(f"{len(segs)} marker segments but {len(args.angles)} angles given.")
        return
    n = int(args.window * log.fs)

    vals = []
    for (i0, i1), ref in zip(segs, args.angles):
        a, b = quietest_window(est["rate"], i0, i1, n)
        vals.append([ref] + [est[k][a:b].mean() for k in ("accel", "comp", "kf")] + [(a + b) / 2])
    vals = np.array(vals)

    rows = []
    for v in vals:
        rel_ref = v[0] - vals[0, 0]
        rel = v[1:4] - vals[0, 1:4]
        rows.append([v[0], v[1], v[2], v[3], v[1] - v[0], (rel - rel_ref)[0], (rel - rel_ref)[1], (rel - rel_ref)[2]])
    print("\nStatic angles [deg]. 'rel err' uses the first pose as zero (mounting offset removed).")
    print_table(["ref", "accel", "comp", "kf", "abs err", "rel accel", "rel comp", "rel kf"], rows)
    rel = np.array(rows)[1:, 5:8]
    if len(rel):
        print(f"max |rel err|: accel {np.abs(rel[:, 0]).max():.2f}, comp {np.abs(rel[:, 1]).max():.2f}, "
              f"kf {np.abs(rel[:, 2]).max():.2f} deg")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(log.t, est["accel"], lw=0.5, alpha=0.6, label="accel")
    ax.plot(log.t, est["comp"], label="complementary")
    ax.plot(log.t, est["kf"], label="Kalman")
    for v in vals:
        ax.plot(log.t[int(v[4])], v[0] + (vals[0, 1] - vals[0, 0]), "kx", ms=10)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("angle [deg]")
    ax.set_title("Known angles (x = reference, shifted by first-pose offset)")
    ax.legend()
    save_fig(fig, args.out, "test3_known_angles.png")


if __name__ == "__main__":
    main()
