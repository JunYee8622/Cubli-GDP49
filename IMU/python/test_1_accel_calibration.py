"""Test 1 — accelerometer six-position calibration + gravity magnitude check.

Protocol (one recording, ~1 minute):
  1. Start log_serial.py --label six_pose.
  2. Hold the IMU still with +Z up for ~5 s, press Enter.
  3. Move to the next pose, hold ~5 s, press Enter. Repeat for -Z, +X, -X, +Y, -Y (any order).
  4. Optional: add 3-6 extra random still orientations (Enter after each) for the magnitude check.
Use the cube faces / a flat table so each axis is close to vertical.

Output: accelerometer offset + scale written to calibration.json, a C++ snippet for config.h,
and |a| in every still pose before vs after calibration (should all be 1 g).
"""
import numpy as np

from imu_common import G, common_args, load_log, marker_segments, quietest_window, save_cal, cpp_snippet, save_fig, plt, print_table

AXES = "XYZ"


def main():
    p = common_args(__doc__)
    p.add_argument("--window", type=float, default=2.0, help="still window length per pose [s]")
    args = p.parse_args()

    log = load_log(args.file, args.skip)
    norm = np.linalg.norm(log.acc, axis=1)
    n = int(args.window * log.fs)

    poses = []
    for i0, i1 in marker_segments(log.marker):
        a, b = quietest_window(norm, i0, i1, n)
        mean = log.acc[a:b].mean(axis=0)
        std = log.acc[a:b].std(axis=0).max()
        poses.append((mean, std))

    # Pick, for each of the six directions, the pose where that axis is most vertical.
    best = {}
    for mean, std in poses:
        k = int(np.argmax(np.abs(mean)))
        sign = int(np.sign(mean[k]))
        dominance = abs(mean[k]) / np.linalg.norm(mean)
        if dominance > 0.95 and ((k, sign) not in best or dominance > best[(k, sign)][1]):
            best[(k, sign)] = (mean, dominance)

    missing = [f"{'+' if s > 0 else '-'}{AXES[k]}" for k in range(3) for s in (1, -1) if (k, s) not in best]
    if missing:
        print(f"Missing still poses for: {', '.join(missing)}. "
              f"Found {len(poses)} segments — did you press Enter after each pose?")
        return

    offset, scale = np.zeros(3), np.ones(3)
    for k in range(3):
        up, down = best[(k, 1)][0][k], best[(k, -1)][0][k]
        offset[k] = (up + down) / 2
        scale[k] = (up - down) / (2 * G)

    print("\nAccelerometer calibration")
    print_table(["axis", "offset", "scale"], [[AXES[k], offset[k], scale[k]] for k in range(3)])

    rows, before, after = [], [], []
    for i, (mean, std) in enumerate(poses):
        b = np.linalg.norm(mean) / G
        a = np.linalg.norm((mean - offset) / scale) / G
        flag = "  <- not still" if std > 0.2 else ""
        rows.append([i + 1, b, a, f"{(a - 1) * 100:+.2f}%{flag}"])
        before.append(b)
        after.append(a)
    print("\nGravity magnitude in each still pose (target 1.000 g)")
    print_table(["pose", "|a| raw", "|a| cal", "error"], rows)
    print(f"\nworst error: raw {max(abs(np.array(before) - 1)) * 100:.2f}%  "
          f"-> calibrated {max(abs(np.array(after) - 1)) * 100:.2f}%")

    cal = save_cal(args.cal, {"acc_offset": offset.tolist(), "acc_scale": scale.tolist()})
    print(f"\nsaved to {args.cal}. Paste into config.h:\n{cpp_snippet(cal)}")

    fig, ax = plt.subplots(figsize=(8, 4))
    x = np.arange(1, len(poses) + 1)
    ax.bar(x - 0.2, before, 0.4, label="raw")
    ax.bar(x + 0.2, after, 0.4, label="calibrated")
    ax.axhline(1.0, color="k", lw=0.8)
    ax.set_ylim(min(before + after) - 0.02, max(before + after) + 0.02)
    ax.set_xlabel("still pose")
    ax.set_ylabel("|a| / g")
    ax.set_title("Gravity magnitude check")
    ax.legend()
    save_fig(fig, args.out, "test1_gravity_magnitude.png")


if __name__ == "__main__":
    main()
