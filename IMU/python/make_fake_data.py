"""Generate simulated logs for all six tests, so the scripts can be checked before the IMU arrives.

    python make_fake_data.py          # writes fake_data/*.csv
    (then run the tests on them — see README)

The simulated IMU has KNOWN errors (printed below). If the tests recover roughly these numbers,
the analysis code works and you can trust it on real data.
"""
import os

import numpy as np

from imu_common import G, HEADER, complementary, kalman

FS = 500.0
R_PIVOT = 0.10  # IMU distance from the pivot [m] -> motion corrupts the accelerometer
rng = np.random.default_rng(7)

ACC_OFFSET = np.array([0.15, -0.10, 0.25])  # m/s^2
ACC_SCALE = np.array([1.010, 0.995, 1.008])
GYRO_BIAS = np.array([0.60, -0.40, 0.30])   # dps at 28 C
GYRO_TEMPCO = np.array([0.02, 0.015, -0.01])  # dps per C
GYRO_SCALE = 1.015                          # reads 1.5 % high
ACC_NOISE, GYRO_NOISE = 0.04, 0.10


def min_jerk(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def profile(steps, start=0.0):
    """steps: ('hold', dur) or ('move', dur, target). Marker increments at each move start."""
    th, mk, cur, m = [], [], start, 0
    for st in steps:
        n = int(st[1] * FS)
        if st[0] == "hold":
            th.append(np.full(n, cur))
        else:
            m += 1
            th.append(cur + (st[2] - cur) * min_jerk(np.linspace(0, 1, n)))
            cur = st[2]
        mk.append(np.full(n, m))
    return np.concatenate(th), np.concatenate(mk)


def tilt_motion(theta_deg):
    """Specific force + body rate for rotation about IMU X, IMU at R_PIVOT along body z."""
    th = np.radians(theta_deg)
    w = np.gradient(th, 1 / FS)
    a = np.gradient(w, 1 / FS)
    f = np.column_stack([np.zeros_like(th), G * np.sin(th) - a * R_PIVOT, G * np.cos(th) - w**2 * R_PIVOT])
    rate = np.column_stack([np.degrees(w), np.zeros_like(th), np.zeros_like(th)])
    return f, rate


def write(name, f, rate, marker, temp=None):
    n = len(f)
    if temp is None:
        temp = np.full(n, 30.0)
    t_us = (np.arange(n) / FS * 1e6 + rng.normal(0, 20, n)).astype(np.int64) + 3_000_000
    t_us = np.maximum.accumulate(t_us)
    walk = np.cumsum(rng.normal(0, 0.0001, (n, 3)), axis=0)
    acc = f * ACC_SCALE + ACC_OFFSET + rng.normal(0, ACC_NOISE, (n, 3))
    gyr = rate * GYRO_SCALE + GYRO_BIAS + GYRO_TEMPCO * (temp[:, None] - 28) + walk + rng.normal(0, GYRO_NOISE, (n, 3))
    # live estimates as the firmware would compute them with an uncalibrated config.h
    dt = np.full(n, 1 / FS)
    acc_ang = np.degrees(np.arctan2(acc[:, 1], acc[:, 2]))
    comp = complementary(gyr[:, 0], acc_ang, dt, 0.98)
    kf = kalman(gyr[:, 0], acc_ang, dt, 0.001, 0.003, 0.03)[0]
    os.makedirs("fake_data", exist_ok=True)
    path = os.path.join("fake_data", name)
    cols = np.column_stack([t_us, acc, gyr, temp, marker, comp, kf])
    np.savetxt(path, cols, delimiter=",", header=",".join(HEADER), comments="",
               fmt=["%d"] + ["%.4f"] * 6 + ["%.2f", "%d", "%.3f", "%.3f"])
    print(f"wrote {path}  ({n / FS:.0f} s)")


def six_pose():
    dirs = [[0, 0, 1], [0, 0, -1], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0],
            [0.6, 0.0, 0.8], [0.0, -0.707, 0.707], [0.577, 0.577, 0.577]]
    f, rate, mk = [], [], []
    for i, d in enumerate(dirs):
        n_move, n_hold = int(1.0 * FS), int(5.0 * FS)
        f.append(G * np.array(d) + rng.normal(0, 2.0, (n_move, 3)))  # handling
        rate.append(rng.normal(0, 60, (n_move, 3)))
        f.append(np.tile(G * np.array(d) / np.linalg.norm(d), (n_hold, 1)))
        rate.append(np.zeros((n_hold, 3)))
        mk.append(np.full(n_move + n_hold, i))
    write("t1_six_pose.csv", np.vstack(f), np.vstack(rate), np.concatenate(mk))


def still():
    n = int(300 * FS)
    temp = 28 + 6 * (1 - np.exp(-np.arange(n) / FS / 60))  # warm-up
    write("t2_still.csv", np.tile([0, 0, G], (n, 1)), np.zeros((n, 3)), np.zeros(n, int), temp)


def known_angles():
    steps = [("hold", 5)]
    for a in (90, 45, 30, 0):
        steps += [("move", 1.5, a), ("hold", 5)]
    th, mk = profile(steps)
    f, rate = tilt_motion(th)
    write("t3_known_angles.csv", f, rate, mk)


def rolls():
    steps = [("hold", 3)]
    for dur in (1.2, 0.4, 0.8, 0.3, 1.5, 0.5):
        target = 90 if len([s for s in steps if s[0] == "move"]) % 2 == 0 else 0
        steps += [("move", dur, target), ("hold", 3)]
    th, mk = profile(steps)
    f, rate = tilt_motion(th)
    write("t4_roll.csv", f, rate, mk)


def return_to_start():
    t = np.arange(int(25 * FS)) / FS
    env = np.sin(np.pi * t / t[-1]) ** 2
    wild = env * (35 * np.sin(2 * np.pi * 0.7 * t) + 15 * np.sin(2 * np.pi * 2.3 * t + 1) + 3 * np.sin(2 * np.pi * 5 * t))
    th = np.concatenate([np.zeros(int(3 * FS)), wild, np.zeros(int(3 * FS))])
    f, rate = tilt_motion(th)
    write("t5_return.csv", f, rate, np.zeros(len(th), int))


def vibration():
    speeds = [0, 1000, 2000, 3000, 4000]
    n = int(5 * FS)
    th = np.full(n * len(speeds), 45.0)
    f, rate = tilt_motion(th)
    t = np.arange(len(th)) / FS
    rpm = np.repeat(speeds, n)
    amp = (rpm / 4000) ** 2
    vib = np.sin(2 * np.pi * rpm / 60 * t)
    f = f + np.column_stack([0.5 * amp * vib, 3.0 * amp * vib, 2.0 * amp * vib])
    rate = rate + np.column_stack([4.0 * amp * vib, np.zeros_like(t), np.zeros_like(t)])
    write("t6_vibration.csv", f, rate, np.repeat(np.arange(len(speeds)), n))


if __name__ == "__main__":
    print("Simulated IMU errors the tests should recover:")
    print(f"  accel offset {ACC_OFFSET} m/s^2, scale {ACC_SCALE}")
    print(f"  gyro bias {GYRO_BIAS} dps at 28 C (+ warm-up drift), gyro scale {GYRO_SCALE} "
          f"-> test 4 should report about {1 / GYRO_SCALE:.4f}")
    six_pose()
    still()
    known_angles()
    rolls()
    return_to_start()
    vibration()
