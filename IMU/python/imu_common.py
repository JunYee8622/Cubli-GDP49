"""Shared helpers for the IMU test scripts.

Loading logs, applying calibration, the same 1D filters as firmware/filters.h,
segmenting by marker, and plotting. Every test script imports this.
"""
import argparse
import json
import os
from types import SimpleNamespace

import numpy as np
import matplotlib

matplotlib.use("Agg")  # save PNGs; works without a display
import matplotlib.pyplot as plt

G = 9.81  # m/s^2
HEADER = ["t_us", "ax", "ay", "az", "gx", "gy", "gz", "temp", "marker", "ang_comp", "ang_kf"]
DEFAULT_CAL = {
    "acc_offset": [0.0, 0.0, 0.0],
    "acc_scale": [1.0, 1.0, 1.0],
    "gyro_bias": [0.0, 0.0, 0.0],
}


# --------------------------------------------------------------------------- I/O
def load_log(path, skip=0.0):
    """Read a CSV written by log_serial.py. Returns a namespace of numpy arrays."""
    d = np.genfromtxt(path, delimiter=",", comments="#", names=True)
    t_us = d["t_us"].astype(np.int64)
    step = np.diff(t_us)
    step[step < 0] += 2**32  # micros() wraps every ~71 minutes
    t = np.concatenate([[0], np.cumsum(step)]) * 1e-6
    log = SimpleNamespace(
        t=t,
        acc=np.column_stack([d["ax"], d["ay"], d["az"]]),
        gyr=np.column_stack([d["gx"], d["gy"], d["gz"]]),
        temp=d["temp"],
        marker=d["marker"].astype(int),
    )
    if skip > 0:
        keep = log.t >= skip
        for k in ("t", "acc", "gyr", "temp", "marker"):
            setattr(log, k, getattr(log, k)[keep])
        log.t = log.t - log.t[0]
    log.dt = np.diff(log.t, prepend=log.t[0] - np.median(np.diff(log.t)))
    log.fs = 1.0 / np.median(np.diff(log.t))
    return log


def load_cal(path):
    cal = json.loads(json.dumps(DEFAULT_CAL))
    if path and os.path.exists(path):
        with open(path) as f:
            cal.update(json.load(f))
    return cal


def save_cal(path, updates):
    cal = load_cal(path)
    cal.update(updates)
    with open(path, "w") as f:
        json.dump(cal, f, indent=2)
    return cal


def apply_cal(log, cal):
    """Return (acc [m/s^2], gyro [deg/s]) after calibration."""
    acc = (log.acc - np.array(cal["acc_offset"])) / np.array(cal["acc_scale"])
    gyr = log.gyr - np.array(cal["gyro_bias"])
    return acc, gyr


def cpp_snippet(cal):
    f = lambda v: ", ".join(f"{x:.5f}f" for x in v)
    return (
        f"static const float ACC_OFFSET[3] = {{{f(cal['acc_offset'])}}};\n"
        f"static const float ACC_SCALE[3]  = {{{f(cal['acc_scale'])}}};\n"
        f"static const float GYRO_BIAS[3]  = {{{f(cal['gyro_bias'])}}};"
    )


# --------------------------------------------------------------------------- 1D tilt
def wrap180(x):
    """Wrap an angle difference into [-180, 180) deg."""
    return (x + 180.0) % 360.0 - 180.0


def accel_angle(acc, axis):
    """Tilt angle from gravity [deg], in [-180, 180]. Not unwrapped: during violent motion the
    accelerometer can point anywhere, and unwrapping would turn that into false 360 deg jumps."""
    if axis == 0:
        ang = np.arctan2(acc[:, 1], acc[:, 2])
    else:
        ang = np.arctan2(-acc[:, 0], acc[:, 2])
    return np.degrees(ang)


def gyro_only(rate, dt, start):
    return start + np.cumsum(rate * dt)


def complementary(rate, acc_ang, dt, alpha):
    """Same maths as ComplementaryFilter in filters.h."""
    out = np.empty_like(rate)
    a = acc_ang[0]
    for i in range(len(rate)):
        pred = a + rate[i] * dt[i]
        a = pred + (1 - alpha) * wrap180(acc_ang[i] - pred)  # = alpha*pred + (1-alpha)*acc, wrap-safe
        out[i] = a
    return out


def kalman(rate, acc_ang, dt, q_angle, q_bias, r):
    """Same maths as KalmanTilt in filters.h. Returns (angle, bias, gain)."""
    n = len(rate)
    ang_out, bias_out, gain_out = np.empty(n), np.empty(n), np.empty(n)
    angle, bias = acc_ang[0], 0.0
    P00, P01, P10, P11 = 10.0, 0.0, 0.0, 1.0
    for i in range(n):
        h = dt[i]
        angle += (rate[i] - bias) * h
        P00 += h * (h * P11 - P01 - P10 + q_angle)
        P01 -= h * P11
        P10 -= h * P11
        P11 += q_bias * h
        y = wrap180(acc_ang[i] - angle)
        S = P00 + r
        K0, K1 = P00 / S, P10 / S
        angle += K0 * y
        bias += K1 * y
        p00, p01 = P00, P01
        P00 -= K0 * p00
        P01 -= K0 * p01
        P10 -= K1 * p00
        P11 -= K1 * p01
        ang_out[i], bias_out[i], gain_out[i] = angle, bias, K0
    return ang_out, bias_out, gain_out


def run_filters(log, cal, args):
    """Run every estimator offline on one log. Returns dict of time series [deg]."""
    acc, gyr = apply_cal(log, cal)
    rate = gyr[:, args.axis]
    acc_ang = accel_angle(acc, args.axis)
    kf_ang, kf_bias, kf_gain = kalman(rate, acc_ang, log.dt, args.q_angle, args.q_bias, args.r)
    return {
        "rate": rate,
        "accel": acc_ang,
        "gyro": gyro_only(rate, log.dt, acc_ang[0]),
        "comp": complementary(rate, acc_ang, log.dt, args.alpha),
        "kf": kf_ang,
        "kf_bias": kf_bias,
        "kf_gain": kf_gain,
    }


# --------------------------------------------------------------------------- segments
def marker_segments(marker):
    """Index ranges [(start, end), ...] for each marker value."""
    edges = np.flatnonzero(np.diff(marker)) + 1
    starts = np.r_[0, edges]
    ends = np.r_[edges, len(marker)]
    return list(zip(starts, ends))


def quietest_window(signal, i0, i1, n):
    """Window of n samples inside [i0, i1) where `signal` has the lowest std."""
    x = signal[i0:i1]
    if len(x) <= n:
        return i0, i1
    c1 = np.cumsum(np.r_[0, x])
    c2 = np.cumsum(np.r_[0, x * x])
    s = c1[n:] - c1[:-n]
    var = (c2[n:] - c2[:-n]) / n - (s / n) ** 2
    k = int(np.argmin(var))
    return i0 + k, i0 + k + n


# --------------------------------------------------------------------------- CLI + plots
def common_args(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument("file", help="CSV log from log_serial.py")
    p.add_argument("--axis", type=int, default=0, choices=[0, 1],
                   help="tilt axis: 0 = about IMU X, 1 = about IMU Y (match TILT_AXIS)")
    p.add_argument("--cal", default="calibration.json", help="calibration file (read/updated)")
    p.add_argument("--skip", type=float, default=0.0, help="seconds to drop from the start")
    p.add_argument("--out", default="results", help="folder for plots")
    p.add_argument("--alpha", type=float, default=0.98, help="complementary filter alpha")
    p.add_argument("--q-angle", type=float, default=0.001, help="Kalman Q_angle")
    p.add_argument("--q-bias", type=float, default=0.003, help="Kalman Q_bias")
    p.add_argument("--r", type=float, default=0.03, help="Kalman R (deg^2)")
    return p


def save_fig(fig, out, name):
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, name)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"plot saved: {path}")


def print_table(headers, rows, fmt="{:>10}"):
    print(" ".join(fmt.format(h) for h in headers))
    for r in rows:
        cells = [f"{c:.3f}" if isinstance(c, (float, np.floating)) else str(c) for c in r]
        print(" ".join(fmt.format(c) for c in cells))
