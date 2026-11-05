#!/usr/bin/env python3
"""log_udp.py - receive the ESP32 imu_logger CSV stream over WiFi (UDP), save it, and show it live.

Usage:
    python log_udp.py                         # save + live readout line refreshing 20x per second
    python log_udp.py --hz 30                 # faster live readout
    python log_udp.py --plot                  # also open a live scrolling plot (needs matplotlib)
    python log_udp.py --show 25               # scrolling raw CSV lines (every 25th) instead of the live line
    python log_udp.py --esp 172.20.10.3       # use the IP if cube-esp32.local doesn't resolve
    python log_udp.py --out run1.csv

While running, type in this window and press Enter:
    (just Enter)   insert a marker (increments the 'marker' column on the ESP32)
    c 0.2          motor current command 0.2 A, and enable (clamped on the ESP32)
    c -0.2         reverse
    x              stop: 0 A and disable
    Ctrl+C         stop logging and save (also sends 'x'), or close the plot window

All display work happens on the laptop, so it never slows the ESP32. The CSV contains every row
received, with timestamps (t_us) taken on the ESP32. Columns are read from the header the ESP32
sends, so new firmware columns appear automatically.
"""
import argparse
import collections
import datetime
import os
import shutil
import socket
import statistics
import sys
import threading
import time

CLEAR_LINE = "\r\x1b[2K"
print_lock = threading.Lock()


def out(text):
    """Print a normal line above the live readout line."""
    with print_lock:
        sys.stdout.write(CLEAR_LINE + text + "\n")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="Log and monitor ESP32 IMU + motor data streamed over UDP.")
    ap.add_argument("--esp", default="cube-esp32.local",
                    help="ESP32 hostname or IP (default: cube-esp32.local)")
    ap.add_argument("--port", type=int, default=4210, help="UDP port on the ESP32 (default 4210)")
    ap.add_argument("--out", default=None, help="output CSV (default: imu_<timestamp>.csv)")
    ap.add_argument("--hz", type=float, default=20.0, help="live readout refresh rate (default 20)")
    ap.add_argument("--show", type=int, default=0,
                    help="print every Nth raw CSV line instead of the live readout (0 = off)")
    ap.add_argument("--plot", action="store_true", help="open a live scrolling plot")
    ap.add_argument("--window", type=float, default=10.0, help="plot time window in seconds")
    args = ap.parse_args()

    if os.name == "nt":
        os.system("")  # enable ANSI escape codes in the Windows console

    out_path = args.out or datetime.datetime.now().strftime("imu_%Y%m%d_%H%M%S.csv")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("", 0))
    sock.settimeout(0.2)

    try:
        esp_ip = socket.gethostbyname(args.esp)
        out(f"# ESP32 {args.esp} -> {esp_ip}:{args.port}")
    except socket.gaierror:
        esp_ip = "255.255.255.255"
        out(f"# Could not resolve {args.esp}; broadcasting instead.")
        out("# If no data arrives, rerun with --esp <IP> (shown in Arduino IDE under Tools > Port).")
    target = (esp_ip, args.port)

    stop = threading.Event()
    lock = threading.Lock()
    plot_buf = collections.deque(maxlen=max(100, int(args.window * 1000)))  # room for up to 1 kHz
    st = {"rows": 0, "missed": 0, "markers": 0, "latest": None, "last_rx": 0.0,
          "ovr": None, "status_rest": None, "cols": None, "idx": {}}

    def send(msg):
        try:
            sock.sendto(msg, target)
        except OSError:
            pass

    def hello_loop():
        # Keeps the ESP32 streaming to us (and keeps its motor watchdog alive).
        while not stop.is_set():
            send(b"hello")
            if st["cols"] is None and st["rows_pending"]:
                send(b"h")                      # header packet was lost: ask again
            stop.wait(1.0)

    def input_loop():
        for raw in sys.stdin:
            if stop.is_set():
                break
            cmd = raw.strip()
            if cmd == "":
                send(b"m")
                st["markers"] += 1
                out(f"# marker {st['markers']} sent")
            elif cmd.lower() in ("x", "s", "stop"):
                send(b"x")
            elif cmd.lower().startswith("c"):
                try:
                    amps = float(cmd[1:].strip())
                except ValueError:
                    out("# usage: c <amps>   e.g. c 0.2")
                    continue
                send(f"c {amps:.4f}".encode())
            else:
                out("# commands: Enter = marker | c <amps> = motor current | x = stop")

    f = open(out_path, "w", newline="")
    st["rows_pending"] = []                     # rows received before the header

    def handle_status(line):
        # Line looks like: "# overruns=102  wifi=connected ip=... udp=streaming  hall_err=0"
        first, _, rest = line[2:].partition(" ")
        try:
            ovr = int(first.split("=")[1])
        except (IndexError, ValueError):
            out(line)
            return
        changed = (ovr != st["ovr"]) or (rest != st["status_rest"])
        st["ovr"] = ovr
        st["status_rest"] = rest
        if changed:
            out(line)

    def receiver():
        last_t = None
        dts = []
        period = None
        t_flush = time.time()

        def process_row(line):
            nonlocal last_t, period
            f.write(line + "\n")
            st["rows"] += 1
            if args.show > 0 and st["rows"] % args.show == 0:
                out(line)
            try:
                vals = [float(v) for v in line.split(",")]
            except ValueError:
                return
            if len(vals) != len(st["cols"]):
                return
            st["latest"] = vals
            st["last_rx"] = time.time()
            if args.plot:
                with lock:
                    plot_buf.append(vals)

            # Gap detection from t_us: estimate the loop period, then count jumps.
            t = int(vals[0])
            if last_t is not None:
                dt = t - last_t
                if dt <= 0:
                    out("# t_us went backwards (ESP32 rebooted?)")
                    if args.plot:
                        with lock:
                            plot_buf.clear()
                elif period is None:
                    dts.append(dt)
                    if len(dts) >= 200:
                        period = statistics.median(dts)
                elif dt > 1.5 * period:
                    st["missed"] += round(dt / period) - 1
            last_t = t

        while not stop.is_set():
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                data = b""
            except ConnectionResetError:
                # Windows reports an ICMP "port unreachable" this way; just keep going.
                data = b""
            except OSError:
                break

            for line in data.decode("ascii", errors="replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                if line.startswith("#"):
                    if line.startswith("# overruns="):
                        handle_status(line)      # periodic ESP32 report: show only if changed
                    else:
                        out(line)
                    continue
                if line.startswith("t_us"):
                    cols = line.split(",")
                    if st["cols"] is None:
                        st["cols"] = cols
                        st["idx"] = {name: i for i, name in enumerate(cols)}
                        f.write(line + "\n")
                        out(f"# columns: {len(cols)} ({', '.join(cols[11:]) or 'IMU only'})")
                        for pending in st["rows_pending"]:
                            process_row(pending)
                        st["rows_pending"].clear()
                    elif cols != st["cols"]:
                        out("# WARNING: ESP32 columns changed mid-run (new firmware?). "
                            "Restart log_udp.py for a clean file.")
                    continue
                if st["cols"] is None:
                    if len(st["rows_pending"]) < 5000:
                        st["rows_pending"].append(line)
                    continue
                process_row(line)

            if time.time() - t_flush >= 2.0:
                f.flush()
                t_flush = time.time()

    def display_loop():
        # One line, rewritten in place args.hz times per second.
        dt = 1.0 / max(1.0, args.hz)
        rate, rows_prev, t_prev = 0.0, 0, time.time()
        while not stop.is_set():
            time.sleep(dt)
            now = time.time()
            if now - t_prev >= 0.5:
                rate = (st["rows"] - rows_prev) / (now - t_prev)
                rows_prev, t_prev = st["rows"], now

            r = st["latest"]
            if r is None or now - st["last_rx"] > 2.0:
                text = "waiting for data... (same hotspot? ESP32 powered? IMU found? firewall allowed?)"
            else:
                ix = st["idx"]
                parts = [f"comp {r[ix['ang_comp']]:+7.2f}°  kf {r[ix['ang_kf']]:+7.2f}°"]
                if "cmd_A" in ix:
                    en = "ON " if r[ix["en"]] > 0.5 else "off"
                    ready = {1: "rdy", 0: "FAULT"}.get(int(r[ix["ready"]]), "")
                    parts.append(f"motor {en} cmd {r[ix['cmd_A']]:+5.2f}A  I {r[ix['esc_A']]:+5.2f}A  "
                                 f"hall {r[ix['hall_rpm']]:+6.0f}  esc {r[ix['esc_rpm']]:+6.0f} rpm {ready}")
                parts.append(f"gyro {r[ix['gx']]:+7.1f} {r[ix['gy']]:+7.1f} {r[ix['gz']]:+7.1f} °/s")
                ovr = st["ovr"] if st["ovr"] is not None else "-"
                parts.append(f"{rate:4.0f} Hz  miss {st['missed']}  ovr {ovr}  m{int(r[ix['marker']])}")
                parts.append(f"acc {r[ix['ax']]:+6.2f} {r[ix['ay']]:+6.2f} {r[ix['az']]:+6.2f}")
                parts.append(f"t {r[0] * 1e-6:8.2f}s")
                text = " | ".join(parts)
            width = shutil.get_terminal_size((120, 20)).columns - 1
            with print_lock:
                sys.stdout.write(CLEAR_LINE + text[:width])
                sys.stdout.flush()

    threading.Thread(target=hello_loop, daemon=True).start()
    threading.Thread(target=input_loop, daemon=True).start()
    rx = threading.Thread(target=receiver, daemon=True)
    rx.start()
    if args.show == 0:
        threading.Thread(target=display_loop, daemon=True).start()

    out(f"# Saving to {out_path}.  Enter = marker | c <amps> = motor current | x = stop | Ctrl+C = quit")

    try:
        if args.plot:
            run_plot(plot_buf, lock, args.window, stop, st)
        else:
            while not stop.is_set():
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):                      # make sure the motor is stopped
            send(b"x")
            time.sleep(0.05)
        stop.set()
        rx.join(timeout=2.0)
        f.close()
        sock.close()

    total = st["rows"] + st["missed"]
    pct = 100.0 * st["missed"] / total if total else 0.0
    out(f"# Saved {st['rows']} rows to {out_path} "
        f"(missed ~{st['missed']}, {pct:.2f}%), markers: {st['markers']}")


def run_plot(plot_buf, lock, window_s, stop, st):
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        out("# matplotlib not installed (pip install matplotlib). Continuing without the plot.")
        while not stop.is_set():
            time.sleep(0.2)
        return

    out("# plot: waiting for the column header from the ESP32...")
    while st["cols"] is None and not stop.is_set():
        time.sleep(0.1)
    if stop.is_set():
        return
    idx = st["idx"]

    panels = [("tilt [deg]", ["ang_comp", "ang_kf"]),
              ("gyro raw [deg/s]", ["gx", "gy", "gz"]),
              ("wheel speed [rpm]", ["hall_rpm", "esc_rpm"]),
              ("motor current [A]", ["cmd_A", "esc_A"])]
    panels = [(label, [c for c in cols if c in idx]) for label, cols in panels]
    panels = [(label, cols) for label, cols in panels if cols]

    fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(10, 2.2 * len(panels) + 1))
    if len(panels) == 1:
        axes = [axes]
    fig.canvas.manager.set_window_title("ESP32 IMU + motor live")
    lines = []
    for ax, (label, cols) in zip(axes, panels):
        for c in cols:
            ln, = ax.plot([], [], label=c, lw=0.9)
            lines.append((ln, idx[c]))
        ax.set_ylabel(label)
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time relative to latest sample [s]")
    axes[0].set_xlim(-window_s, 0)
    title = fig.suptitle("waiting for data...")

    def update(_):
        with lock:
            rows = list(plot_buf)
        if len(rows) < 2:
            return
        t_end = rows[-1][0]
        rows = [r for r in rows if (t_end - r[0]) * 1e-6 <= window_s]
        step = max(1, len(rows) // 1500)            # draw at most ~1500 points per line
        rows_d = rows[::step]
        x = [(r[0] - t_end) * 1e-6 for r in rows_d]
        for ln, i in lines:
            ln.set_data(x, [r[i] for r in rows_d])
        for ax in axes:
            ax.relim()
            ax.autoscale_view(scalex=False)
        axes[0].set_xlim(-window_s, 0)
        last = rows[-1]
        txt = f"ang_kf {last[idx['ang_kf']]:+.2f}°   marker {int(last[idx['marker']])}"
        if "cmd_A" in idx:
            txt += f"   cmd {last[idx['cmd_A']]:+.2f} A   hall {last[idx['hall_rpm']]:+.0f} rpm"
        title.set_text(txt)

    anim = FuncAnimation(fig, update, interval=40, cache_frame_data=False)  # ~25 fps redraw
    plt.show()   # returns when the window is closed
    del anim


if __name__ == "__main__":
    main()
