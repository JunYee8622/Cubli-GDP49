#!/usr/bin/env python3
"""log_udp.py - receive the cube-controller telemetry stream from the ESP32 over WiFi (UDP),
save it to a CSV file and show it live.
g
gx
g

Usage (laptop and ESP32 must be on the same WiFi / hotspot):
    python log_udp.py                       # find the ESP32 on the WiFi subnet automatically, save + live readout
    python log_udp.py --esp 172.20.10.3     # use the ESP32's IP (printed on the Serial Monitor at boot)
    python log_udp.py --plot                # also open a live scrolling plot (needs matplotlib)
    python log_udp.py --out run1.csv        # choose the file name (default: logs/cube_<timestamp>.csv)

While running (type in this window and press Enter):
    x         STOP: the controller's motor torque is forced to zero (the motor is disabled and coasts)
    g         GO: let the controller run again; it arms when the cube is within 5 deg of the balance point
    (Enter)   insert a marker (increments the 'marker' column; use it to label events in the data)
    Ctrl+C    stop logging and save (does not send a stop command)

With --plot the plot window also has STOP and GO buttons (the space bar is a STOP shortcut).

Stopping needs the WiFi link. It is not an emergency stop: keep the ESCON's power or enable switch within reach.
"""
import argparse
import collections
import datetime
import os
import socket
import statistics
import sys
import threading
import time

CLEAR_LINE = "\r\x1b[2K"
print_lock = threading.Lock()


def out(text):
    """Print a normal message without leaving the live readout line behind."""
    with print_lock:
        sys.stdout.write(CLEAR_LINE + text + "\n")
        sys.stdout.flush()


def discover_targets(port):
    """Every host address on the /24 around the laptop's default-route IP (the WiFi), plus a broadcast."""
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))      # no packet is sent; this only picks the outgoing interface
        my_ip = probe.getsockname()[0]
        probe.close()
    except OSError:
        return [("255.255.255.255", port)]
    base = my_ip.rsplit(".", 1)[0]
    found = [(f"{base}.{i}", port) for i in range(1, 255) if f"{base}.{i}" != my_ip]
    return found + [("255.255.255.255", port)]


def main():
    ap = argparse.ArgumentParser(description="Log and monitor cube-controller telemetry streamed over UDP.")
    ap.add_argument("--esp", default=None,
                    help="ESP32 IP address (default: probe the WiFi subnet and use the first device that answers)")
    ap.add_argument("--port", type=int, default=4210, help="UDP port on the ESP32 (default 4210)")
    ap.add_argument("--out", default=None, help="output CSV (default: logs/cube_<timestamp>.csv)")
    ap.add_argument("--hz", type=float, default=30.0, help="live readout refresh rate (default 30)")
    ap.add_argument("--plot", action="store_true", help="open a live scrolling plot")
    ap.add_argument("--window", type=float, default=10.0, help="plot time window in seconds")
    args = ap.parse_args()

    if os.name == "nt":
        os.system("")  # enable ANSI escape codes in the Windows console

    if args.out:
        out_path = os.path.abspath(args.out)
    else:
        logs = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(logs, exist_ok=True)
        out_path = os.path.join(logs, datetime.datetime.now().strftime("cube_%Y%m%d_%H%M%S.csv"))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("", 0))
    sock.settimeout(0.2)

    stop = threading.Event()
    lock = threading.Lock()
    plot_buf = collections.deque(maxlen=max(100, int(args.window * 300)))
    st = {"rows": 0, "missed": 0, "markers": 0, "latest": None, "last_rx": 0.0,
          "cols": None, "idx": {}}

    # Where to send: the given IP, or (until the ESP32 answers) every host on the WiFi's /24 subnet.
    # A plain broadcast is not enough: on a PC with VMware/VPN adapters Windows may send it out of the wrong one.
    if args.esp:
        targets = [(args.esp, args.port)]
        out(f"# Sending hello to {args.esp}:{args.port}")
    else:
        targets = discover_targets(args.port)
        out(f"# ESP32 IP not given: probing {len(targets)} addresses on your WiFi subnet until it answers")
        out("# (use --esp <IP> to skip this; the IP is printed in the Serial Monitor at boot)")

    def send(msg):
        for t in list(targets):
            try:
                sock.sendto(msg, t)
            except OSError:
                pass

    def hello_loop():
        # Keeps the ESP32 streaming to us; also asks for the header again if it was lost.
        while not stop.is_set():
            send(b"hello")
            if st["cols"] is None:
                send(b"h")
            stop.wait(1.0)

    def send_stop():
        for _ in range(3):      # UDP can drop a packet; the extra copies are harmless
            send(b"x")
            time.sleep(0.01)
        out("# STOP sent")

    def send_go():
        send(b"g")
        out("# GO sent")

    def input_loop():
        for raw in sys.stdin:
            if stop.is_set():
                break
            cmd = raw.strip().lower()
            if cmd in ("x", "s", "stop"):
                send_stop()
            elif cmd in ("g", "go"):
                send_go()
            elif cmd == "":
                send(b"m")
                st["markers"] += 1
                out(f"# marker {st['markers']} sent")
            else:
                out("# unknown command. x = stop, g = go, Enter = marker")

    f = open(out_path, "w", newline="")
    pending = []   # rows received before the header

    def receiver():
        last_t = None
        dts = []
        period = None
        t_flush = time.time()

        def process_row(line):
            nonlocal last_t, period
            try:
                vals = [float(v) for v in line.split(",")]
            except ValueError:
                return
            if len(vals) != len(st["cols"]):
                return
            f.write(line + "\n")
            st["rows"] += 1
            st["latest"] = vals
            st["last_rx"] = time.time()
            if args.plot:
                with lock:
                    plot_buf.append(vals)

            # Gap detection from the ESP32's t_us: estimate the sample period, then count jumps.
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
                    if len(dts) >= 50:
                        period = statistics.median(dts)
                elif dt > 1.5 * period:
                    st["missed"] += round(dt / period) - 1
            last_t = t

        while not stop.is_set():
            try:
                data, addr = sock.recvfrom(4096)
                if len(targets) > 1 and addr[1] == args.port:
                    # First answer wins: from now on talk only to this ESP32
                    targets[:] = [(addr[0], args.port)]
                    out(f"# Found ESP32 at {addr[0]}")
            except socket.timeout:
                data = b""
            except ConnectionResetError:
                data = b""      # Windows reports an ICMP "port unreachable" this way; keep going
            except OSError:
                break

            for line in data.decode("ascii", errors="replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                if line.startswith("#"):
                    if line.startswith("# BUSY"):
                        # The ESP32 repeats this while another window holds the stream; show it only once
                        if not st.get("busy_shown"):
                            st["busy_shown"] = True
                            out(line)
                        continue
                    out(line)
                    continue
                if line.startswith("t_us"):
                    cols = line.split(",")
                    if st["cols"] is None:
                        st["cols"] = cols
                        st["idx"] = {name: i for i, name in enumerate(cols)}
                        f.write(line + "\n")
                        out(f"# columns: {', '.join(cols)}")
                        for p in pending:
                            process_row(p)
                        pending.clear()
                    continue
                if st["cols"] is None:
                    if len(pending) < 5000:
                        pending.append(line)
                    continue
                process_row(line)

            if time.time() - t_flush >= 2.0:
                f.flush()
                t_flush = time.time()

    def display_loop():
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
                text = ("waiting for data... (another log_udp.py still open? same WiFi? ESP32 powered? "
                        "firewall allowed Python?)")
            else:
                ix = st["idx"]
                if "stopped" in ix and r[ix["stopped"]] > 0.5:
                    state = "STOPPED"
                else:
                    state = "HALTED" if r[ix["halted"]] > 0.5 else ("ARMED" if r[ix["armed"]] > 0.5 else "ready")
                text = (f"{state:7s} | phi {r[ix['phi_deg']]:+6.2f} deg | M1 {r[ix['wheel_rpm']]:+7.0f} rpm"
                        f" | M1 tau {r[ix['tau_Nm']]:+7.4f} Nm"
                        f" | gyro {r[ix['gx_dps']]:+7.1f} {r[ix['gy_dps']]:+7.1f} {r[ix['gz_dps']]:+7.1f} deg/s"
                        f" | {rate:4.0f} Hz  miss {st['missed']}  m{int(r[ix['marker']])}"
                        f" | t {r[0] * 1e-6:8.2f}s")
            width = (os.get_terminal_size().columns if sys.stdout.isatty() else 120) - 1
            with print_lock:
                if stop.is_set():
                    return
                sys.stdout.write(CLEAR_LINE + text[:width])
                sys.stdout.flush()

    threading.Thread(target=hello_loop, daemon=True).start()
    threading.Thread(target=input_loop, daemon=True).start()
    rx = threading.Thread(target=receiver, daemon=True)
    rx.start()
    threading.Thread(target=display_loop, daemon=True).start()

    out(f"# Saving to {out_path}")
    out("# x + Enter = STOP | g + Enter = GO | Enter = marker | Ctrl+C = stop logging and save")

    try:
        if args.plot:
            run_plot(plot_buf, lock, args.window, stop, st, send_stop, send_go)
        else:
            while not stop.is_set():
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        rx.join(timeout=2.0)
        f.close()
        sock.close()

    total = st["rows"] + st["missed"]
    pct = 100.0 * st["missed"] / total if total else 0.0
    out(f"# Saved {st['rows']} rows to {out_path} "
        f"(missed ~{st['missed']}, {pct:.2f}%), markers: {st['markers']}")


def run_plot(plot_buf, lock, window_s, stop, st, send_stop, send_go):
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import Button
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

    panels = [("error angle [deg]", ["phi_deg"]),
              ("gyro [deg/s]", ["gx_dps", "gy_dps", "gz_dps"]),
              ("motor 1 wheel speed [rpm]", ["wheel_rpm"]),
              ("motor 1 torque [Nm]", ["tau_Nm"])]
    panels = [(label, [c for c in cols if c in idx]) for label, cols in panels]
    panels = [(label, cols) for label, cols in panels if cols]

    fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(10, 2.2 * len(panels) + 1))
    if len(panels) == 1:
        axes = [axes]
    fig.canvas.manager.set_window_title("Cubli telemetry live")
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

    # STOP / GO buttons along the bottom of the window (keep references so they are not garbage collected)
    fig.subplots_adjust(bottom=0.16)
    stop_ax = fig.add_axes([0.62, 0.02, 0.15, 0.07])
    go_ax = fig.add_axes([0.80, 0.02, 0.15, 0.07])
    stop_btn = Button(stop_ax, "STOP (space)", color="#e57373", hovercolor="#ef5350")
    go_btn = Button(go_ax, "GO", color="#81c784", hovercolor="#66bb6a")
    stop_btn.on_clicked(lambda _e: send_stop())
    go_btn.on_clicked(lambda _e: send_go())
    fig.canvas.mpl_connect("key_press_event", lambda e: send_stop() if e.key == " " else None)

    def update(_):
        with lock:
            rows = list(plot_buf)
        if len(rows) < 2:
            return
        t_end = rows[-1][0]
        rows = [r for r in rows if (t_end - r[0]) * 1e-6 <= window_s]
        x = [(r[0] - t_end) * 1e-6 for r in rows]
        for ln, i in lines:
            ln.set_data(x, [r[i] for r in rows])
        for ax in axes:
            ax.relim()
            ax.autoscale_view(scalex=False)
        axes[0].set_xlim(-window_s, 0)

    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.show()   # returns when the window is closed
    del anim


if __name__ == "__main__":
    main()
