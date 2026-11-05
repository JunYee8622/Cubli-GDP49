"""Record the ESP32 stream to a CSV file.

    python log_serial.py COM5 --label still --seconds 300
    python log_serial.py /dev/ttyUSB0 --label six_pose

Press Enter while logging to insert a marker (the 'marker' column increments).
The tests use markers to split a recording into poses / segments.
Ctrl+C stops early. Opening the port resets the ESP32, which is expected.
"""
import argparse
import os
import sys
import threading
import time
from datetime import datetime

import serial  # pip install pyserial

from imu_common import HEADER


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("port")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--label", default="log", help="test name, used in the file name")
    p.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    p.add_argument("--dir", default="data")
    args = p.parse_args()

    os.makedirs(args.dir, exist_ok=True)
    path = os.path.join(args.dir, f"{args.label}_{datetime.now():%Y%m%d_%H%M%S}.csv")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(1.5)  # board reboots when the port opens
    ser.reset_input_buffer()

    markers = [0]

    def key_thread():
        for _ in sys.stdin:
            ser.write(b"m")
            markers[0] += 1
            print(f"  marker -> {markers[0]}")

    threading.Thread(target=key_thread, daemon=True).start()

    n, t0 = 0, time.time()
    print(f"logging to {path}  (Enter = marker, Ctrl+C = stop)")
    # Column names come from the board's own "t_us,..." header line, so firmware that adds
    # columns (e.g. motor telemetry) still logs. Falls back to the IMU-only HEADER.
    cols = None
    with open(path, "w") as f:
        try:
            while args.seconds == 0 or time.time() - t0 < args.seconds:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    print(line)  # status / overrun messages from the board
                    continue
                if line.startswith("t_us"):
                    if cols is None:
                        cols = line.split(",")
                        f.write(line + "\n")
                    continue
                if cols is None:
                    if line.count(",") != len(HEADER) - 1:
                        continue  # header not seen yet and not the IMU-only format
                    cols = HEADER
                    f.write(",".join(HEADER) + "\n")
                if line.count(",") != len(cols) - 1:
                    continue  # partial line from the reset
                f.write(line + "\n")
                n += 1
                if n % 2500 == 0:
                    print(f"  {n} samples, {time.time() - t0:.0f} s")
        except KeyboardInterrupt:
            pass
    print(f"done: {n} samples -> {path}")


if __name__ == "__main__":
    main()
