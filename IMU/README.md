# IMU Test Kit — GDP 49 Balancing Cube

Bench tests for the ICM-20948 + ESP32 before it goes into the cube: calibrate the IMU,
check it is accurate, and compare the complementary and Kalman filters on real data.

The ESP32 only streams raw data. All calibration and analysis runs on a PC in Python,
so you can re-run and retune on the same recording without touching the hardware.

```
imu_test_kit/
├── firmware/imu_logger/
│   ├── imu_logger.ino      ESP32: read IMU at a fixed rate, stream CSV, live filter estimates
│   ├── config.h            pins, rate, ranges, calibration constants, filter tuning  ← edit this
│   └── filters.h           complementary + 1D Kalman (reused later in the balance controller)
└── python/
    ├── log_serial.py       record the stream to data/*.csv  (Enter = insert marker)
    ├── imu_common.py       shared: loading, calibration, the same filters in Python, plotting
    ├── test_1_accel_calibration.py   six-position accel calibration + gravity magnitude check
    ├── test_2_gyro_still.py          gyro bias, noise, drift, warm-up, Allan deviation
    ├── test_3_known_angles.py        static accuracy at known angles
    ├── test_4_gyro_scale_roll.py     gyro scale factor from 90° rolls
    ├── test_5_return_to_start.py     filter returns to the same angle after motion
    ├── test_6_vibration.py           reaction-wheel vibration: noise vs speed, spectrogram
    ├── visualise_graph.py            plot any log: accel, gyro, filter angles, temp, markers
    └── make_fake_data.py             simulated logs to dry-run every test now
```

## Setup

**Hardware.** Adafruit ICM-20948 → ESP32 over I2C: 3V3, GND, SDA → GPIO21, SCL → GPIO22
(change in `config.h` for other boards). Default address 0x69.

**Firmware.** Arduino IDE with the ESP32 board package. Library Manager: *Adafruit ICM20X*,
*Adafruit BusIO*, *Adafruit Unified Sensor*. Open `firmware/imu_logger/imu_logger.ino`, set
`TILT_AXIS` in `config.h` to the axis the cube will tilt about, upload.

**Python.** `pip install numpy matplotlib pyserial`, then run everything from `python/`.

**MATLAB.** Prefer MATLAB? `MATLAB/` is a full port of `python/` (same tests, same
`calibration.json`). Run `run_fake_demo` from inside it; see `MATLAB/README.md`.

## Dry run now (no IMU needed)

```
cd python
python make_fake_data.py
python test_1_accel_calibration.py fake_data/t1_six_pose.csv
python test_2_gyro_still.py        fake_data/t2_still.csv --skip 120
python test_3_known_angles.py      fake_data/t3_known_angles.csv --angles 0 90 45 30 0
python test_4_gyro_scale_roll.py   fake_data/t4_roll.csv
python test_5_return_to_start.py   fake_data/t5_return.csv
python test_6_vibration.py         fake_data/t6_vibration.csv --speeds 0 1000 2000 3000 4000
```

The simulated IMU has known errors (printed by `make_fake_data.py`), e.g. a 1.5 % gyro scale
error. Test 4 should report about +1.45 % — if the tests recover the injected errors, the
analysis code is working. Plots go to `results/`. Run tests 1 and 2 first: they write
`calibration.json`, which every later test reads.

## Real IMU: run in this order

Record with `python log_serial.py <PORT> --label <name> [--seconds N]`
(`COM5` on Windows, `/dev/ttyUSB0` on Linux, `/dev/cu.usbserial-*` on Mac).
Press **Enter** while logging to insert a marker between poses or segments.

| # | Record | Protocol | Run |
|---|--------|----------|-----|
| 1 | `--label six_pose` | Each axis straight up then straight down (6 poses), hold ~5 s each, Enter after each. Optionally add a few random still orientations. | `test_1_accel_calibration.py data/six_pose_*.csv` |
| 2 | `--label still --seconds 300` | Don't touch anything for 5 min. | `test_2_gyro_still.py data/still_*.csv --skip 60` |
| 3 | `--label known` | Hold at each reference angle ~5 s, Enter between. Cube faces give 0° and 90°. | `test_3_known_angles.py data/known_*.csv --angles 0 90 45 0` |
| 4 | `--label roll` | Flat ≥ 2 s, roll onto adjacent face, hold ≥ 2 s, roll back. ~6 rolls, mix slow and fast. | `test_4_gyro_scale_roll.py data/roll_*.csv` |
| 5 | `--label return` | Against a fixed stop ≥ 3 s, swing/tumble 20–30 s, back on the stop ≥ 3 s. | `test_5_return_to_start.py data/return_*.csv` |
| 6 | `--label vib` | Cube clamped, step wheel speed every ~5 s, Enter at each change. Needs the motor mounted. | `test_6_vibration.py data/vib_*.csv --speeds 0 1000 2000 3000` |

After tests 1 and 2, paste the printed C++ block into `config.h` and re-upload so the live
`ang_comp` / `ang_kf` columns use the calibration.

Filter tuning is shared by tests 3–6: `--alpha`, `--q-angle`, `--q-bias`, `--r`. Re-run the
same recording with different values to compare; when you're happy, copy them into `config.h`.

## What "good" looks like (rough guide)

| Check | Rough target | If it fails |
|-------|--------------|-------------|
| Gravity magnitude after calibration (1) | within ±0.5 % in every pose | re-do six-position with steadier poses |
| Gyro drift after bias removal (2) | well under 1 °/min | let it warm up, re-measure bias after warm-up |
| Relative static error (3) | < 0.5° | check reference, mounting, calibration |
| Gyro vs accel on rolls (4) | same % on slow and fast rolls, < ~1 % | consistent % = scale factor; scatter = timing / loop overruns |
| Return-to-start (5) | < 0.5° for comp and Kalman | retune α / Q / R |
| Vibration (6) | estimate std not growing much with speed | lower DLPF cutoff, notch filter, balance wheel, stiffen mount |

For balancing, a small constant angle offset doesn't matter (the balance point is found
experimentally anyway). Noise, drift and lag do — tests 2, 4, 5 and 6.

## Notes

- **Calibrate in the range you will use.** `config.h` defaults to ±16 g / ±2000 dps for jump-up.
- **Watch for `# overruns=N`** in the logger output. A rising count means the loop can't hold
  `LOOP_HZ`; lower it or speed up I2C. Timing jitter corrupts gyro integration.
- The magnetometer is ignored on purpose: balancing only needs tilt, and the motors would
  corrupt it.
- These are 1D (edge-balancing) filters. Corner balancing needs a 3D quaternion filter
  (Mahony or a multiplicative EKF); tests 1, 2 and 6 still apply unchanged.
