# IMU Test Kit — MATLAB version

A MATLAB port of everything in `../python/`. Same tests, same maths as the firmware filters,
same `calibration.json` format (the Python and MATLAB scripts can share one calibration file),
same CSV logs. Needs MATLAB R2021a or newer (`serialport` for logging needs R2019b+). No toolboxes.

Run everything from this folder (`cd IMU/MATLAB`); `data/`, `fake_data/`, `results/` and
`calibration.json` are created in the current folder.

## Dry run now (no IMU needed)

```matlab
run_fake_demo
```

That generates simulated logs with known errors and runs all six tests plus the visualiser.
If the tests recover the injected errors (e.g. test 4 reports about +1.5 % gyro scale) the
analysis is working. Plots are saved to `results/`.

## Python → MATLAB

Command-line flags become name-value pairs:

| Python | MATLAB |
|--------|--------|
| `python log_serial.py COM5 --label still --seconds 300` | `log_serial('COM5', 'Label', 'still', 'Seconds', 300)` |
| `python test_1_accel_calibration.py data/six_pose_*.csv` | `test_1_accel_calibration('data/six_pose_XXXX.csv')` |
| `python test_2_gyro_still.py data/still_*.csv --skip 60` | `test_2_gyro_still('data/still_XXXX.csv', 'Skip', 60)` |
| `python test_3_known_angles.py data/known_*.csv --angles 0 90 45 0` | `test_3_known_angles('data/known_XXXX.csv', 'Angles', [0 90 45 0])` |
| `python test_4_gyro_scale_roll.py data/roll_*.csv` | `test_4_gyro_scale_roll('data/roll_XXXX.csv')` |
| `python test_5_return_to_start.py data/return_*.csv` | `test_5_return_to_start('data/return_XXXX.csv')` |
| `python test_6_vibration.py data/vib_*.csv --speeds 0 1000 2000` | `test_6_vibration('data/vib_XXXX.csv', 'Speeds', [0 1000 2000])` |
| `python visualise_graph.py data/known_*.csv --t0 5 --t1 20` | `visualise_graph('data/known_XXXX.csv', 'T0', 5, 'T1', 20)` |
| `python make_fake_data.py` | `make_fake_data` |

MATLAB has no shell wildcards, so give the exact file name (tab-complete it). Options shared by
every script: `Axis` (0 = tilt about IMU X, 1 = about Y), `Cal`, `Skip`, `Out`, `Alpha`, `QAngle`,
`QBias`, `R`, and `Show` (`true` also leaves the figure open on screen). `help test_4_gyro_scale_roll`
lists the extra options of each test. Use the same order of tests as in `../README.md` — tests 1
and 2 write `calibration.json`, which later tests read.

## Files

| File | What it is |
|------|------------|
| `log_serial.m` | record the ESP32 stream to `data/*.csv`; a small window with a Marker button (Enter/Space) |
| `test_1` … `test_6_*.m` | the six tests, one function each |
| `visualise_graph.m` | plot any log: accel, gyro, filter angles, temperature, markers |
| `make_fake_data.m`, `run_fake_demo.m` | simulated logs and a script that runs every test on them |
| `load_log`, `load_cal`, `save_cal`, `apply_cal`, `cpp_snippet` | loading logs and calibration |
| `accel_angle`, `gyro_only`, `complementary_filter`, `kalman_tilt`, `run_filters`, `wrap180` | the filters (same maths as `firmware/filters.h`) |
| `marker_segments`, `quietest_window`, `print_table`, `common_args`, `new_fig`, `save_fig`, `imu_const` | shared helpers (what `imu_common.py` held) |

## Checked against Python

Tests 1–6 were run on the same simulated CSV files in MATLAB and in Python and print identical
numbers. `log_serial.m` could not be tested without a board, so check your first recording with
`visualise_graph` (and watch the `# overruns` messages from the ESP32).
