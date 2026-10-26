% RUN_FAKE_DEMO  Dry run of every test on simulated data (no IMU needed).
%
% Run from this folder:   run_fake_demo
% Tests 1 and 2 write calibration.json, which every later test reads, so keep this order.
% The simulated IMU has known errors (printed by make_fake_data); if the tests recover them,
% the analysis code is working. Plots go to results/.
make_fake_data();

test_1_accel_calibration('fake_data/t1_six_pose.csv');
test_2_gyro_still('fake_data/t2_still.csv', 'Skip', 120);
test_3_known_angles('fake_data/t3_known_angles.csv', 'Angles', [0 90 45 30 0]);
test_4_gyro_scale_roll('fake_data/t4_roll.csv');
test_5_return_to_start('fake_data/t5_return.csv');
test_6_vibration('fake_data/t6_vibration.csv', 'Speeds', [0 1000 2000 3000 4000]);
visualise_graph('fake_data/t3_known_angles.csv');
