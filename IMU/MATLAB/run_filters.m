function est = run_filters(lg, cal, args)
% RUN_FILTERS  Run every estimator offline on one log. Returns a struct of time series [deg]:
%   rate, accel, gyro, comp, kf, kf_bias, kf_gain
[acc, gyr] = apply_cal(lg, cal);
rate = gyr(:, args.Axis + 1);
acc_ang = accel_angle(acc, args.Axis);
[kf_ang, kf_bias, kf_gain] = kalman_tilt(rate, acc_ang, lg.dt, args.QAngle, args.QBias, args.R);
est.rate = rate;
est.accel = acc_ang;
est.gyro = gyro_only(rate, lg.dt, acc_ang(1));
est.comp = complementary_filter(rate, acc_ang, lg.dt, args.Alpha);
est.kf = kf_ang;
est.kf_bias = kf_bias;
est.kf_gain = kf_gain;
end
