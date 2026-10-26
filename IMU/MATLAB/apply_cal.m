function [acc, gyr] = apply_cal(lg, cal)
% APPLY_CAL  Return calibrated accelerometer [m/s^2] and gyro [deg/s].
acc = (lg.acc - cal.acc_offset) ./ cal.acc_scale;
gyr = lg.gyr - cal.gyro_bias;
end
