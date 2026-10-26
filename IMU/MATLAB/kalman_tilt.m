function [ang_out, bias_out, gain_out] = kalman_tilt(rate, acc_ang, dt, q_angle, q_bias, r)
% KALMAN_TILT  Same maths as KalmanTilt in firmware/filters.h. Returns angle, bias, gain.
% (Named kalman_tilt, not kalman, so it does not shadow the Control System Toolbox function.)
n = numel(rate);
ang_out = zeros(n, 1);
bias_out = zeros(n, 1);
gain_out = zeros(n, 1);
angle = acc_ang(1);
bias = 0.0;
P00 = 10.0; P01 = 0.0; P10 = 0.0; P11 = 1.0;
for i = 1:n
    h = dt(i);
    angle = angle + (rate(i) - bias) * h;
    P00 = P00 + h * (h * P11 - P01 - P10 + q_angle);
    P01 = P01 - h * P11;
    P10 = P10 - h * P11;
    P11 = P11 + q_bias * h;
    y = wrap180(acc_ang(i) - angle);
    S = P00 + r;
    K0 = P00 / S;
    K1 = P10 / S;
    angle = angle + K0 * y;
    bias = bias + K1 * y;
    p00 = P00; p01 = P01;
    P00 = P00 - K0 * p00;
    P01 = P01 - K0 * p01;
    P10 = P10 - K1 * p00;
    P11 = P11 - K1 * p01;
    ang_out(i) = angle;
    bias_out(i) = bias;
    gain_out(i) = K0;
end
end
