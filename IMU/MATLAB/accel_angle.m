function ang = accel_angle(acc, axis)
% ACCEL_ANGLE  Tilt angle from gravity [deg], in [-180, 180]. Not unwrapped: during violent
% motion the accelerometer can point anywhere, and unwrapping would turn that into false
% 360 deg jumps.  axis 0 = tilt about IMU X, 1 = about IMU Y.
if axis == 0
    ang = atan2(acc(:, 2), acc(:, 3));
else
    ang = atan2(-acc(:, 1), acc(:, 3));
end
ang = rad2deg(ang);
end
