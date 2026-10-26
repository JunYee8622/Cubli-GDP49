function c = imu_const()
% IMU_CONST  Shared constants: gravity and the CSV column order of the log files.
c.G = 9.81;  % m/s^2
c.HEADER = {'t_us', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp', 'marker', 'ang_comp', 'ang_kf'};
end
