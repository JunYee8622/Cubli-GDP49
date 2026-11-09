function ang = gyro_only(rate, dt, start)
% GYRO_ONLY  Integrate the gyro rate, starting from the angle START.
ang = start + cumsum(rate .* dt);
end
