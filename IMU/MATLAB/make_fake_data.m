function make_fake_data()
% MAKE_FAKE_DATA  Generate simulated logs for all six tests, so the scripts can be checked
% before the IMU arrives.
%
%   make_fake_data         % writes fake_data/*.csv in the current folder
%   (then run the tests on them - see run_fake_demo.m or the README)
%
% The simulated IMU has KNOWN errors (printed below). If the tests recover roughly these
% numbers, the analysis code works and you can trust it on real data.
c = imu_const();
G = c.G;
HEADER = c.HEADER;

FS = 500.0;
R_PIVOT = 0.10;  % IMU distance from the pivot [m] -> motion corrupts the accelerometer
rng(7);

ACC_OFFSET = [0.15, -0.10, 0.25];   % m/s^2
ACC_SCALE = [1.010, 0.995, 1.008];
GYRO_BIAS = [0.60, -0.40, 0.30];    % dps at 28 C
GYRO_TEMPCO = [0.02, 0.015, -0.01]; % dps per C
GYRO_SCALE = 1.015;                 % reads 1.5 % high
ACC_NOISE = 0.04;
GYRO_NOISE = 0.10;

fprintf('Simulated IMU errors the tests should recover:\n');
fprintf('  accel offset [%g %g %g] m/s^2, scale [%g %g %g]\n', ACC_OFFSET, ACC_SCALE);
fprintf('  gyro bias [%g %g %g] dps at 28 C (+ warm-up drift), gyro scale %g -> test 4 should report about %.4f\n', ...
        GYRO_BIAS, GYRO_SCALE, 1 / GYRO_SCALE);
six_pose();
still();
known_angles();
rolls();
return_to_start();
vibration();

    function y = min_jerk(s)
        y = 10 * s .^ 3 - 15 * s .^ 4 + 6 * s .^ 5;
    end

    function [th, mk] = profile(steps, start)
        % steps: {'hold', dur} or {'move', dur, target}. Marker increments at each move start.
        th = [];
        mk = [];
        cur = start;
        m = 0;
        for q = 1:numel(steps)
            st = steps{q};
            n = round(st{2} * FS);
            if strcmp(st{1}, 'hold')
                seg = repmat(cur, n, 1);
            else
                m = m + 1;
                seg = cur + (st{3} - cur) * min_jerk(linspace(0, 1, n)');
                cur = st{3};
            end
            th = [th; seg]; %#ok<AGROW>
            mk = [mk; repmat(m, n, 1)]; %#ok<AGROW>
        end
    end

    function [f, rate] = tilt_motion(theta_deg)
        % Specific force + body rate for rotation about IMU X, IMU at R_PIVOT along body z.
        th = deg2rad(theta_deg);
        w = gradient(th, 1 / FS);
        a = gradient(w, 1 / FS);
        f = [zeros(size(th)), G * sin(th) - a * R_PIVOT, G * cos(th) - w .^ 2 * R_PIVOT];
        rate = [rad2deg(w), zeros(size(th)), zeros(size(th))];
    end

    function write_log(name, f, rate, marker, temp)
        n = size(f, 1);
        if nargin < 5, temp = 30.0 * ones(n, 1); end
        t_us = floor((0:n - 1)' / FS * 1e6 + 20 * randn(n, 1)) + 3000000;
        t_us = cummax(t_us);
        walk = cumsum(0.0001 * randn(n, 3), 1);
        acc = f .* ACC_SCALE + ACC_OFFSET + ACC_NOISE * randn(n, 3);
        gyr = rate * GYRO_SCALE + GYRO_BIAS + (temp - 28) .* GYRO_TEMPCO + walk + GYRO_NOISE * randn(n, 3);
        % live estimates as the firmware would compute them with an uncalibrated config.h
        dt = ones(n, 1) / FS;
        acc_ang = rad2deg(atan2(acc(:, 2), acc(:, 3)));
        comp = complementary_filter(gyr(:, 1), acc_ang, dt, 0.98);
        kf = kalman_tilt(gyr(:, 1), acc_ang, dt, 0.001, 0.003, 0.03);
        if ~isfolder('fake_data'), mkdir('fake_data'); end
        path = fullfile('fake_data', name);
        fid = fopen(path, 'w');
        fprintf(fid, '%s\n', strjoin(HEADER, ','));
        cols = [t_us, acc, gyr, temp, marker, comp, kf];
        fprintf(fid, '%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%d,%.3f,%.3f\n', cols');
        fclose(fid);
        fprintf('wrote %s  (%.0f s)\n', path, n / FS);
    end

    function six_pose()
        dirs = [0 0 1; 0 0 -1; 1 0 0; -1 0 0; 0 1 0; 0 -1 0; ...
                0.6 0.0 0.8; 0.0 -0.707 0.707; 0.577 0.577 0.577];
        f = []; rate = []; mk = [];
        for i = 1:size(dirs, 1)
            d = dirs(i, :);
            n_move = floor(1.0 * FS);
            n_hold = floor(5.0 * FS);
            f = [f; G * d + 2.0 * randn(n_move, 3)]; %#ok<AGROW>  handling
            rate = [rate; 60 * randn(n_move, 3)]; %#ok<AGROW>
            f = [f; repmat(G * d / norm(d), n_hold, 1)]; %#ok<AGROW>
            rate = [rate; zeros(n_hold, 3)]; %#ok<AGROW>
            mk = [mk; repmat(i - 1, n_move + n_hold, 1)]; %#ok<AGROW>
        end
        write_log('t1_six_pose.csv', f, rate, mk);
    end

    function still()
        n = floor(300 * FS);
        temp = 28 + 6 * (1 - exp(-(0:n - 1)' / FS / 60));  % warm-up
        write_log('t2_still.csv', repmat([0 0 G], n, 1), zeros(n, 3), zeros(n, 1), temp);
    end

    function known_angles()
        steps = {{'hold', 5}};
        for a = [90 45 30 0]
            steps{end + 1} = {'move', 1.5, a}; %#ok<AGROW>
            steps{end + 1} = {'hold', 5}; %#ok<AGROW>
        end
        [th, mk] = profile(steps, 0.0);
        [f, rate] = tilt_motion(th);
        write_log('t3_known_angles.csv', f, rate, mk);
    end

    function rolls()
        steps = {{'hold', 3}};
        for dur = [1.2 0.4 0.8 0.3 1.5 0.5]
            n_moves = sum(cellfun(@(s) strcmp(s{1}, 'move'), steps));
            if mod(n_moves, 2) == 0, target = 90; else, target = 0; end
            steps{end + 1} = {'move', dur, target}; %#ok<AGROW>
            steps{end + 1} = {'hold', 3}; %#ok<AGROW>
        end
        [th, mk] = profile(steps, 0.0);
        [f, rate] = tilt_motion(th);
        write_log('t4_roll.csv', f, rate, mk);
    end

    function return_to_start()
        t = (0:floor(25 * FS) - 1)' / FS;
        env = sin(pi * t / t(end)) .^ 2;
        wild = env .* (35 * sin(2 * pi * 0.7 * t) + 15 * sin(2 * pi * 2.3 * t + 1) + 3 * sin(2 * pi * 5 * t));
        th = [zeros(floor(3 * FS), 1); wild; zeros(floor(3 * FS), 1)];
        [f, rate] = tilt_motion(th);
        write_log('t5_return.csv', f, rate, zeros(numel(th), 1));
    end

    function vibration()
        speeds = [0 1000 2000 3000 4000];
        n = floor(5 * FS);
        th = 45.0 * ones(n * numel(speeds), 1);
        [f, rate] = tilt_motion(th);
        t = (0:numel(th) - 1)' / FS;
        rpm = repelem(speeds(:), n);
        amp = (rpm / 4000) .^ 2;
        vib = sin(2 * pi * (rpm / 60) .* t);
        f = f + [0.5 * amp .* vib, 3.0 * amp .* vib, 2.0 * amp .* vib];
        rate = rate + [4.0 * amp .* vib, zeros(size(t)), zeros(size(t))];
        write_log('t6_vibration.csv', f, rate, repelem((0:numel(speeds) - 1)', n));
    end
end
