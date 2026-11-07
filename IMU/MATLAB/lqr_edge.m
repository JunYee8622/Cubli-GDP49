% lqr_edge.m  LQR gains for 1D edge balancing of the reaction-wheel cube (one motor).
% Base MATLAB only (no toolboxes). Fill in the parameters, run, and paste the three #define lines
% it prints into balance_config.h.
%
% Model (cube pivoting on its edge, wheel speed measured relative to the cube, i.e. motor speed):
%   I0 * theta_dd = m*g*l*sin(theta) - Km*i
%   Iw * (omega_d + theta_dd) = Km*i
% States x = [theta; theta_dot; omega] in rad, rad/s, rad/s.  Input u = motor current i [A].
% theta = 0 at the balance point; positive current pushes the body towards negative theta
% (the 'p' sign test on the cube sets TILT_SIGN so the firmware matches this).

% ===================== PARAMETERS: replace with your cube's values =====================
m   = 1.20;            % total mass of the cube incl. wheel, motor, boards [kg]       (weigh it)
a   = 0.15;            % cube side length [m]                                         (measure)
l   = a / sqrt(2);     % balancing edge -> centre of mass [m]. Use CAD if the CoM isn't central.
I0  = 2/3 * m * a^2;   % inertia of the whole cube about the balancing edge [kg m^2]
                       %   (uniform-cube estimate; use the CAD mass properties if you have them)
Iw  = 5.934469e-4;     % wheel + motor rotor inertia about the motor axis [kg m^2]
                       %   (CAD, or ESCON Studio auto-tuning with the wheel fitted: "Mass Moment of
                       %    Inertia" in g*cm^2 -> multiply by 1e-7)
Km  = 0.0396;          % motor torque constant [Nm/A] (your ESCON tuning: 39.6 mNm/A)
% ===================== DESIGN CHOICES =====================
Ts   = 1/500;          % control period [s] (LOOP_HZ)
Imax = 2.0;            % current clamp [A] (I_MAX_BAL)
% Bryson's rule: roughly "how big may each state get before it matters"
th_max  = 2   * pi/180;      % 2 deg
thd_max = 20  * pi/180;      % 20 deg/s
w_max   = 2000 * 2*pi/60;    % 2000 rpm
i_max   = 1.0;               % 1 A
% ===========================================================================================

g = 9.81;
M = m * g * l;
A = [0 1 0;  M/I0 0 0;  -M/I0 0 0];
B = [0; -Km/I0; Km*(1/Iw + 1/I0)];

% Zero-order-hold discretisation without the Control System Toolbox
E  = expm([A B; zeros(1, 4)] * Ts);
Ad = E(1:3, 1:3);
Bd = E(1:3, 4);

% Discrete LQR by Riccati iteration (same result as dlqr)
Q = diag([1/th_max^2, 1/thd_max^2, 1/w_max^2]);
R = 1/i_max^2;
P = Q;
for k = 1:50000
    K = (R + Bd' * P * Bd) \ (Bd' * P * Ad);
    Pn = Q + Ad' * P * (Ad - Bd * K);
    if max(abs(Pn(:) - P(:))) < 1e-9 * max(abs(P(:))), P = Pn; break; end
    P = Pn;
end
K = (R + Bd' * P * Bd) \ (Bd' * P * Ad);
poles = eig(Ad - Bd * K);

fprintf('\nOpen loop: falls with time constant %.3f s (unstable pole %.1f rad/s)\n', sqrt(I0/M), sqrt(M/I0));
fprintf('Closed-loop pole magnitudes: %s (all must be < 1)\n', mat2str(abs(poles'), 5));
fprintf('\nPaste into balance_config.h:\n');
fprintf('#define K_TH              (%.5ff)     // A per rad\n', K(1));
fprintf('#define K_RATE            (%.5ff)     // A per (rad/s)\n', K(2));
fprintf('#define K_WHEEL           (%.7ff)     // A per (rad/s) of wheel speed\n', K(3));
fprintf('(= %.3f A per degree of tilt, %.4f A per deg/s)\n', K(1)*pi/180, K(2)*pi/180);

% ---------------- nonlinear simulation: catch at 3 deg, with clamp and sensor noise ----------------
T = 5; N = round(T/Ts);
x = [3*pi/180; 0; 0];
X = zeros(3, N); U = zeros(1, N);
rng(1);
for k = 1:N
    meas = x + [0.02*pi/180*randn; 0.3*pi/180*randn; 20*2*pi/60*randn];  % your measured noise levels
    u = -K * meas;
    u = max(-Imax, min(Imax, u));
    tau = Km * u;
    thdd = (M * sin(x(1)) - tau) / I0;
    wd = tau / Iw - thdd;
    x = x + Ts * [x(2); thdd; wd];
    X(:, k) = x; U(k) = u;
end
t = (1:N) * Ts;
fprintf('\nSimulation from 3 deg: final tilt %.3f deg, peak current %.2f A, peak wheel %.0f rpm\n', ...
        X(1, end)*180/pi, max(abs(U)), max(abs(X(3, :)))*60/(2*pi));

figure('Name', 'Edge balance LQR simulation');
subplot(3, 1, 1); plot(t, X(1, :)*180/pi); grid on; ylabel('tilt [deg]');
title('Catch from 3 deg (current clamp + measured sensor noise)');
subplot(3, 1, 2); plot(t, U); grid on; ylabel('current [A]');
subplot(3, 1, 3); plot(t, X(3, :)*60/(2*pi)); grid on; ylabel('wheel [rpm]'); xlabel('time [s]');
