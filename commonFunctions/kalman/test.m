% Test of numerical propagation of the NAS II quaternion state
% ------------------------------------------------------------------------

clear
clc
close all

% %% Reference Motion (analytical solution)
% vel = @(t) [0.1 * sin(2*pi*0.1*t);
%             0.2 * cos(2*pi*0.1*t);
%             0.3 * sin(2*pi*0.1*t)]; % Angular velocity function (rad/s)
% 
% % Euler angles reference as integral of angular velocity
% euler_ref = @(t) [-cos(pi*t/5)/(2*pi);
%                sin(pi*t/5)/pi;
%                -(3*cos(pi*t/5))/(2*pi)]; % Euler angles function (rad) (XYZ)
% 
% q_ref = @(x, y, z) angle2quat(x, y, z, 'XYZ'); % Quaternion function [q0, q1, q2, q3]

% Original constant angular velocity (commented out)
% Omega = pi/3;
% q_ref = @(t) [cos(Omega*t/2); sin(Omega*t/2); 0*t; 0*t]; % [w, x, y, z] - MATLAB convention
% Analytical solution for Euler angles: rotation about x-axis
% eulref = @(t) [Omega*t, 0*t, 0*t]; % [roll, pitch, yaw] in radians

%% Variable angular velocity profile (from demo_quaternion_integration)
% Time-varying angular velocity components - increased for more visible effects
omega_profile = @(t) [0.1 * sin(0.2*t);    % wx - increased from 0.1 to 1.0
                      0.15 * cos(0.1*t);     % wy - increased from 0.15 to 1.5
                      0.2 * ones(size(t))]; % wz - increased from 0.2 to 2.0

%% Sensor sampling
dt = 0.01; % Time step (s)
t = 0:dt:200; % Time vector
gyro_measures = zeros(3, length(t)); % Simulated gyro measures (rad/s)

% Noise parameters
gyro_noise_std = 0.05; % Standard deviation of gyro noise (rad/s)
rng(42); % Set random seed for reproducibility

% Calculate time-varying angular velocities with noise
for k = 1:length(t)
    omega_true = omega_profile(t(k));
    % Add white Gaussian noise to each gyro axis
    gyro_noise = gyro_noise_std * randn(3, 1);
    gyro_measures(:, k) = omega_true + gyro_noise;
end
q = zeros(4, length(t)); % Preallocate quaternion storage for 1st order [w, x, y, z]
q(:, 1) = [1, 0, 0, 0]'; % Initial quaternion state [w, x, y, z]
q1 = q; % Preallocate quaternion storage for 4th order
q2 = q; % Preallocate quaternion storage for Exponential map
eul = zeros(length(t), 3); % Preallocate Euler angles storage for 1st order
eul1 = zeros(length(t), 3); % Preallocate Euler angles storage for 2nd order
eul2 = zeros(length(t), 3); % Preallocate Euler angles storage for Exponential map

for i = 2:length(t)
    %% 1st Order Integration
    % Omega matrix for quaternion derivative: dq/dt = 0.5 * Omega * q
    % Convention: q = [w, x, y, z], omega = [wx, wy, wz]
    omega = [0, -gyro_measures(1, i), -gyro_measures(2, i), -gyro_measures(3, i);
             gyro_measures(1, i), 0, gyro_measures(3, i), -gyro_measures(2, i);
             gyro_measures(2, i), -gyro_measures(3, i), 0, gyro_measures(1, i);
             gyro_measures(3, i), gyro_measures(2, i), -gyro_measures(1, i), 0];

    q(:, i) = q(:, i-1) + 0.5 * dt * omega * q(:, i-1); % Quaternion [w, x, y, z]
    q(:, i) = q(:, i) / norm(q(:, i)); % Normalization of the quaternion
    eul(i, :) = quat2eul(q(:, i)', 'XYZ'); % Euler angles for reference

    %% 4th Order Integration (Runge-Kutta)
    % Using MATLAB's quatmultiply which expects [w, x, y, z] format
    k1 = 0.5 * quatmultiply(q1(:, i-1)', [0, gyro_measures(:, i)'])';
    k2 = 0.5 * quatmultiply((q1(:, i-1) + dt/2 * k1)', [0, gyro_measures(:, i)'])';
    k3 = 0.5 * quatmultiply((q1(:, i-1) + dt/2 * k2)', [0, gyro_measures(:, i)'])';
    k4 = 0.5 * quatmultiply((q1(:, i-1) + dt * k3)', [0, gyro_measures(:, i)'])';
    q1(:, i) = q1(:, i-1) + dt/6 * (k1 + 2*k2 + 2*k3 + k4); % Quaternion [w, x, y, z]
    q1(:, i) = q1(:, i) / norm(q1(:, i)); % Normalization of the quaternion
    eul1(i, :) = quat2eul(q1(:, i)', 'XYZ'); % Euler angles for reference

    %% Exponential Integration
    theta = norm(gyro_measures(:, i)) * dt; % Rotation angle
    if theta > 0
        axis = gyro_measures(:, i) / norm(gyro_measures(:, i)); % Rotation axis
        dq = [cos(theta/2); sin(theta/2) * axis]; % Delta quaternion [w, x, y, z]
    else
        dq = [1; 0; 0; 0]; % Identity quaternion
    end
    q2(:, i) = quatmultiply(q2(:, i-1)', dq')'; % Updated quaternion
    eul2(i, :) = quat2eul(q2(:, i)', 'XYZ'); % Euler angles for reference

end

%% Reference solution using ODE45 (analytical reference)
% Integrate quaternion using ODE45 for comparison
quatODE = @(t, q) 0.5 * [0, -omega_profile(t)';
                         omega_profile(t), -skew_symmetric(omega_profile(t))] * q;

% Set very strict tolerances for high precision reference
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-14);

% Solve using ODE45 with strict tolerances
[t_ref, q_ref_ode] = ode45(quatODE, [t(1), t(end)], [1; 0; 0; 0], options);

% Interpolate reference solution to match our time vector
qref = zeros(4, length(t));
for i = 1:4
    qref(i, :) = interp1(t_ref, q_ref_ode(:, i), t, 'linear', 'extrap');
end

% Calculate reference Euler angles
eulref_val = zeros(length(t), 3);
for k = 1:length(t)
    eulref_val(k, :) = quat2eul(qref(:, k)', 'XYZ');
end

% Helper function for skew-symmetric matrix
function S = skew_symmetric(w)
    S = [0, -w(3), w(2);
         w(3), 0, -w(1);
         -w(2), w(1), 0];
end

%% Plot
f = figure; f.WindowStyle = 'docked';
subplot(4,1,1);
plot(t, q(1, :), 'b', 'LineWidth', 2, 'DisplayName', '1st Order');
hold on;
plot(t, q1(1, :), 'r--', 'LineWidth', 2, 'DisplayName', '4th Order (Runge-Kutta)');
plot(t, q2(1, :), 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Map');
plot(t, qref(1, :), 'k', 'LineWidth', 2, 'DisplayName', 'Reference');
xlabel('Time (s)');
ylabel('q0 (w)');
legend show;
grid on;
subplot(4,1,2);
plot(t, q(2, :), 'b', 'LineWidth', 2, 'DisplayName', '1st Order');
hold on;
plot(t, q1(2, :), 'r--', 'LineWidth', 2, 'DisplayName', '4th Order (Runge-Kutta)');
plot(t, q2(2, :), 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Map');
plot(t, qref(2, :), 'k', 'LineWidth', 2, 'DisplayName', 'Reference');
xlabel('Time (s)');
ylabel('q1 (x)');
legend show;
grid on;
subplot(4,1,3);
plot(t, q(3, :), 'b', 'LineWidth', 2, 'DisplayName', '1st Order');
hold on;
plot(t, q1(3, :), 'r--', 'LineWidth', 2, 'DisplayName', '4th Order (Runge-Kutta)');
plot(t, q2(3, :), 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Map');
plot(t, qref(3, :), 'k', 'LineWidth', 2, 'DisplayName', 'Reference');
xlabel('Time (s)');
ylabel('q2 (y)');
legend show;
grid on;
subplot(4,1,4);
plot(t, q(4, :), 'b', 'LineWidth', 2, 'DisplayName', '1st Order');
hold on;
plot(t, q1(4, :), 'r--', 'LineWidth', 2, 'DisplayName', '4th Order (Runge-Kutta)');
plot(t, q2(4, :), 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Map');
plot(t, qref(4, :), 'k', 'LineWidth', 2, 'DisplayName', 'Reference');
xlabel('Time (s)');
ylabel('q3 (z)');
legend show;
grid on;
sgtitle('Quaternion Propagation: 1st Order vs 4th Order (Runge-Kutta) vs Exponential Map vs ODE45 Reference');

f = figure; f.WindowStyle = 'docked';
subplot(3,1,1);
plot(t, rad2deg(eul(:, 1)), 'b', 'LineWidth', 2, 'DisplayName', '1st Order');
hold on;
plot(t, rad2deg(eul1(:, 1)), 'r--', 'LineWidth', 2, 'DisplayName', '4th Order (Runge-Kutta)');
plot(t, rad2deg(eul2(:, 1)), 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Map');
plot(t, rad2deg(eulref_val(:, 1)), 'k', 'LineWidth', 2, 'DisplayName', 'Reference');
xlabel('Time (s)');
ylabel('Roll (deg)');
legend show;
grid on;

subplot(3,1,2);
plot(t, rad2deg(eul(:, 2)), 'b', 'LineWidth', 2, 'DisplayName', '1st Order');
hold on;
plot(t, rad2deg(eul1(:, 2)), 'r--', 'LineWidth', 2, 'DisplayName', '4th Order (Runge-Kutta)');
plot(t, rad2deg(eul2(:, 2)), 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Map');
plot(t, rad2deg(eulref_val(:, 2)), 'k', 'LineWidth', 2, 'DisplayName', 'Reference');
xlabel('Time (s)');
ylabel('Pitch (deg)');
legend show;
grid on;

subplot(3,1,3);
plot(t, rad2deg(eul(:, 3)), 'b', 'LineWidth', 2, 'DisplayName', '1st Order');
hold on;
plot(t, rad2deg(eul1(:, 3)), 'r--', 'LineWidth', 2, 'DisplayName', '4th Order (Runge-Kutta)');
plot(t, rad2deg(eul2(:, 3)), 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Map');
plot(t, rad2deg(eulref_val(:, 3)), 'k', 'LineWidth', 2, 'DisplayName', 'Reference');
xlabel('Time (s)');
ylabel('Yaw (deg)');
legend show;
grid on;

sgtitle('Euler Angles from Quaternion: 1st Order vs 4th Order (Runge-Kutta) vs Exponential Map vs ODE45 Reference');

% Error Plot
f = figure; f.WindowStyle = 'docked';

% Filter parameters for error smoothing
filter_window = 201; % Moving average window size (must be odd)
filter_order = 3;    % Savitzky-Golay filter order

% Calculate errors
roll_error_1st = abs(rad2deg(eulref_val(:, 1) - eul(:, 1)));
roll_error_4th = abs(rad2deg(eulref_val(:, 1) - eul1(:, 1)));
roll_error_exp = abs(rad2deg(eulref_val(:, 1) - eul2(:, 1)));

pitch_error_1st = abs(rad2deg(eulref_val(:, 2) - eul(:, 2)));
pitch_error_4th = abs(rad2deg(eulref_val(:, 2) - eul1(:, 2)));
pitch_error_exp = abs(rad2deg(eulref_val(:, 2) - eul2(:, 2)));

yaw_error_1st = abs(rad2deg(eulref_val(:, 3) - eul(:, 3)));
yaw_error_4th = abs(rad2deg(eulref_val(:, 3) - eul1(:, 3)));
yaw_error_exp = abs(rad2deg(eulref_val(:, 3) - eul2(:, 3)));

% Apply Savitzky-Golay filter to smooth the errors
roll_error_1st_filt = sgolayfilt(roll_error_1st, filter_order, filter_window);
roll_error_4th_filt = sgolayfilt(roll_error_4th, filter_order, filter_window);
roll_error_exp_filt = sgolayfilt(roll_error_exp, filter_order, filter_window);

pitch_error_1st_filt = sgolayfilt(pitch_error_1st, filter_order, filter_window);
pitch_error_4th_filt = sgolayfilt(pitch_error_4th, filter_order, filter_window);
pitch_error_exp_filt = sgolayfilt(pitch_error_exp, filter_order, filter_window);

yaw_error_1st_filt = sgolayfilt(yaw_error_1st, filter_order, filter_window);
yaw_error_4th_filt = sgolayfilt(yaw_error_4th, filter_order, filter_window);
yaw_error_exp_filt = sgolayfilt(yaw_error_exp, filter_order, filter_window);

subplot(3,1,1);
semilogy(t, roll_error_exp_filt, 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Error (Filtered)');
hold on;
semilogy(t, roll_error_4th_filt, 'r--', 'LineWidth', 2, 'DisplayName', '4th Order Error (Filtered)');
semilogy(t, roll_error_1st_filt, 'b', 'LineWidth', 2, 'DisplayName', '1st Order Error (Filtered)');

xlabel('Time (s)');
ylabel('|Roll Error| (deg)');
legend show;
grid on;
subplot(3,1,2);
semilogy(t, pitch_error_exp_filt, 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Error (Filtered)');
hold on;
semilogy(t, pitch_error_4th_filt, 'r--', 'LineWidth', 2, 'DisplayName', '4th Order Error (Filtered)');
semilogy(t, pitch_error_1st_filt, 'b', 'LineWidth', 2, 'DisplayName', '1st Order Error (Filtered)');
xlabel('Time (s)');
ylabel('|Pitch Error| (deg)');
legend show;
grid on;
subplot(3,1,3);
semilogy(t, yaw_error_exp_filt, 'm-.', 'LineWidth', 2, 'DisplayName', 'Exponential Error (Filtered)');
hold on;
semilogy(t, yaw_error_4th_filt, 'r--', 'LineWidth', 2, 'DisplayName', '4th Order Error (Filtered)');
semilogy(t, yaw_error_1st_filt, 'b', 'LineWidth', 2, 'DisplayName', '1st Order Error (Filtered)');
xlabel('Time (s)');
ylabel('|Yaw Error| (deg)');
legend show;
grid on;

sgtitle('Filtered Euler Angle Errors: Comparison of Integration Methods with Noisy Gyro Data');