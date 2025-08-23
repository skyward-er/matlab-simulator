% Test of numerical propagation of the NAS II quaternion state
% ------------------------------------------------------------------------


gyro_measures = -pi + rand(3, 200) * 2*pi; % Simulated gyro measures (rad/s)
q = zeros(4, 200); % Preallocate quaternion storage for 1st order
q(:, 1) = [1; 0; 0; 0]; % Initial quaternion state
q1 = q; % Preallocate quaternion storage for 2nd order
dt = 0.01; % Time step (s)
eul = zeros(200, 3); % Preallocate Euler angles storage for 1st order
eul1 = zeros(200, 3); % Preallocate Euler angles storage for 2nd order

for i = 2:200
    %% 1st Order Integration

    omega = [0 -gyro_measures(3, i) -gyro_measures(2, i) -gyro_measures(1, i);
                gyro_measures(3, i) 0 gyro_measures(1, i) -gyro_measures(2, i);
                gyro_measures(2, i) -gyro_measures(1, i) 0 gyro_measures(3, i);
                gyro_measures(1, i) gyro_measures(2, i) -gyro_measures(3, i) 0];                  % Omega matrix [rad/s]

    q(:, i) = q(:, i-1) + 0.5 * dt * omega * q(:, i-1); % Quaternion [q0, q1, q2, q3]
    q(:, i) = q(:, i) / norm(q(:, i)); % Normalization of the quaternion
    q([2, 4], i) = q([4, 2], i);
    eul (i, :) = quat2eul(q(:, i)'); % Euler angles for reference

    %% 4th Order Integration (Runge-Kutta)
    k1 = 0.5 * quatmultiply(q1(:, i-1)', [0; gyro_measures(:, i)]')';
    k2 = 0.5 * quatmultiply((q1(:, i-1) + dt/2 * k1)', [0; gyro_measures(:, i)]')';
    k3 = 0.5 * quatmultiply((q1(:, i-1) + dt/2 * k2)', [0; gyro_measures(:, i)]')';
    k4 = 0.5 * quatmultiply((q1(:, i-1) + dt * k3)', [0; gyro_measures(:, i)]')';
    q1(:, i) = q1(:, i-1) + dt/6 * (k1 + 2*k2 + 2*k3 + k4); % Quaternion [q0, q1, q2, q3]
    q1(:, i) = q1(:, i) / norm(q1(:, i)); % Normalization of the quaternion
    eul1 (i, :) = quat2eul(q1(:, i)'); % Euler angles for reference

end

%% Plot
f = figure; f.WindowStyle = 'docked';
subplot(4,1,1);
plot(0:dt:(200-1)*dt, q(1, :), 'b', 'DisplayName', '1st Order');
hold on;
plot(0:dt:(200-1)*dt, q1(1, :), 'r--', 'DisplayName', '4th Order (Runge-Kutta)');
xlabel('Time (s)');
ylabel('q0');
legend show;
subplot(4,1,2);
plot(0:dt:(200-1)*dt, q(2, :), 'b', 'DisplayName', '1st Order');
hold on;
plot(0:dt:(200-1)*dt, q1(2, :), 'r--', 'DisplayName', '4th Order (Runge-Kutta)');
xlabel('Time (s)');
ylabel('q1');
legend show;
subplot(4,1,3);
plot(0:dt:(200-1)*dt, q(3, :), 'b', 'DisplayName', '1st Order');
hold on;
plot(0:dt:(200-1)*dt, q1(3, :), 'r--', 'DisplayName', '4th Order (Runge-Kutta)');
xlabel('Time (s)');
ylabel('q2');
legend show;
subplot(4,1,4);
plot(0:dt:(200-1)*dt, q(4, :), 'b', 'DisplayName', '1st Order');
hold on;
plot(0:dt:(200-1)*dt, q1(4, :), 'r--', 'DisplayName', '4th Order (Runge-Kutta)');
xlabel('Time (s)');
ylabel('q3');
legend show;
sgtitle('Quaternion Propagation: 1st Order vs 4th Order (Runge-Kutta)');

f = figure; f.WindowStyle = 'docked';
subplot(3,1,1);
plot(0:dt:(200-1)*dt, rad2deg(eul(:, 1)), 'b', 'DisplayName', '1st Order');
hold on;
plot(0:dt:(200-1)*dt, rad2deg(eul1(:, 1)), 'r--', 'DisplayName', '4th Order (Runge-Kutta)');
xlabel('Time (s)');
ylabel('Roll (deg)');
legend show;

subplot(3,1,2);
plot(0:dt:(200-1)*dt, rad2deg(eul(:, 2)), 'b', 'DisplayName', '1st Order');
hold on;
plot(0:dt:(200-1)*dt, rad2deg(eul1(:, 2)), 'r--', 'DisplayName', '4th Order (Runge-Kutta)');
xlabel('Time (s)');
ylabel('Pitch (deg)');
legend show;

subplot(3,1,3);
plot(0:dt:(200-1)*dt, rad2deg(eul(:, 3)), 'b', 'DisplayName', '1st Order');
hold on;
plot(0:dt:(200-1)*dt, rad2deg(eul1(:, 3)), 'r--', 'DisplayName', '4th Order (Runge-Kutta)');
xlabel('Time (s)');
ylabel('Yaw (deg)');
legend show;

sgtitle('Euler Angles from Quaternion: 1st Order vs 4th Order (Runge-Kutta)');
