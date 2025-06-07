%% ZVK simulator
% Author: Guglielmo Gualdana
% Co-Author: Alessandro Cartocci
% Skyward Experimental Rocketry | GNC Dept | gnc??@kywarder.eu
% email: guglielmo.gualdana@skywarder.eu, alessandro.cartocci@skywarder.eu
% Release date: tbd
%

%% Load variables
clear variables;
close all;
clc;

IMU_main = table2array(readtable("main_Boardcore_IMUDataEUROC")); % [tempo*e-6 acc_x acc_y acc_z w_x,w_y,w_z]

dt = mean(diff(IMU_main(:,1))); %Only for plotting

% Last usefull Index estimation
idx_end_main = find(IMU_main(:,2)>20,1,'first');
idx_end_main = round(idx_end_main - 5e6/dt);

%Plotting misure IMU main
time_acc_main    = IMU_main(1:idx_end_main,1)*1e-6;
acc_main         = IMU_main(1:idx_end_main,2:4);
time_gyr_main    = IMU_main(1:idx_end_main,5)*1e-6;
gyr_main         = IMU_main(1:idx_end_main,6:8);

figure()
subplot(2,1,1)
title('acc main')
hold on
grid on
plot(time_acc_main, acc_main(:,1), 'DisplayName','x');
plot(time_acc_main, acc_main(:,2), 'DisplayName','y');
plot(time_acc_main, acc_main(:,3), 'DisplayName','z');
legend

subplot(2,1,2)
title('gyro main')
hold on
grid on
plot(time_gyr_main, gyr_main(:,1), 'DisplayName','x');
plot(time_gyr_main, gyr_main(:,2), 'DisplayName','y');
plot(time_gyr_main, gyr_main(:,3), 'DisplayName','z');
legend




IMU_payload = table2array(readtable("payload_Boardcore_IMUDataEUROC")); % [tempo*e-6 acc_x acc_y acc_z w_x,w_y,w_z]


dt = mean(diff(IMU_payload(:,1))); %Only for plotting

% Last usefull Index estimation
idx_end_payload = find(IMU_payload(:,2)>20,1,'first');
idx_end_payload = round(idx_end_payload - 5e6/dt);

% Plotting IMU Payload
time_acc_payload    = IMU_payload(1:idx_end_payload,1)*1e-6;
acc_payload         = IMU_payload(1:idx_end_payload,2:4);
time_gyr_payload    = IMU_payload(1:idx_end_payload,5)*1e-6;
gyr_payload         = IMU_payload(1:idx_end_payload,6:8);

figure()
subplot(2,1,1)
title('acc payload')
hold on
grid on
plot(time_acc_payload, acc_payload(:,1), 'DisplayName','x');
plot(time_acc_payload, acc_payload(:,2), 'DisplayName','y');
plot(time_acc_payload, acc_payload(:,3), 'DisplayName','z');
legend

subplot(2,1,2)
title('gyro payload')
hold on
grid on
plot(time_gyr_payload, gyr_payload(:,1), 'DisplayName','x');
plot(time_gyr_payload, gyr_payload(:,2), 'DisplayName','y');
plot(time_gyr_payload, gyr_payload(:,3), 'DisplayName','z');
legend

% % Imput data's CSV
% input_ZVK_main = array2table(IMU_main(:,1:8),    'VariableNames', {'time_acc_main', 'acc_x_main', 'acc_y_main', 'acc_z_main', 'time_gyro_main', 'gyro_x_main', 'gyro_y_main', 'gyro_z_main'});
% input_ZVK_payload = array2table(IMU_payload(:,1:8), 'VariableNames', {'time_acc_payload', 'acc_x_payload', 'acc_y_payload', 'acc_z_payload', 'time_gyro_payload', 'gyro_x_payload', 'gyro_y_payload', 'gyro_z_payload'});
% writetable(input_ZVK_main, 'input_ZVK_main.csv');
% writetable(input_ZVK_payload, 'input_ZVK_payload.csv');


%% Initial params
x0 = zeros(1,24); % vel, acc, bias_acc_main, bias_acc_payload, theta, omega, bias_gyro_main, bias_gyro_payoad

% Estimated noise standard deviations
sigma_gyro         = 5e-3;     % [rad/s]   gyroscope std(standard deviation)
sigma_beta_g       = 1e-5;     % [rad/s]   gyroscope bias std
sigma_acc          = 4e-2;     % [m/s^2]   accelerometer std
sigma_beta_acc     = 1e-5;     % [m/s^2]   accelerometer bias std

zvk.Q = diag([
            (1e-6)^2                            * ones(3,1);    % vel
            (1e-6)^2                            * ones(3,1);    % acceleration
            sigma_beta_acc^2                    * ones(3,1);    % bias accelerometer
            sigma_beta_acc^2                    * ones(3,1);    % bias accelerometer
            (1e-6)^2                            * ones(3,1);    % theta
            (1e-6)^2                            * ones(3,1);    % angular velocity
            sigma_beta_g^2                      * ones(3,1);    % bias gyroscope
            sigma_beta_g^2                      * ones(3,1);    % bias gyroscope
        ]);


% Initial state covariance matrix P
P0 = diag([
        (1e-5)^2    * ones(3,1);    % velocity uncertainty [m/s]
        (1e-5)^2    * ones(3,1);    % acceleration [m/s^2]
        (1e-1)^2    * ones(3,1);    % accelerometer bias main [m/s^2]
        (1e-1)^2    * ones(3,1);    % accelerometer bias payload [m/s^2]
        (1e-5)^2    * ones(3,1);    % ang velocity [rad/s]
        (1e-5)^2    * ones(3,1);    % ang acceleration [rad/s^2]
        (1e-3)^2    * ones(3,1)     % gyro bias main [rad/s]
        (1e-3)^2    * ones(3,1)     % gyro bias payload [rad/s]
        ]);
            

% Measurement noise covariance matrix R
zvk.R_fake      = diag([
                (1e-6)^2                        * ones(3,1);        % FAKE zero velocity [m/s]
                (1e-6)^2                        * ones(3,1);        % FAKE angle [rad]
            ]);
zvk.R_acc       = diag( sigma_acc^2 * ones(3,1));   % accelerometer [m/s^2]
zvk.R_gyro      = diag(sigma_gyro^2 * ones(3,1));   % angular velocity [rad/s]

% zvk.quaternions 
Q0 = angle2quat(133*pi/180, 85*pi/180, 0*pi/180, 'ZYX')';

% % zvk.quaternion error simulations
% error = [1 0.5*deg2rad([0 0 0]) ];
% Q0 = zvk.quatmultiply(Q0', error);
%Q0 = Q0';

zvk.quat = [Q0(2:4); Q0(1)];

% load("Ksave");

%% Initializzation
x = x0;
P = P0;

%time vector generation
t_IMU_main      = IMU_main(1:idx_end_main,1);
t_IMU_payload   = IMU_payload(1:idx_end_payload,1);

% Start index
ii = 2;

ZVK_freq = 50; %[Hz]
ZVK_sampling_time = 1/ZVK_freq*1e6;

% Last sample initializzation
last_t = t_IMU_main(1);

t_start = t_IMU_main(1);
t_end   = t_IMU_main(end);

ZVK_time = [];

%% SIMULATION
tic

for t = t_start:ZVK_sampling_time:t_end
    
    dt = (t-last_t)*1e-6;

    % MAIN measures
    index_imu = sum(t >= t_IMU_main);
    IMU_main_measurment = IMU_main(index_imu,[2:4 6:8]);
    acc_meas_m = IMU_main_measurment(1:3);
    omeg_meas_m = IMU_main_measurment(4:6);
    
    % PAYLOAD measures
    index_imu = sum(t >= t_IMU_payload);
    IMU_payload_measurment = IMU_payload(index_imu,[2:4 6:8]);
    acc_meas_p = IMU_payload_measurment(1:3);
    omeg_meas_p = IMU_payload_measurment(4:6);

    %%% ZVK(PALLE)
    [x(ii,:), P(:,:,ii)] = run_zvk(x(ii-1,:), P(:,:,ii-1), dt, acc_meas_m, omeg_meas_m, acc_meas_p, omeg_meas_p, zvk);
    
    ii = ii+1;
    last_t = t;

    ZVK_time = [ZVK_time, t];
    disp(t/t_end*100)
end
x = x(2:end,:); %only for plotting reasons

toc

%% Output data's CSV
output_ZVK = array2table([ZVK_time, x], 'VariableNames', {'ZVK timestamp', ...
                         'vel_x', 'vel_y', 'vel_z', ...
                         'acc_x', 'acc_y', 'acc_z', ...
                         'bias_acc_main_x', 'bias_acc_main_y', 'bias_acc_main_z', ...
                         'bias_acc_payload_x', 'bias_acc_payload_y', 'bias_acc_payload_z', ...
                         'theta_x', 'theta_y', 'theta_z', ...
                         'angular_vel_x', 'angular_vel_y', 'angular_vel_z', ...
                         'bias_gyro_main_x', 'bias_gyro_main_y', 'bias_gyro_main_z', ...
                         'bias_gyro_payload_x', 'bias_gyro_payload_y', 'bias_gyro_payload_z'});
writetable(output_ZVK, 'output_ZVK.csv');

%%
% clearvars
close all
% clc

% load("bckp_doppia_imu.mat")

est_bias_a_main         = x(end,7:9);
est_bias_a_payload      = x(end,10:12);
est_bias_g_main         = x(end,19:21);
est_bias_g_payload      = x(end,22:24);

est_bias_a_main_std     = P(7:9,7:9,end);
est_bias_a_payload_std  = P(10:12,10:12,end);
est_bias_g_main_std     = P(19:21,19:21,end);
est_bias_g_payload_std  = P(22:24,22:24,end);


% corrected_acc_main = (acc_main - est_bias_a_main);
% figure()
% subplot(2,1,1)
% plot(movmean(acc_main(end-100:end,:),1,100))
% subplot(2,1,2)
% plot(movmean(corrected_acc_main(end-100:end,:),1,100))
% 
% corrected_acc_payload = (acc_payload - est_bias_a_payload);
% figure()
% subplot(2,1,1)
% plot(movmean(acc_payload(end-100:end,:),1,100))
% subplot(2,1,2)
% plot(movmean(corrected_acc_payload(end-100:end,:),1,100))



corrected_gyr_main = (gyr_main - est_bias_g_main);
figure()
subplot(2,1,1)
plot(movmean(gyr_main(1:end,:),1,100))
subplot(2,1,2)
plot(movmean(corrected_gyr_main(1:end,:),1,100))

corrected_gyr_payload = (gyr_payload - est_bias_g_payload);
figure()
subplot(2,1,1)
plot(movmean(gyr_payload(1:end,:),1,100))
subplot(2,1,2)
plot(movmean(corrected_gyr_payload(1:end,:),1,100))





%% Plotting 
close all

ZVK_time = ZVK_time*1e-6;

figure(); title('bias accelerometer main')
        subplot(3,1,1)
        plot(ZVK_time,x(:,7), 'r', 'LineWidth',2);
        legend('bias accelerometro x')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,2)
        plot(ZVK_time,x(:,8), 'g','LineWidth',2);
        legend('bias accelerometro y')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,3)
        plot(ZVK_time,x(:,9),'b', 'LineWidth',2);
        legend('bias accelerometro z')
        xlabel('time [s]'); ylabel('bias');grid on
        grid on

figure(); title('bias accelerometer payload')
        subplot(3,1,1)
        plot(ZVK_time,x(:,10), 'r', 'LineWidth',2);
        legend('bias accelerometro x')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,2)
        plot(ZVK_time,x(:,11), 'g','LineWidth',2);
        legend('bias accelerometro y')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,3)
        plot(ZVK_time,x(:,12),'b', 'LineWidth',2);
        legend('bias accelerometro z')
        xlabel('time [s]'); ylabel('bias');grid on
        grid on

figure(); title('Bias Gyro main')
        subplot(3,1,1)
        plot(ZVK_time,x(:,19), 'r', 'LineWidth',2);
        legend('bias gyro x')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,2)
        plot(ZVK_time,x(:,20), 'g','LineWidth',2);
        legend('bias gyro y')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,3)
        plot(ZVK_time,x(:,21), 'b','LineWidth',2);
        legend('bias gyro z')
        xlabel('time [s]'); ylabel('bias');grid on

figure(); title('Bias Gyro payload')
        subplot(3,1,1)
        plot(ZVK_time,x(:,22), 'r', 'LineWidth',2);
        legend('bias gyro x')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,2)
        plot(ZVK_time,x(:,23), 'g','LineWidth',2);
        legend('bias gyro y')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,3)
        plot(ZVK_time,x(:,24), 'b','LineWidth',2);
        legend('bias gyro z')
        xlabel('time [s]'); ylabel('bias');grid on


% figure(); title('velocità')
%         subplot(3,1,1)
%         plot(ZVK_time,x(:,1), 'r', 'LineWidth',2);
%         legend('velocità x')
%         xlabel('time [s]'); ylabel('m/s');grid on
%         subplot(3,1,2)
%         plot(ZVK_time,x(:,2), 'g','LineWidth',2);
%         legend('velocità y')
%         xlabel('time [s]'); ylabel('m/s');grid on
%         subplot(3,1,3)
%         plot(ZVK_time,x(:,3),'b', 'LineWidth',2);
%         legend('velocità z')
%         xlabel('time [s]'); ylabel('m/s');grid on
% 
% figure(); title('accelerazione')
%         subplot(3,1,1)
%         plot(ZVK_time,x(:,4), 'r', 'LineWidth',2);
%         legend('accelerazione x')
%         xlabel('time [s]'); ylabel('m/s2');grid on
%         subplot(3,1,2)
%         plot(ZVK_time,x(:,5), 'g','LineWidth',2);
%         legend('accelerazione y')
%         xlabel('time [s]'); ylabel('m/s2');grid on
%         subplot(3,1,3)
%         plot(ZVK_time,x(:,6),'b', 'LineWidth',2);
%         legend('accelerazione z')
%         xlabel('time [s]'); ylabel('m/s2');grid on

% figure(); title('accelerazione vera');
%         subplot(3,1,1)
%         plot(IMU_main(2:length(t_plot)+1,2), 'r', 'LineWidth',2);
%         legend('accelerazione x')
%         xlabel('time [s]'); ylabel('m/s2');grid on
%         subplot(3,1,2)
%         plot(IMU_main(2:length(t_plot)+1,3), 'g','LineWidth',2);
%         legend('accelerazione y')
%         xlabel('time [s]'); ylabel('m/s2');grid on
%         subplot(3,1,3)
%         plot(IMU_main(2:length(t_plot)+1,4),'b', 'LineWidth',2);
%         legend('accelerazione z')
%         xlabel('time [s]'); ylabel('m/s2');grid on
% 
% figure(); title('angoli')
%         subplot(3,1,1)
%         plot(x(:,10), 'r', 'LineWidth',2);
%         legend('angoli x')
%         xlabel('time [s]'); ylabel('rad');grid on
%         subplot(3,1,2)
%         plot(x(:,11), 'g','LineWidth',2);
%         legend('angoli y')
%         xlabel('time [s]'); ylabel('rad');grid on
%         subplot(3,1,3)
%         plot(x(:,12),'b', 'LineWidth',2);
%         legend('angoli z')
%         xlabel('time [s]'); ylabel('rad');grid on

% figure(); title('omega')
%         subplot(3,1,1)
%         plot(ZVK_time,x(:,13), 'r', 'LineWidth',2);
%         legend('omega x')
%         xlabel('time [s]'); ylabel('rad/s');grid on
%         subplot(3,1,2)
%         plot(ZVK_time,x(:,14), 'g','LineWidth',2);
%         legend('omega y')
%         xlabel('time [s]'); ylabel('rad/s');grid on
%         subplot(3,1,3)
%         plot(ZVK_time,x(:,15),'b', 'LineWidth',2);
%         legend('omega z')
%         xlabel('time [s]'); ylabel('rad/s');grid on

% figure(); title('omega vera')
%         subplot(3,1,1)
%         plot(IMU_main(2:length(t_plot)+1,8), 'r', 'LineWidth',2);
%         legend('omega x')
%         xlabel('time [s]'); ylabel('rad/s');grid on
%         subplot(3,1,2)
%         plot(IMU_main(2:length(t_plot)+1,7), 'g','LineWidth',2);
%         legend('omega y')
%         xlabel('time [s]'); ylabel('rad/s');grid on
%         subplot(3,1,3)
%         plot(IMU_main(2:length(t_plot)+1,8),'b', 'LineWidth',2);
%         legend('omega z')
%         xlabel('time [s]'); ylabel('rad/s');grid on

