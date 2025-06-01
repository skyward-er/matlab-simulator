%% Load variables
clear variables;
close all;
clc;


addpath(genpath("logs"))


IMU_main = table2array(readtable("main_Boardcore_IMUDataEUROC")); % [tempo*e-6 acc_x acc_y acc_z w_x,w_y,w_z]

dt = mean(diff(IMU_main(:,1)));

idxend_main = find(IMU_main(:,2)>20,1,'first');
idxend_main = round(idxend_main - 5e6/dt);

time_acc_main    = IMU_main(1:idxend_main,1)*1e-6;
acc_main         = IMU_main(1:idxend_main,2:4);
time_gyr_main    = IMU_main(1:idxend_main,5)*1e-6;
gyr_main         = IMU_main(1:idxend_main,6:8);

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


dt = mean(diff(IMU_payload(:,1)));

idxend_payload = find(IMU_payload(:,2)>20,1,'first');
idxend_payload = round(idxend_payload - 5e6/dt);

time_acc_payload    = IMU_payload(1:idxend_payload,1)*1e-6;
acc_payload         = IMU_payload(1:idxend_payload,2:4);
time_gyr_payload    = IMU_payload(1:idxend_payload,5)*1e-6;
gyr_payload         = IMU_payload(1:idxend_payload,6:8);

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




%% Initial params
x0 = zeros(1,24);
% vel, acc, bias_acc_main, bias_acc_payload, theta, omega, bias_gyro_main, bias_gyro_payoad

% Estimated noise standard deviations
sigma_gyro         = 5e-3;     % [rad/s]   gyroscope std
sigma_beta_g       = 1e-5;     % [rad/s]   gyroscope bias std
sigma_acc          = 4e-2;     % [m/s^2]   accelerometer std
sigma_beta_acc     = 1e-5;     % [m/s^2]   accelerometer bias std

Q = diag([
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
R_fake      = diag([
                (1e-6)^2                        * ones(3,1);        % FAKE zero velocity [m/s]
                (1e-6)^2                        * ones(3,1);        % FAKE angle [rad]
            ]);
R_acc       = diag( sigma_acc^2 * ones(3,1));   % accelerometer [m/s^2]
R_gyro      = diag(sigma_gyro^2 * ones(3,1));   % angular velocity [rad/s]


Q0 = angle2quat(133*pi/180, 85*pi/180, 0*pi/180, 'ZYX')';
% error = [1 0.5*deg2rad([0 0 0]) ];
% Q0 = quatmultiply(Q0', error);
%Q0 = Q0';
quat = [Q0(2:4); Q0(1)];

% load("Ksave");

%% Initializzation
x = x0;
P = P0;

%% SIMU_mainlation

t_IMU_main      = IMU_main(1:idxend_main,1);
t_IMU_payload   = IMU_payload(1:idxend_payload,1);
ii = 2;

ZVK_freq = 50; %[Hz]
ZVK_sampling_time = 1/ZVK_freq*1e6;

last_t = t_IMU_main(1);

t_start = t_IMU_main(1);
t_end   = t_IMU_main(end);

ZVK_time = [];

for t = t_start:ZVK_sampling_time:t_end

    %%% Prediction
    dt = (t-last_t)*1e-6;
    [x(ii,:), P(:,:,ii)] = predictor_zvk( x(ii-1,:), P(:,:,ii-1), dt, Q);


    %%% Correction
    % FAKE
    [x(ii,[1:3,13:15]), P([1:3,13:15],[1:3,13:15],ii)] = corrector_zvk( x(ii,[1:3,13:15]), P([1:3,13:15],[1:3,13:15],ii), quat, R_fake);


    % MAIN
    index_imu = sum(t >= t_IMU_main);
    IMU_main_measurment = IMU_main(index_imu,[2:4 6:8]);
    acc_meas = IMU_main_measurment(1:3);
    omeg_meas = IMU_main_measurment(4:6);
    
    [x(ii,[4:6,7:9]), P([4:6,7:9],[4:6,7:9],ii)]                = corrector_zvk_acc(  x(ii,[4:6,7:9]),      P([4:6,7:9],[4:6,7:9],ii),          quat, acc_meas, R_acc);
    [x(ii,[16:18,19:21]), P([16:18,19:21],[16:18,19:21],ii)]    = corrector_zvk_gyro( x(ii,[16:18,19:21]),  P([16:18,19:21],[16:18,19:21],ii),  quat, omeg_meas, R_gyro);
    
    % PAYLOAD
    index_imu = sum(t >= t_IMU_payload);
    IMU_payload_measurment = IMU_payload(index_imu,[2:4 6:8]);
    acc_meas = IMU_payload_measurment(1:3);
    omeg_meas = IMU_payload_measurment(4:6);

    [x(ii,[4:6,10:12]), P([4:6,10:12],[4:6,10:12],ii)]          = corrector_zvk_acc(  x(ii,[4:6,10:12]),    P([4:6,10:12],[4:6,10:12],ii),      quat, acc_meas, R_acc);
    [x(ii,[16:18,22:24]), P([16:18,22:24],[16:18,22:24],ii)]    = corrector_zvk_gyro( x(ii,[16:18,22:24]),  P([16:18,22:24],[16:18,22:24],ii),  quat, omeg_meas, R_gyro);
    

    ii = ii+1;
    last_t = t;

    ZVK_time = [ZVK_time, t];
end
x = x(2:end,:);



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






% %%
% Ksave = K(:,:,end);
% save("Ksave")
