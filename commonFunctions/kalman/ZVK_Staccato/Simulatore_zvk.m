%% Load variables
clear variables;
close all;
clc;

IMU = table2array(readtable('IMU.csv')); % [tempo*e-6 acc_x acc_y acc_z w_x,w_y,w_z]
dt_IMU = 1000;
lastImuIdx = 0;

t_end = 3870000;

%% Initial params
x0 = zeros(1,18);

% Estimated noise standard deviations
sigma_gyro         = 5e-5;     % [rad/s]   gyroscope variance               NAS
sigma_beta_g       = 1e-6;     % [rad/s]   gyroscope bias variance         NAS
sigma_acc          = 5e-2;     % [m/s^2]   accelerometer variance           NAS
sigma_beta_acc     = 1e-5;     % [m/s^2]   accelerometer bias variance     BOH
sigma_mag          = 3;       % [mgauss]  magnetometer variance            NAS = 10?????   mentre sito STM di 3mGs RMS noise


Q = diag([
            (1e-6)^2                            * ones(3,1);    % vel
            (1e-6)^2                            * ones(3,1);    % acceleration
            sigma_beta_acc^2                    * ones(3,1);    % bias accelerometer
            (1e-6)^2                            * ones(3,1);    % theta
            (1e-6)^2                            * ones(3,1);    % angular velocity
            sigma_beta_g^2                      * ones(3,1);    % bias gyroscope
        ]);


% Initial state covariance matrix P
P0 = diag([
        (1e-5)^2    * ones(3,1);    % velocity uncertainty [m/s]
        (1e-5)^2    * ones(3,1);    % acceleration [m/s^2]
        (1e-1)^2    * ones(3,1);    % accelerometer bias [m/s^2]
        (1e-5)^2    * ones(3,1);    % ang velocity [rad/s]
        (1e-5)^2    * ones(3,1);    % ang acceleration [rad/s^2]
        (1e-3)^2    * ones(3,1)     % gyro bias [rad/s]
        ]);


% Measurement noise covariance matrix R
R = diag([
            (1e-6)^2                        * ones(3,1);        % FAKE zero velocity [m/s]
            sigma_acc^2                     * ones(3,1);        % accelerometer [m/s^2]
            (1e-6)^2                        * ones(3,1);        % FAKE angle [rad]
            sigma_gyro^2                    * ones(3,1)         % angular velocity [rad/s]
        ]);

Q0 = angle2quat(133*pi/180, 85*pi/180, 0*pi/180, 'ZYX')';
error = [1 0.5*deg2rad([0 1 0]) ];
Q0 = quatmultiply(Q0', error);
Q0 = Q0';
quat = [Q0(2:4); Q0(1)];

%% Initializzation
x = x0;
P = P0;


%% Simulation
ii = 2;
for t = 0:2000:t_end
    
    dt = t*1e-6;
    % Prende la misura dei sensori

    imu_idx = sum(IMU(:,1) <= t); %conta quanti campioni sono stati presi
    if imu_idx > lastImuIdx % se c'è una nuova misura
        IMU_measurment = IMU(imu_idx, 2:7); %aggiorna la misura
        lastImuIdx = imu_idx;
    else
        IMU_measurment = IMU(lastImuIdx,2:7); %altrimenti lascia il vecchio indice
    end
    acc_meas = IMU_measurment(1:3);
    omeg_meas = IMU_measurment(4:6);
    
    %%% Prediction
    [x(ii,:), P(:,:,ii)] = predictor_zvk( x(ii-1,:), P(:,:,ii-1), dt, Q);
        
    %%% Correction
    [x(ii,:), P(:,:,ii)] = corrector_zvk( x(ii,:), P(:,:,ii), quat, acc_meas, omeg_meas, R);
    x(ii,:)

    ii = ii+1;
end
x = x(2:end,:);
%% Plotting 

t_plot = (0:2000:t_end)*1e-6;

figure()
        subplot(3,1,1)
        plot(t_plot,x(:,7), 'r', 'LineWidth',2);
        legend('bias accelerometro x')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,2)
        plot(t_plot,x(:,8), 'g','LineWidth',2);
        legend('bias accelerometro y')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,3)
        plot(t_plot,x(:,9),'b', 'LineWidth',2);
        legend('bias acceleromtro z')
        xlabel('time [s]'); ylabel('bias');grid on
        grid on
        
figure()
        subplot(3,1,1)
        plot(t_plot,x(:,16), 'r', 'LineWidth',2);
        legend('bias gyro x')
        xlabel('time [s]'); ylabel('bias');grid on
        subplot(3,1,2)
        xlabel('time [s]'); ylabel('bias');grid on
        plot(t_plot,x(:,17), 'g','LineWidth',2);
        legend('bias gyro y')
        subplot(3,1,3)
        plot(t_plot,x(:,18), 'b','LineWidth',2);
        legend('bias gyro z')
        xlabel('time [s]'); ylabel('bias');grid on
        


        estimated_bias_acc          = x(end,7:9);
        estimated_bias_acc_sigma    = sqrt( diag(P(7:9, 7:9, end))' ); 
        
        estimated_bias_gyro         = x(end,16:18);
        estimated_bias_gyro_sigma   = sqrt( diag(P(16:18, 16:18, end))' );