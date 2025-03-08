function [] = ZVK(Tf, mag_NED, sensorData, sensorTot, settings, environment)

% settings
settings.frequencies.ZVKFrequency = 50;



% recall zvk time
t_zvk = sensorTot.zvk.time(end):1/settings.frequencies.ZVKFrequency:Tf;


% preallocate update
x           =   zeros(length(t_zvk),6);         % Pre-allocation of corrected estimation
p           =   zeros(length(t_zvk),7);         % Pre-allocation of quaternions and biases
z           =   zeros(length(t_zvk),13);

P_xx        =   zeros(12,12,length(t_zvk));
P_pp        =   zeros(6,6,length(t_zvk));       % Pre-allocation of the covariance matrix
P_z         =   zeros(6,6,length(t_zvk));


% first update
x(1,:)      =   sensorData.zvk.states(end,1:6);                 % Allocation of the initial value
p(1,:)      =   sensorData.zvk.states(end,7:13);
z(1,:)      =   [x(1,:), p(1,:)];

P_xx(:,:,1) =   sensorData.zvk.P(1:6,1:6,end);
P_pp(:,:,1) =   sensorData.zvk.P(7:12,7:12,end);
P_z(:,:,1)  =   sensorData.zvk.P(:,:,end);



if length(t_zvk) > 1
    dt_k    =   t_zvk(2)-t_zvk(1);          % Kalman time step
    
    % Time vectors agumentation
    t_gpstemp  = [sensorTot.gps.time];
    t_barotemp = [sensorTot.barometer.time];
    t_imutemp  = [sensorTot.imu.time];
    t_pittemp  = [sensorTot.pitot.time];
    
    for ii = 2:length(t_zvk)
        
        %%% Prediction
        index_imu   =  sum(t_zvk(ii) >= t_imutemp);

        %%%% PREDICTOR %%%% PREDICTOR %%%% PREDICTOR %%%% 
        % [x_lin(ii,:),~,P_lin(:,:,ii)] = predictorLinear2(x_lin(ii-1,:),P_lin(:,:,ii-1), dt_k,sensorTot.imu.accelerometer_measures(index_imu,:),xq(ii-1,1:4),nas.QLinear);
        % [xq(ii,:),P_q(:,:,ii)]        = predictorQuat(xq(ii-1,:),P_q(:,:,ii-1), sensorTot.imu.gyro_measures(index_imu,:),dt_k,nas.Qq);
        %%%% PREDICTOR %%%% PREDICTOR %%%% PREDICTOR %%%% 



        %%% Correction
        
        
    end

end

end