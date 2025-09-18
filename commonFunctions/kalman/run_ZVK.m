function [sensorData, sensorTot] = run_ZVK(Tf, sensorData, sensorTot, settings, mag_NED, environment)


% recall zvk time
t_zvk = sensorTot.zvk.time(end):1/settings.frequencies.ZVKFrequency:Tf;

% preallocate update
x =   zeros(length(t_zvk),24);        % Pre-allocation of pos, vel, quaternion and biases     [4quat 3v 3r 3beta_a 3beta_g]

P =   zeros(24,24,length(t_zvk));


% first update
x(1,:)      =   sensorData.zvk.states(end,:);                 % Allocation of the initial value

P(:,:,1)    =   sensorData.zvk.P(:,:,end);



if length(t_zvk) > 1
    dt_k    =   t_zvk(2)-t_zvk(1);          % Kalman time step
    
    % Time vectors agumentation
    t_imutemp  = [sensorTot.imu.time];
    
    for ii = 2:length(t_zvk)
        
        index_imu   =  sum(t_zvk(ii) >= t_imutemp);
        
        % MAIN measures
        acc_meas_m = sensorTot.imu.accelerometer_measures(index_imu,:);
        omeg_meas_m = sensorTot.imu.gyro_measures(index_imu,:);

        %PAYLOAD measures
        acc_meas_p = sensorTot.imu.accelerometer_1_measures(index_imu,:);
        omeg_meas_p = sensorTot.imu.gyro_1_measures(index_imu,:);
        
        %%% Prediction
        [x(ii,:), P(:,:,ii)] = predictorZVK( x(ii-1,:), P(:,:,ii-1), dt_k, settings.zvk.Q);

        %%% Correction
        %FAKE
        [x(ii,[1:3,13:15]), P([1:3,13:15],[1:3,13:15],ii)] = correctorZVK( x(ii,[1:3,13:15]), P([1:3,13:15],[1:3,13:15],ii), settings.zvk.R_fake);
        
        %MAIN
        [x(ii,[4:6,7:9]), P([4:6,7:9],[4:6,7:9],ii)] = correctorZVK_acc( x(ii,[4:6,7:9]), P([4:6,7:9],[4:6,7:9],ii), settings.zvk.quat, acc_meas_m, settings.zvk.R_acc);
        [x(ii,[16:18,19:21]), P([16:18,19:21],[16:18,19:21],ii)] = correctorZVK_gyro( x(ii,[16:18,19:21]), P([16:18,19:21],[16:18,19:21],ii), omeg_meas_m, settings.zvk.R_gyro);
        
        %PAYLOAD
        [x(ii,[4:6,10:12]), P([4:6,10:12],[4:6,10:12],ii)] = correctorZVK_acc( x(ii,[4:6,10:12]), P([4:6,10:12],[4:6,10:12],ii), settings.zvk.quat, acc_meas_p, settings.zvk.R_acc);
        [x(ii,[16:18,22:24]), P([16:18,22:24],[16:18,22:24],ii)] = correctorZVK_gyro( x(ii,[16:18,22:24]), P([16:18,22:24],[16:18,22:24],ii), omeg_meas_p, settings.zvk.R_gyro);
        
    end
    
    sensorData.zvk.states = x;
    sensorData.zvk.P = P;
    sensorData.zvk.time = t_zvk;

    sensorTot.zvk.P(1:24, 1:24, sensorTot.zvk.n_old:sensorTot.zvk.n_old + size(sensorData.zvk.P,3)-2 )   = sensorData.zvk.P(:,:,2:end);
    sensorTot.zvk.states(sensorTot.zvk.n_old:sensorTot.zvk.n_old + size(sensorData.zvk.states,1)-2, 1:24 )  = sensorData.zvk.states(2:end,:); % ZVK output
    sensorTot.zvk.time( sensorTot.zvk.n_old : sensorTot.zvk.n_old+size(sensorData.zvk.states,1)-2 )    = sensorData.zvk.time(2:end); % ZVK time output
    % sensorTot.zvk.time(:)
    sensorTot.zvk.n_old = sensorTot.zvk.n_old + size(sensorData.zvk.states,1)-1;
end

end