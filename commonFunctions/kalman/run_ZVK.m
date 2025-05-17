function [sensorData, sensorTot] = run_ZVK(Tf, sensorData, sensorTot, settings, mag_NED, environment)


% recall zvk time
t_zvk = sensorTot.zvk.time(end):1/settings.frequencies.ZVKFrequency:Tf;

% preallocate update
x =   zeros(length(t_zvk),18);        % Pre-allocation of pos, vel, quaternion and biases     [3v 3a 3beta_a || 3angle 3om 3beta_g]
% note that covariance has one state less because error state has  3 small
% deviation angles instead of quaternions
P =   zeros(18,18,length(t_zvk));


% first update
x(1,:)      =   sensorData.zvk.states(end,:);                 % Allocation of the initial value


P(:,:,1)    =   sensorData.zvk.P(:,:,end);

% disp(P(:,:,1))


if length(t_zvk) > 1
    dt_k    =   t_zvk(2)-t_zvk(1);          % Kalman time step
    
    % Time vectors agumentation
    t_imutemp  = [sensorTot.imu.time];
    
    for ii = 2:length(t_zvk)
        
        %%% Prediction
        index_imu   =  sum(t_zvk(ii) >= t_imutemp);
        
        a_b_m = sensorTot.imu.accelerometer_measures(index_imu,:);
        om_b_m = sensorTot.imu.gyro_measures(index_imu,:);

        [x(ii,:), P(:,:,ii)] = predictorZVK( x(ii-1,:), P(:,:,ii-1), sensorTot.zvk.quat, a_b_m, om_b_m, dt_k, settings.zvk);


        
        mag_meas = sensorTot.imu.magnetometer_measures(index_imu,:);
        
        %%% Correction
        [x(ii,:), P(:,:,ii)] = correctorZVK( x(ii,:), P(:,:,ii), sensorTot.zvk.quat, a_b_m, om_b_m, mag_meas, mag_NED, settings.zvk);

       
        
    end
    
    sensorData.zvk.states = x;
    sensorData.zvk.P = P;
    sensorData.zvk.time = t_zvk;

    sensorTot.zvk.P(1:18, 1:18, sensorTot.zvk.n_old:sensorTot.zvk.n_old + size(sensorData.zvk.P,3)-2 )   = sensorData.zvk.P(:,:,2:end);
    sensorTot.zvk.states(sensorTot.zvk.n_old:sensorTot.zvk.n_old + size(sensorData.zvk.states,1)-2, 1:18 )  = sensorData.zvk.states(2:end,:); % ZVK output
    sensorTot.zvk.time( sensorTot.zvk.n_old : sensorTot.zvk.n_old+size(sensorData.zvk.states,1)-2 )    = sensorData.zvk.time(2:end); % ZVK time output
    % sensorTot.zvk.time(:)
    sensorTot.zvk.n_old = sensorTot.zvk.n_old + size(sensorData.zvk.states,1)-1;
end

end