function [sensorTot] = run_ZVK(Tf, sensorData, sensorTot, settings, environment)


% recall zvk time
t_zvk = sensorTot.zvk.time(end):1/settings.frequencies.ZVKFrequency:Tf;

% preallocate update
x           =   zeros(length(t_zvk),16);        % Pre-allocation of pos, vel, quaternion and biases     [4quat 3r 3v 3beta_g 3beta_a]
p           =   zeros(length(t_zvk),18);        % Pre-allocation of cal parameters                      [9W_g 9W_a]  oppure  [3lam_g 3mu_g 3nu_g + idem per _a]
z           =   zeros(length(t_zvk),34);

% note that covariance has one state less because error state has  3 small
% deviation angles instead of quaternions
% P_xx        =   zeros(15,15,length(t_zvk));
% P_xp        =   zeros(18,18,length(t_zvk));       % Pre-allocation of the covariance matrix
P_z         =   zeros(33,33,length(t_zvk));


% first update
x(1,:)      =   sensorData.zvk.states(end,1:16);                 % Allocation of the initial value
p(1,:)      =   sensorData.zvk.states(end,17:end);
z(1,:)      =   [x(1,:), p(1,:)];

% P_xx(:,:,1)  =   sensorData.zvk.P(1:15,1:15,end);
% P_xp(:,:,1)  =   sensorData.zvk.P(16:33,1:15,end);
P_z(:,:,1)   =   sensorData.zvk.P(:,:,end);


if length(t_zvk) > 1
    dt_k    =   t_zvk(2)-t_zvk(1);          % Kalman time step
    
    % Time vectors agumentation
    t_imutemp  = [sensorTot.imu.time];
    
    for ii = 2:length(t_zvk)
        
        %%% Prediction
        index_imu   =  sum(t_zvk(ii) >= t_imutemp);
        
        a_b_m = sensorTot.imu.accelerometer_measures(index_imu,:);
        om_b_m = sensorTot.imu.gyro_measures(index_imu,:);

        [x(ii,:), P_z(:,:,ii)] = predictorZVK( x(ii-1,:), P_z(:,:,ii-1), a_b_m, om_b_m, dt_k, settings.zvk);



        
        %%% Correction
        correctorZVK( x(ii,:), P_z(:,:,ii) );

        
        
        
    end

    sensorData.zvk.states = [x,p];
    sensorData.zvk.time = t_zvk;

    sensorTot.zvk.states(sensorTot.zvk.n_old:sensorTot.zvk.n_old + size(sensorData.zvk.states(:,1),1)-2, 1:16 )  = sensorData.zvk.states(2:end,1:16); % ZVK output
    sensorTot.zvk.time( sensorTot.zvk.n_old : sensorTot.zvk.n_old+size(sensorData.zvk.states(:,1),1)-2 )    = sensorData.zvk.time(2:end); % ZVK time output
    % sensorTot.zvk.time(:)
    sensorTot.zvk.n_old = sensorTot.zvk.n_old + size(sensorData.zvk.states,1)-1;
end

end