function [sensorDataMEA, sensorTotMEA] = run_MEA_LY(settingsMEA, sensorTotMEA, ...
    sensorDataMEA, t_chambPress, t_nas, t_imu, MEAFrequency, NAS_states, S_ref, ...
    combChambMeasures, imuAccelerometer, coeff_Cd,T1, CD_correction_shutDown, z0, N_prediction_threshold)

ign = 30;
% mass estimation
K_t = settingsMEA.K_t;
V_e = settingsMEA.V_e;
R_min = settingsMEA.Rs(1);
R_max = settingsMEA.Rs(2);

%define constants
g = 9.81;

dt = 1/MEAFrequency;
alpha = (R_max - R_min)/(100^2-30^2);
c = -alpha*30^2+R_min;

% prediction

% define time array for mea algorithm
t_mea = sensorTotMEA.time(end):1/MEAFrequency:T1;
% initialise state update
m = zeros(length(t_mea), 1);
m(1) = sensorDataMEA.x(end);
P(1) = sensorDataMEA.P(end);


z_nas = zeros(length(t_mea), 1);
vnorm_nas = zeros(length(t_mea), 1);
vz_nas = zeros(length(t_mea), 1);
z_nas(1) = NAS_states(end,3)  ;
vnorm_nas(1) = norm(NAS_states(end,4:6));
vz_nas(1) = NAS_states(end,6);

index_chambPress = sum(t_mea(1) >= t_chambPress);
index_NAS = sum(t_mea(1) >= t_nas);

for ii = 2:length(t_mea)
    % retrieve NAS data
    index_NAS = sum(t_mea(ii) >= t_nas);
    z_nas(ii,1) = NAS_states(index_NAS,3);
    vnorm_nas(ii,1) = norm(NAS_states(index_NAS,4:6));
    vz_nas(ii,1) = NAS_states(index_NAS,6);

    %propagation
    index_chambPress = sum(t_mea(ii) >= t_chambPress);
    index_imu  = sum(t_mea(ii) >= t_imu);
    if combChambMeasures(index_chambPress) > 1
        m(ii) = m(ii-1) - combChambMeasures(index_chambPress)*K_t*dt/V_e;


        P(ii) = P(ii-1) + settingsMEA.Q ;

        %correction

        cd = CD_correction_shutDown*getDrag(vnorm_nas(ii), -z_nas(ii), 0, coeff_Cd); %add correction shut_down??
        [~,~,~, rho] = atmosisa(-z_nas(ii));


        q = 0.5*rho*vnorm_nas(ii)^2; %dynamic pressure
        F_a = q*S_ref*cd;        %aerodynamic force

        %linearized model
        C2 = K_t .* (combChambMeasures(index_chambPress) ./ m(ii)) - g - F_a./m(ii);
        H = -K_t .* combChambMeasures(index_chambPress)./m(ii).^2 + F_a ./ m(ii).^2;

        R2 = (alpha*q + c);

        S = H.*P(ii).*H + R2;

        K = P(ii).*H ./ S;
        P(ii) = (1-K*H)*P(ii);

        if norm(imuAccelerometer(index_imu, :)) > ign && vnorm_nas(ii) > 40
            m(ii) = m(ii) + K.*(imuAccelerometer(index_imu, 1) -g - C2);
        end
    else
        m(ii) = m(ii);
    end

end


% compute CD
CD = CD_correction_shutDown*getDrag(vnorm_nas, -z_nas, 0, coeff_Cd);
[~,~,~,rho] = atmosisa(-z_nas);

propagation_steps = N_prediction_threshold - settingsMEA.counter_shutdown;
if propagation_steps >=1
    [z_pred, vz_pred] = PredictFuture(-z_nas,-vz_nas,  K_t .* combChambMeasures(index_chambPress), ...
        S_ref, CD, rho, m, dt, propagation_steps);
else
    z_pred = -z_nas;
    vz_pred = -vz_nas;
end

predicted_apogee = z_pred-z0 + 1./(2.*( 0.5.*rho .* CD * S_ref ./ m))...
    .* log(1 + (vz_pred.^2 .* (0.5 .* rho .* CD .* S_ref) ./ m) ./ 9.81 );

% update local state
sensorDataMEA.time = t_mea;
sensorDataMEA.x = m;
sensorDataMEA.P = P;
sensorDataMEA.predicted_apogee = predicted_apogee;

% update total state
sensorTotMEA.mass(sensorTotMEA.n_old:sensorTotMEA.n_old + length(sensorDataMEA.x)-2) = sensorDataMEA.x(2:end);
sensorTotMEA.prediction(sensorTotMEA.n_old:sensorTotMEA.n_old + length(sensorDataMEA.x)-2) = sensorDataMEA.predicted_apogee(2:end);
sensorTotMEA.time(sensorTotMEA.n_old:sensorTotMEA.n_old + length(sensorDataMEA.x)-2) = sensorDataMEA.time(2:end);
sensorTotMEA.n_old = sensorTotMEA.n_old + size(m,1) - 1;

end