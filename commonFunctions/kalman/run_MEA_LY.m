function [sensorData,sensorTot] = run_MEA_LY(sensorData,sensorTot,settings,contSettings,T1)

%range for mass in case MEA diverges
m_max = 40; 
m_min = 20;
M_est = 29; %probable mass used to predict the apogee in this case

ign = 40;
% mass estimation
K_t = settings.mea.K_t;
V_e = settings.mea.V_e+200;
R_min = settings.mea.Rs(1);
R_max = settings.mea.Rs(2);
[~,~,P_0] = atmosisa(200);

%define constants
g = 9.81;
dt = 0.02;
alpha = (R_max - R_min)/(100^2-30^2);
c = -alpha*30^2+R_min;

% define time array for mea algorithm
t_mea = sensorTot.mea.time(end):1/settings.frequencies.MEAFrequency:T1;
% define time array for sensors
t_chambPress = sensorTot.comb_chamber.time;%(sensorTot.comb_chamber.time >= T1);
t_nas = sensorTot.nas.time;%(sensorTot.nas.time>= T1); % we need also nas to estimate cd etc
t_imu = sensorTot.imu.time;
% initialise state update
x(1,:) = sensorData.mea.estimated_mass(end);
P(1) = sensorData.mea.P(end);
predicted_apogee = 0;

z_nas = zeros(length(t_mea), 1);
vnorm_nas = zeros(length(t_mea), 1);
vz_nas = zeros(length(t_mea), 1);
z_nas(1) = sensorData.nas.states(end,3);
vnorm_nas(1) = norm(sensorData.nas.states(end,4:6));
vz_nas(1) = sensorData.nas.states(end,6);

index_chambPress = 0;

for ii = 2:length(t_mea)
    index_chambPress = sum(t_mea(ii) >= t_chambPress);
    index_imu = sum(t_mea(ii) >= t_imu);
    index_nas = sum(t_mea(ii) >= t_nas);

    z_nas(ii,1) = sensorTot.nas.states(index_nas,3);
    vnorm_nas(ii,1) = norm(sensorTot.nas.states(index_nas,4:6));
    vz_nas(ii,1) = sensorTot.nas.states(index_nas,6);

    if sensorTot.comb_chamber.measures(index_chambPress)  > 1
        % prediction
        x(ii,:) = x(ii-1,:) - sensorTot.comb_chamber.measures(index_chambPress)*K_t*dt/V_e;
        P(ii) = P(ii-1) + settings.mea.Q ;
        if norm(sensorTot.imu.accelerometer_measures(index_imu, :)) > ign && vnorm_nas(ii) > 40

            % model
            cd = 1*getDrag(vnorm_nas(ii), -z_nas(ii), 0, contSettings.coeff_Cd); %add correction shut_down??
            [~,~,P_e, rho] = atmosisa(-z_nas(ii));
            q = 0.5*rho*vnorm_nas(ii)^2; %dynamic pressure
            F_a = q*settings.S*cd;        %aerodynamic force

            if  -z_nas(ii,1)> 800
                F_s = (P_0-P_e)*settings.motor.Ae ;
            else 
                F_s = 0;
            end

            C2 = (K_t * sensorTot.comb_chamber.measures(index_chambPress) + F_s)/x(ii) - g - F_a/x(ii);
            H = -(K_t * sensorTot.comb_chamber.measures(index_chambPress)+ F_s)/x(ii)^2 + F_a/x(ii)^2;

            %correction
            R2 = (alpha*q + c);
           
            S = H*P(ii)*H' + R2;
            if ~det(S)<1e-3
                K = P(ii)*H' / S;
                P(ii) = (1-K*H)*P(ii);
                x(ii,:) = x(ii,:) + K.*(sensorTot.imu.accelerometer_measures(index_imu, 1) - g - C2);
            end

        end

    else
        x(ii,:) = x(ii-1,:);
    end

        % propagate apogee
    CD = settings.CD_correction_shutDown*getDrag(vnorm_nas(ii), -z_nas(ii), 0, contSettings.coeff_Cd); % coeffs potrebbe essere settings.coeffs
    [~,~,~,rho] = atmosisa(-z_nas(ii));
    
    
    propagation_steps = contSettings.N_prediction_threshold - settings.mea.counter_shutdown;
    
    if x(ii, :) > m_max || x(ii, :) < m_min
        m_pred = M_est;
    else 
        m_pred = x(ii, :);
    end
    
    if propagation_steps >=1
        [z_pred, vz_pred] = PredictFuture(-z_nas(ii),-vz_nas(ii), ...
            K_t .* sensorTot.comb_chamber.measures(index_chambPress), ...
            settings.S, CD, rho,m_pred, dt, propagation_steps);
    else
        z_pred = -z_nas(ii);
        vz_pred = -vz_nas(ii);
    end
    
    predicted_apogee(ii) = z_pred-settings.z0 + 1./(2.*( 0.5.*rho .* CD * settings.S ./ m_pred))...
        .* log(1 + (vz_pred.^2 .* (0.5 .* rho .* CD .* settings.S) ./ m_pred) ./ 9.81 );

end

estimated_mass = x(:,1);

% update local state
sensorData.mea.time = t_mea;
sensorData.mea.P = P;
sensorData.mea.predicted_apogee = predicted_apogee;
sensorData.mea.estimated_mass = estimated_mass;

% update total state
sensorTot.mea.mass(sensorTot.mea.n_old:sensorTot.mea.n_old + length(sensorData.mea.estimated_mass)-2) = sensorData.mea.estimated_mass(2:end);
sensorTot.mea.prediction(sensorTot.mea.n_old:sensorTot.mea.n_old + length(sensorData.mea.estimated_mass)-2) = sensorData.mea.predicted_apogee(2:end);
sensorTot.mea.time(sensorTot.mea.n_old:sensorTot.mea.n_old + length(sensorData.mea.estimated_mass)-2) = sensorData.mea.time(2:end);
sensorTot.mea.n_old = sensorTot.mea.n_old + size(x,1) - 1;
