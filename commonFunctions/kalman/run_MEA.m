function [sensorData,sensorTot] = run_MEA(sensorData,sensorTot,settings,contSettings,u,T1,engineT0,environment,rocket, mission)


% mass estimation
A = settings.mea.engine_model_A1;
B = settings.mea.engine_model_B1;
C = settings.mea.engine_model_C1;

% define time array for mea algorithm
t_mea = sensorTot.mea.time(end):1/settings.frequencies.MEAFrequency:T1;

% define time array for sensors
t_chambPress = sensorTot.comb_chamber.time;%(sensorTot.comb_chamber.time >= T1);
t_nas = sensorTot.nas.time;%(sensorTot.nas.time>= T1); % we need also nas to estimate cd etc
t_imu = sensorTot.imu.time;

% initialise state update
x(1,:) = sensorData.mea.x(end,:);
P(:,:,1) = sensorData.mea.P(:,:,end);
P_acc(:,:,1) = sensorData.mea.P_acc(:,:,end);
predicted_apogee = 0;
z_nas = zeros(length(t_mea), 1);
vnorm_nas = zeros(length(t_mea), 1);
vz_nas = zeros(length(t_mea), 1);
z_nas(1) = sensorData.nas.states(end,3);
vnorm_nas(1) = norm(sensorData.nas.states(end,4:6));
vz_nas(1) = -sensorData.nas.states(end,6);

coeffs = contSettings.coeffs;
 

for ii = 2:length(t_mea)
    index_chambPress = sum(t_mea(ii) >= t_chambPress);
    index_imu = sum(t_mea(ii) >= t_imu);
    index_nas = sum(t_mea(ii) >= t_nas);

    z_nas(ii,1) = sensorTot.nas.states(index_nas,3);
    vnorm_nas(ii,1) = norm(sensorTot.nas.states(index_nas,4:6));
    vz_nas(ii,1) = -sensorTot.nas.states(index_nas,6);

    % prediction
    x(ii,:) = (A*x(ii-1,:)' + B*u)'; % x is a row but to apply matrix product we need it column, therefore the transpositions
    P(:,:,ii) = A*P(:,:,ii-1)*A' + settings.mea.Q;

    % barometer correction
    if sensorTot.comb_chamber.measures(index_chambPress)  > 1
        S = C*P(:,:,ii)*C' + settings.mea.R;
        if ~det(S)<1e-3
            K = P(:,:,ii)*C' / S; % if you want to try with constant gain [0.267161;-0.10199;-0.000205604 ];
            P(:,:,ii) = (eye(3)-K*C)*P(:,:,ii);
            x(ii,:) = x(ii,:)' + K* (sensorTot.comb_chamber.measures(index_chambPress) -  C * x(ii,:)'); % /1000 to have the measure in bar
        end
    end
    %accelerometer correction (not for 2023)
    if contains(mission.name, '2024')
        K_t = settings.mea.K_t;
        alpha = settings.mea.alpha;
        c = settings.mea.c;
        P_0 = settings.mea.P0;
        acc_threshold = settings.mea.acc_threshold;
        vel_threshold = settings.mea.vel_threshold;
        mass_max = settings.mea.mass_interval(2);
        mass_min = settings.mea.mass_interval(1);

        if norm(sensorTot.imu.accelerometer_measures(index_imu, :)) > acc_threshold...
                && vz_nas(ii) > vel_threshold

            cd = 1*getDrag(vz_nas(ii), -z_nas(ii), 0, coeffs); %add correction shut_down??
            [~,~,P_e, rho] = computeAtmosphericData(-z_nas(ii));
            q = 0.5*rho*vz_nas(ii)^2; %dynamic pressure
            F_a = q*rocket.crossSection*cd;       %aerodynamic force

            if  -z_nas(ii,1)> 800
                F_s = (P_0-P_e)*rocket.motor.ae;
            else
                F_s = 0;
            end

            y_est = (K_t * C*x(ii, :)' + F_s)/x(ii,3)  - F_a/x(ii, 3);
            H = [K_t*C(1)/x(ii, 3), ...
                K_t*C(2)/x(ii, 3),...
                -( K_t*C*x(ii,:)'+ F_s - F_a)/x(ii, 3)^2];

            R2 = (alpha*q + c);

            S = H*P(:,:,ii)*H' + R2;

            if ~det(S)<1e-3
                K = P(:,:,ii)*H' / S;
                P(:,:,ii) = (eye(3)-K*H)*P(:,:,ii);
                x(ii,:) = x(ii,:)' + K.*(sensorTot.imu.accelerometer_measures(index_imu, 1) - y_est);
            end

        end
        
        % use only reasonable masses to predict the apogee
        if x(ii,3) > mass_max
            mass = mass_max;
        elseif x(ii,3) < mass_min
            mass = mass_min;
        else
            mass = x(ii,3);
        end

    else
        mass = x(ii,3);
    end

    %propagate apogee
    CD = settings.CD_correction_shutDown*getDrag(vz_nas(ii), -z_nas(ii), 0, coeffs); % coeffs potrebbe essere settings.coeffs
    [~,~,~,rho] = computeAtmosphericData(-z_nas(ii));

    propagation_steps = 0;%contSettings.N_prediction_threshold - settings.mea.counter_shutdown;
    if propagation_steps >=1
        [z_pred, vz_pred] = PropagateState(-z_nas(ii),-vz_nas(ii), ...
            K_t .* sensorTot.comb_chamber.measures(index_chambPress), ...
            rocket.crossSection, CD, rho,x(ii, 3), 0.02, propagation_steps);
    else
        z_pred = -z_nas(ii);
        vz_pred = -vz_nas(ii);
    end

    predicted_apogee(ii) = z_pred + 1./(2.*( 0.5.*rho .* CD * rocket.crossSection ./ mass))...
        .* log(1 + (vz_pred.^2 .* (0.5 .* rho .* CD .* rocket.crossSection) ./ mass) ./ 9.81 );

    % retrieve NAS data
    index_NAS = sum(t_mea(ii) >= t_nas);
    z_nas(ii,1) = sensorTot.nas.states(index_NAS,3);
    vnorm_nas(ii,1) = norm(sensorTot.nas.states(index_NAS,4:6));
    vz_nas(ii,1) = sensorTot.nas.states(index_NAS,6);
end
% pressure estimation
estimated_pressure = C*x';

% mass estimation
estimated_mass = x(:,3);


% update local state
sensorData.mea.time = t_mea;
sensorData.mea.x = x;
sensorData.mea.P = P;
sensorData.mea.P_acc = P_acc;
sensorData.mea.predicted_apogee = predicted_apogee;
sensorData.mea.estimated_mass = estimated_mass;
sensorData.mea.estimated_pressure = estimated_pressure;

% update total state
sensorTot.mea.pressure(sensorTot.mea.n_old:sensorTot.mea.n_old + size(sensorData.mea.x(:,1),1)-2) = sensorData.mea.estimated_pressure(2:end);
sensorTot.mea.mass(sensorTot.mea.n_old:sensorTot.mea.n_old + size(sensorData.mea.x(:,1),1)-2) = sensorData.mea.estimated_mass(2:end);
sensorTot.mea.prediction(sensorTot.mea.n_old:sensorTot.mea.n_old + size(sensorData.mea.x(:,1),1)-2) = sensorData.mea.predicted_apogee(2:end);
sensorTot.mea.time(sensorTot.mea.n_old:sensorTot.mea.n_old + size(sensorData.mea.x(:,1),1)-2) = sensorData.mea.time(2:end);
sensorTot.mea.n_old = sensorTot.mea.n_old + size(sensorData.mea.x,1) -1;


end