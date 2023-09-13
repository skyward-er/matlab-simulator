function [settings,contSettings,sensorData] = run_MTR_SIM (contSettings,sensorData,settings,sensorTot,T1)



% mass estimation
A = contSettings.Engine_model_A1;
B = contSettings.Engine_model_B1;
C = contSettings.Engine_model_C1;

% prediction

if T1 < settings.timeEngineCut
    u = 1;
else
    u = 0;
end

% define time array for mea algorithm
t_mea = sensorTot.mea.time(end):1/settings.frequencies.MEAFrequency:T1;
% define time array for sensors
t_chambPress = sensorTot.comb_chamber.time;
t_nas = sensorTot.nas.time; % we need also nas to estimate cd etc

% initialise state update
x(1,:) = sensorData.mea.x(end,:);
P(:,:,1) = sensorData.mea.P(:,:,end);
predicted_apogee(1) = sensorData.mea.predicted_apogee(end);
estimated_mass(1) = sensorData.mea.estimated_mass(end);
estimated_pressure(1) = sensorData.mea.estimated_pressure(end);

for ii = 2:length(t_mea)

    % prediction
    x(ii,:) = (A*x(ii-1,:)' + B*u)'; % x is a row but to apply matrix product we need it column, therefore the transpositions
    P(:,:,ii) = A*P(:,:,ii-1)*A' + contSettings.mea.R;

    % correction
    index_chambPress = find(t_mea(ii) >= t_chambPress,1,"last");
    S = C*P(:,:,ii)*C' + contSettings.mea.Q;
    if ~det(S)<1e-3
        K = P(:,:,ii)*C'*inv(S);
        P(:,:,ii) = (eye(3)-K*C)*P(:,:,ii);
    end
    estimated_pressure(ii) = C * x(ii,:)';
    x(ii,:) = x(ii,:)' + K* ((sensorTot.comb_chamber.measures(index_chambPress)-1950)/1000 - estimated_pressure(ii));

    % update mass estimation
    estimated_mass(ii) = x(ii,3);
    
    % retrieve NAS data
    index_NAS = find(t_mea(ii) >= t_nas,1,"last");
    z_nas = sensorTot.nas.states(index_NAS,3);
    vnorm_nas = norm(sensorTot.nas.states(index_NAS,4:6));
    vz_nas = sensorTot.nas.states(index_NAS,6);

    % compute CD
    CD = settings.CD_correction_ref*getDrag(vnorm_nas, -z_nas, 0, contSettings.coeff_Cd); % coeffs potrebbe essere settings.coeffs
    [~,~,~,rho] = atmosisa(-z_nas);

    predicted_apogee(ii) = -z_nas-settings.z0 + 1/(2*( 0.5*rho * CD * settings.S / estimated_mass(ii)))...
        * log(1 + (vz_nas^2 * (0.5 * rho * CD * settings.S) / estimated_mass(ii)) / 9.81 );
end
if predicted_apogee(end) >= settings.z_final_MTR
    settings.counter_shutdown = settings.counter_shutdown + 1*floor(settings.frequencies.MEAFrequency/settings.frequencies.controlFrequency); % the last multiplication is to take into account the frequency difference
    if ~settings.shutdown
        if ~settings.expShutdown && settings.counter_shutdown > contSettings.N_prediction_threshold % threshold set in configControl
            settings.expShutdown = true;
            settings.t_shutdown = T1;
            settings.timeEngineCut =settings.t_shutdown + 0.3;
            settings.expTimeEngineCut = settings.t_shutdown;
        end
        settings.IengineCut = interpLinear(settings.motor.expTime, settings.I, T1);
        settings.expMengineCut = settings.parout.m(end) - settings.ms;
        if T1 > settings.timeEngineCut
            settings.shutdown = true;
            settings = settingsEngineCut(settings);
            settings.quatCut = [sensorTot.nas.states(end, 10) sensorTot.nas.states(end, 7:9)];
            [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
        end
        contSettings.valve_pos = 0;
    else
        settings.t_shutdown = nan;
    end
else
    settings.counter_shutdown = 0;
end

sensorData.mea.time = t_mea;
sensorData.mea.x = x;
sensorData.mea.P = P;
sensorData.mea.predicted_apogee = predicted_apogee;
sensorData.mea.estimated_mass = estimated_mass;
sensorData.mea.estimated_pressure = estimated_pressure;


