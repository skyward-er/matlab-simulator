function [t_shutdown,settings,contSettings,predicted_apogee,tPrediction,estimated_mass,estimated_pressure] =...
    run_MTR_SIM (contSettings,sensorData,settings,iTimes,c,Tf,x_est_tot)

% mass estimation

    A = contSettings.Engine_model_A1;
    B = contSettings.Engine_model_B1;
    C = contSettings.Engine_model_C1;

% prediction

if Tf(end) >= settings.timeEngineCut
    u = 0;
else
    u = 1;
end

contSettings.xe=A*contSettings.xe + B*u;
contSettings.P_mat = A*contSettings.P_mat*A' + contSettings.R;
% correction
S = C*contSettings.P_mat*C' + contSettings.Q;
if ~det(S)<1e-3
    K = contSettings.P_mat*C'*inv(S);
    contSettings.P_mat = (eye(3)-K*C)*contSettings.P_mat;
end
estimated_pressure(iTimes) = C * contSettings.xe;
contSettings.xe=contSettings.xe + K* ((c.cp_tot(end)-1950)/1000 - estimated_pressure(iTimes));

estimated_mass(iTimes) = contSettings.xe(3);
m = estimated_mass(iTimes);
% magic formula seguire traiettorie è meglio?
CD(iTimes) = settings.CD_correction_ref*getDrag(norm([sensorData.kalman.vx,sensorData.kalman.vy,sensorData.kalman.vz]), -sensorData.kalman.z, 0, contSettings.coeff_Cd); % coeffs potrebbe essere settings.coeffs
[~,~,~,rho] = atmosisa(-sensorData.kalman.z);

%% TEST WITH MASS ESTIMATION THAT DOESN'T WORK
% mass = mass_dry
% m = settings.ms;
% m = settings.ms + (settings.m0-settings.ms)/2; 
%%
predicted_apogee = -sensorData.kalman.z-settings.z0 + 1/(2*( 0.5*rho * CD(iTimes) * settings.S / m))...
    * log(1 + (sensorData.kalman.vz^2 * (0.5 * rho * CD(iTimes) * settings.S) / m) / 9.81 );
tPrediction = Tf(end);
t_shutdown = Inf;
if predicted_apogee >= settings.z_final_MTR
    if ~settings.shutdown
        settings.expShutdown = 1;
        settings.shutdown = true;
        if contSettings.MTR_fault 
            if contSettings.valve_pos
                settings.shutdown = 0;
                settings.expTimeEngineCut = Tf(end);
            end
        else 
            t_shutdown = Tf(end);
            settings.timeEngineCut = t_shutdown + 0.3;
            settings.expTimeEngineCut = t_shutdown;
            settings.IengineCut = interpLinear(settings.motor.expTime, settings.I, Tf(end));
            settings.expMengineCut = m - settings.ms;
            if Tf(end) > settings.timeEngineCut
                settings.shutdown = 1;
                settings = settingsEngineCut(settings);
                settings.quatCut = [x_est_tot(end, 10) x_est_tot(end, 7:9)];
                [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
            end
        end
        contSettings.valve_pos = 0;
    else
        t_shutdown = inf;
    end
end
