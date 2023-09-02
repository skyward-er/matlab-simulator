function [t_shutdown,settings,contSettings,sensorData] = run_MTR_SIM (contSettings,sensorData,settings,sensorTot,Tf)

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

sensorData.mea.x = A*sensorData.mea.x + B*u;
sensorData.mea.P = A*sensorData.mea.P*A' + contSettings.R;
% correction
S = C*sensorData.mea.P*C' + contSettings.Q;
if ~det(S)<1e-3
    K = sensorData.mea.P*C'*inv(S);
    sensorData.mea.P = (eye(3)-K*C)*sensorData.mea.P;
end
estimated_pressure = C * sensorData.mea.x;
sensorData.mea.x = sensorData.mea.x + K* ((sensorTot.comb_chamber.measures(end)-1950)/1000 - estimated_pressure);

estimated_mass = sensorData.mea.x(3);
m = estimated_mass;
% magic formula seguire traiettorie è meglio?
CD = settings.CD_correction_ref*getDrag(norm([sensorData.kalman.vx,sensorData.kalman.vy,-sensorData.kalman.vz]), -sensorData.kalman.z, 0, contSettings.coeff_Cd); % coeffs potrebbe essere settings.coeffs
[~,~,~,rho] = atmosisa(-sensorData.kalman.z);

%% TEST WITH MASS ESTIMATION THAT DOESN'T WORK

predicted_apogee = -sensorData.kalman.z-settings.z0 + 1/(2*( 0.5*rho * CD * settings.S / m))...
    * log(1 + (sensorData.kalman.vz^2 * (0.5 * rho * CD * settings.S) / m) / 9.81 );
t_shutdown = Inf;
if predicted_apogee >= settings.z_final_MTR
    if ~settings.shutdown
        settings.expShutdown = 1;
        if contSettings.MTR_fault 
            if contSettings.u
                settings.shutdown = 0;
                settings.expTimeEngineCut = Tf(end);
            end
        else 
            t_shutdown = Tf(end);
            settings.timeEngineCut = t_shutdown + 0.3;
            settings.expTimeEngineCut = t_shutdown;
            % settings.IengineCut = Yf(end,14:16);
            settings.expMengineCut = m - settings.ms;
            if Tf(end) > settings.timeEngineCut
                settings.shutdown = 1;
                settings = settingsEngineCut(settings);
                settings.quatCut = [sensorTot.nas.x(end, 10) sensorTot.nas.x(end, 7:9)];
                [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
            end
        end
        contSettings.u = 0;
    else
        t_shutdown = inf;
    end
end
sensorData.mea.predicted_apogee = predicted_apogee(end);
sensorData.mea.estimated_mass = estimated_mass(end);
sensorData.mea.estimated_pressure = estimated_pressure(end);


