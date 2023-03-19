function [t_shutdown,settings,contSettings,predicted_apogee,estimated_mass,estimated_pressure] =...
    run_MTR_SIM (contSettings,sensorData,settings,iTimes,c,Tf,Yf,x_est_tot)

% mass estimation

u = contSettings.u;

if u == 1
    A = contSettings.Engine_model_A1;
    B = contSettings.Engine_model_B1;
    C = contSettings.Engine_model_C1;
else
    A = contSettings.Engine_model_A2;
    B = contSettings.Engine_model_B2;
    C = contSettings.Engine_model_C2;
end

% %  estimated_pressure(iTimes) = C * contSettings.xe;
% %     e =  (c.cp_tot(end)-1950) - estimated_pressure(iTimes)*1000;
% %     e = e/1000;
% %     K=(A*contSettings.P_mat*C')/(C*contSettings.P_mat*C'+contSettings.Q);
% %     contSettings.P_mat=(A*contSettings.P_mat*A'+contSettings.R)-K*(A*contSettings.P_mat*C')';
% %     contSettings.xe=A*contSettings.xe + K*e+B*u;
% %
% %       estimated_mass(iTimes) = contSettings.xe(3);
% %       m = estimated_mass(iTimes);

% prediction
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
CD(iTimes) = getDrag(norm([sensorData.kalman.vx,sensorData.kalman.vy,sensorData.kalman.vz]), sensorData.kalman.z, 0, contSettings.coeff_Cd); % coeffs potrebbe essere settings.coeffs
[~,~,~,rho] = atmosisa(sensorData.kalman.z);

predicted_apogee(iTimes) = sensorData.kalman.z-settings.z0 + 1/(2*( 0.5*rho * CD(iTimes) * settings.S / m))...
    * log(1 + (sensorData.kalman.vz^2 * (0.5 * rho * CD(iTimes) * settings.S) / m) / 9.81 );

if predicted_apogee(iTimes) >= settings.z_final_MTR
    contSettings.u = 0;
    if ~settings.shutdown
        t_shutdown = Tf(end);
        settings.timeEngineCut = t_shutdown;
        settings.IengineCut = Yf(end,14:16);
        settings.expMengineCut = m - settings.ms;
        settings.shutdown = 1;
        settings = settingsEngineCut(settings);
        settings.quatCut = [x_est_tot(end, 8:10) x_est_tot(end, 7)];
        [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
    else
        t_shutdown = inf;
    end
end

if ~settings.shutdown && Tf(end) >= settings.tb
    settings.shutdown = 1;
    t_shutdown = settings.tb;
    settings.timeEngineCut = t_shutdown;
    settings = settingsEngineCut(settings);
    settings.quatCut = [x_est_tot(end, 8:10) x_est_tot(end, 7)];
    [~,settings.pitchCut,~]  = quat2angle(settings.quatCut,'ZYX');
else
    t_shutdown = inf;
end