%{

This function runs all subsystem in a simulated environment

%}

if iTimes>3
    if settings.flagADA
        ada_prev  =   xp_ada_tot(end,:);
        Pada_prev =   P_ada(:,:,end);
    end

    if  settings.flagNAS
        x_prev    =   x_est_tot(end,:);
        vels_prev =   vels_tot(end,:);
        P_prev    =   P_c(:,:,end);
    end
end

%% ADA
if settings.flagADA && settings.dataNoise && length(sensorData.barometer.time) > 1

    [xp_ada, xv_ada, P_ada, settings.ada]   =  run_ADA(ada_prev, Pada_prev, sp.pn, sensorData.barometer.time, settings.ada);

    xp_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1) -1,:)  = xp_ada(1:end,:);
    xv_ada_tot(c.n_ada_old:c.n_ada_old + size(xv_ada(:,1),1)-1,:)  = xv_ada(1:end,:);
    t_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1)-1)     = sensorData.barometer.time;
    c.n_ada_old = c.n_ada_old + size(xp_ada,1);
end

%% Navigation system

if settings.flagNAS && settings.dataNoise 

%     sp.pn = sp.pn(end);
%     sp.t_baro = sp.t_baro(end);

    [sensorData.kalman.x_c, vels, P_c, settings.kalman]   =  run_kalman(x_prev, vels_prev, P_prev, sp, settings.kalman, XYZ0*0.01);
    sensorData.kalman.time(iTimes) = Tf(end);
    x_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1,:)  = sensorData.kalman.x_c(:,:); % NAS position output
    vels_tot(c.n_est_old:c.n_est_old + size(vels(:,1),1)-1,:)  = vels(:,:); % NAS speed output
    t_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1)    = sensorData.accelerometer.time; % NAS time output
    c.n_est_old = c.n_est_old + size(sensorData.kalman.x_c,1);

end

% vertical velocity and position
if settings.flagAscent || (not(settings.flagAscent) && settings.ballisticFligth)
    Q    =   Yf(end, 10:13);
    vels =   quatrotate(quatconj(Q), Yf(:, 4:6));
    sensorData.kalman.vz = - vels(end,3);   % down
    sensorData.kalman.vx =   vels(end,2);   % north
    sensorData.kalman.vy =   vels(end,1);   % east

else
    sensorData.kalman.vz = - Yf(end, 6); % actually not coming from NAS in this case
    sensorData.kalman.vx = Yf(end, 5);
    sensorData.kalman.vy = Yf(end, 4);
end
v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));

sensorData.kalman.z  = -x_est_tot(end, 3);
sensorData.kalman.x  =  Yf(end, 2);
sensorData.kalman.y  =  Yf(end, 1);
% sensorData.kalman.time = Tf(end); %% CAPIRE A COSA SERVE

%% Engine Control algorithm
    

if Tf(end) < settings.tb &&...
   (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))

    % mass estimation
A = contSettings.Engine_model_A;
B = contSettings.Engine_model_B;
C = contSettings.Engine_model_C;
% % % %     xe = contSettings.Engine_model_A * xe + contSettings.Engine_model_B * u; % propagation
% % % %     estimated_pressure(iTimes) = contSettings.Engine_model_C * xe; 
% % % %     e =  (c.cp_tot(end)-1950) - estimated_pressure(iTimes)*1000;
% % % %     e = e/1000; % from mbar to bar
% % % %     xe = xe + contSettings.Engine_model_Kgain * e; % correction
% % % % 
% % % %     estimated_mass(iTimes) = xe(3);
% % % %     m = estimated_mass(iTimes);


%%%%%%% online kalman
    estimated_pressure(iTimes) = contSettings.Engine_model_C * xe; 
    e =  (c.cp_tot(end)-1950) - estimated_pressure(iTimes)*1000;
    e = e/1000;
    K=(A*P_mat*C')/(C*P_mat*C'+V2);
    P_mat=(A*P_mat*A'+V1)-K*(A*P_mat*C')';
    xe=A*xe + K*e+B*u;
    
      estimated_mass(iTimes) = xe(3);
      m = estimated_mass(iTimes);

    % magic formula seguire traiettorie è meglio?
    CD(iTimes) = getDrag(norm(vels(end,:)), sensorData.kalman.z, 0, contSettings.coeff_Cd); % coeffs potrebbe essere settings.coeffs
    [~,~,~,rho] = atmosisa(sensorData.kalman.z);

    predicted_apogee(iTimes) = sensorData.kalman.z-settings.z0 + 1/(2*( 0.5*rho * CD(iTimes) * settings.S / m))...
        * log(1 + (sensorData.kalman.vz^2 * (0.5 * rho * CD(iTimes) * settings.S) / m) / 9.81 );
    
    if predicted_apogee(iTimes) >= settings.z_final + 100
            u = 0;
            if ~settings.shutdown 
            t_shutdown = Tf(end);
            settings.timeEngineCut = t_shutdown;
            settings.IengineCut = Yf(end,14:16);
            settings.expMengineCut = m - settings.ms;
            settings.shutdown = 1;
            % modificare la ascent
            end
    end

end

if ~settings.shutdown && Tf(end) >= settings.tb
    settings.shutdown = 1;
    t_shutdown = settings.tb;
end
%% ARB Control algorithm

if str2double(settings.mission(end)) > 2 % only for mission after october 2022

    trajectoryChoice_mass;

end

if flagAeroBrakes && mach < settings.MachControl && settings.flagNAS && settings.control...
        && ~(strcmp(contSettings.algorithm,'NoControl') || strcmp(contSettings.algorithm,'engine') ) 
    if contSettings.flagFirstControl

        t_airbrakes = t0;
        t_last_arb_control = t0;
        idx_airbrakes = n_old+1;

    end
    if Tf(end)-t_last_arb_control >= 1/settings.frequencies.arbFrequency - 1e-5 ||...
            t_last_arb_control == t_airbrakes

        t_last_arb_control = Tf(end);
        ap_ref_old = ap_ref_new;
        [ap_ref_new,contSettings] = run_ARB_SIM(sensorData,settings,contSettings,ap_ref_old); % "simulated" airbrakes because otherwise are run by the HIL.

    end
   
else
    ap_ref_new = 0;
end

 