%{

This function runs all subsystems in a simulated environment

%}




%% Sensor fault detection

% simulation of the faults


% sensor fault detection algorithm
Nsensors = [1,2,3];
goodSensors = Nsensors(not(settings.faulty_sensors));
if settings.flagAscent
    SVM_model= settings.SVM_1;
else
    SVM_model = settings.SVM_2;
end
for i = goodSensors
    chunk{i}(1,1:end-length(sp.pn_sens{i})) = chunk{i}(1+length(sp.pn_sens{i}):end);
    chunk{i}(1,end-length(sp.pn_sens{i})+1:end) = sp.pn_sens{i};
    if length(chunk{i})>SVM_model.N_sample
        warning('chunk length is greater than %d samples',SVM_model.N_sample)
    end
end
[sensorData,sp,chunk,settings.faulty_sensors] = run_SensorFaultDetection_SVM(SVM_model,sensorData,sp,chunk,settings.faulty_sensors,settings.flagAscent,t0);

%% ADA
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

if settings.flagADA && settings.dataNoise && length(sensorData.barometer.time) > 1 ...
        && sensorData.barometer.time(1) >= settings.baro_old
    [xp_ada, xv_ada, P_ada, settings.ada]   =  run_ADA(ada_prev, Pada_prev, sp.pn, sensorData.barometer.time, settings.ada);

    xp_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1) -1,:)  = xp_ada(1:end,:);
    xv_ada_tot(c.n_ada_old:c.n_ada_old + size(xv_ada(:,1),1)-1,:)  = xv_ada(1:end,:);
    t_ada_tot(c.n_ada_old:c.n_ada_old + size(xp_ada(:,1),1)-1)     = sensorData.barometer.time;
    c.n_ada_old = c.n_ada_old + size(xp_ada,1);

    settings.baro_old = sensorData.barometer.time(end);
end

%% Navigation system

if settings.flagNAS && settings.dataNoise

    %     sp.pn = sp.pn(end);
    %     sp.t_baro = sp.t_baro(end);

    [sensorData.kalman.x_c, vels, P_c, settings.kalman]   =  run_kalman(x_prev, vels_prev, P_prev, sp, settings.kalman, XYZ0*0.01,settings.flagAscent);
    sensorData.kalman.time(iTimes) = Tf(end);
    x_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1,:)  = sensorData.kalman.x_c(:,:); % NAS position output
    vels_tot(c.n_est_old:c.n_est_old + size(vels(:,1),1)-1,:)  = vels(:,:); % NAS speed output
    t_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1)    = sensorData.accelerometer.time; % NAS time output
    c.n_est_old = c.n_est_old + size(sensorData.kalman.x_c,1);

    sensorData.kalman.z  = -x_est_tot(end, 3);
    sensorData.kalman.x  =  x_est_tot(end, 2);
    sensorData.kalman.y  =  x_est_tot(end, 1);
    sensorData.kalman.vx =  x_est_tot(end, 4);   % north
    sensorData.kalman.vy =  x_est_tot(end, 5);   % east
    sensorData.kalman.vz = -x_est_tot(end, 6);   % down
    est = sensorData.kalman.vz;
end

% vertical velocity and position
% % % % % % if settings.flagAscent || (not(settings.flagAscent) && settings.ballisticFligth)
% % % % % %     Q    =   Yf(end, 10:13);
% % % % % %     vels =   quatrotate(quatconj(Q), Yf(:, 4:6));
% % % % % %     sensorData.kalman.vz = - vels(end,3);   % down
% % % % % %     sensorData.kalman.vx =   vels(end,2);   % north
% % % % % %     sensorData.kalman.vy =   vels(end,1);   % east
% % % % % %     real = sensorData.kalman.vz;
% % % % % %     est_meno_real = est-real;
% % % % % %     z = -Yf(end, 3);
% % % % % % else
% % % % % %     sensorData.kalman.vz = - Yf(end, 6); % actually not coming from NAS in this case
% % % % % %     sensorData.kalman.vx = Yf(end, 5);
% % % % % %     sensorData.kalman.vy = Yf(end, 4);  
% % % % % % end
v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));

% % % % % % 
% % % % % % sensorData.kalman.z  = -x_est_tot(end, 3);
% % % % % % sensorData.kalman.x  =  Yf(end, 2);
% % % % % % sensorData.kalman.y  =  Yf(end, 1);
% sensorData.kalman.time = Tf(end); %% CAPIRE A COSA SERVE

%% Engine Control algorithm
    
if contains(settings.mission,'_2023')
    if Tf(end) <= settings.tb+0.5 &&...
       (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
    
    if isnan(c.cp_tot(end))
        c.cp_tot(end) = 0;
    end
        if ~settings.shutdown 
           [t_shutdown,settings,contSettings,predicted_apogee,estimated_mass,estimated_pressure] =...
               run_MTR_SIM (contSettings,sensorData,settings,iTimes,c,Tf,Yf,x_est_tot);
           m = estimated_mass(end);
        end
    
        if ~settings.shutdown && Tf(end) >= settings.tb
              t_shutdown = settings.tb;
              settings.expShutdown = 1;
                settings.timeEngineCut = t_shutdown;
                settings.expTimeEngineCut = t_shutdown;
                settings.IengineCut = Yf(end,14:16);
                settings.expMengineCut = m - settings.ms;
                settings.shutdown = 1;
                settings = settingsEngineCut(settings);
                settings.quatCut = [x_est_tot(end, 8:10) x_est_tot(end, 7)];
                [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
        end
    % plot brutti per ora perchè
    % predicted_apogee,estimated_mass,estimated_pressure dovrebbero essere dati
    % in input a run_MTR_SIM
    elseif ~(strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete')) && ...
            Tf(end) > settings.tb
        settings.shutdown = 1;
        settings.expShutdown = 1;
        settings.timeEngineCut = settings.tb;
        settings.expTimeEngineCut = settings.tb;
    end
else
    if t0 > settings.motor.expTime(end)
        settings.expShutdown = 1;
    end
end
%% ARB Control algorithm
if contains(settings.mission,'_2023')
    if flagAeroBrakes && mach < settings.MachControl && settings.flagNAS && settings.control...
            && ~(strcmp(contSettings.algorithm,'NoControl') || strcmp(contSettings.algorithm,'engine') ) ...
            && Tf(end) > settings.expTimeEngineCut + 0.5
    
        if str2double(settings.mission(end)) > 2 % only for mission after october 2022
            %% TEST WITH MASS ESTIMATION THAT DOESN'T WORK
            % mass = mass_dry
    %         m = settings.ms;
    %         m = settings.ms + (settings.m0-settings.ms)/2; 
            %%
            trajectoryChoice_mass;
        end
    
        if contSettings.flagFirstControl
    
            t_airbrakes = t0;
            t_last_arb_control = t0;
            idx_airbrakes = n_old+1;
    
        end
        if Tf(end)-t_last_arb_control >= 1/settings.frequencies.arbFrequency - 1e-5 ||...
                t_last_arb_control == t_airbrakes
    
            t_last_arb_control = Tf(end);
            ap_ref_old = ap_ref_new;
            settings.quat = [x_est_tot(end, 8:10) x_est_tot(end, 7)];
            [~,settings.pitch,~] = quat2angle(settings.quat,'ZYX');
            [ap_ref_new,contSettings] = run_ARB_SIM(sensorData,settings,contSettings,ap_ref_old); % "simulated" airbrakes because otherwise are run by the HIL.
    
        end
       
    else
        ap_ref_new = 0;
    end
else
    if flagAeroBrakes && mach < settings.MachControl && settings.flagNAS && settings.control
    
        if contSettings.flagFirstControl
    
            t_airbrakes = t0;
            t_last_arb_control = t0;
            idx_airbrakes = n_old+1;
    
        end
        if Tf(end)-t_last_arb_control >= 1/settings.frequencies.arbFrequency - 1e-5 ||...
                t_last_arb_control == t_airbrakes
    
            t_last_arb_control = Tf(end);
            ap_ref_old = ap_ref_new;
            settings.quat = [x_est_tot(end, 8:10) x_est_tot(end, 7)];
            [~,settings.pitch,~] = quat2angle(settings.quat,'ZYX');
            [ap_ref_new,contSettings] = run_ARB_SIM(sensorData,settings,contSettings,ap_ref_old); % "simulated" airbrakes because otherwise are run by the HIL.
    
        end
       
    else
        ap_ref_new = 0;
    end
end