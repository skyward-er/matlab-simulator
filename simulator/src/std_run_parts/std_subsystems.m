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
    
    [sensorData.kalman.x_c, vels_NED, P_c, settings.kalman]   =  run_kalman(x_prev, vels_prev, P_prev, sp, settings.kalman, XYZ0*0.01,settings.flagAscent,settings.flagStopPitotCorrection);
    if abs(sensorData.kalman.x_c(3,1)) >settings.stopPitotAltitude+ settings.z0
        settings.flagStopPitotCorrection = true;
    end
    sensorData.kalman.time(iTimes) = Tf(end);
    x_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1,:)  = sensorData.kalman.x_c(:,:); % NAS position output
    vels_tot(c.n_est_old:c.n_est_old + size(vels_NED(:,1),1)-1,:)  = vels_NED(:,:); % NAS speed output
    t_est_tot(c.n_est_old:c.n_est_old + size(sensorData.kalman.x_c(:,1),1)-1)    = sensorData.accelerometer.time; % NAS time output
    c.n_est_old = c.n_est_old + size(sensorData.kalman.x_c,1);

    sensorData.kalman.z  =  x_est_tot(end, 3);
    sensorData.kalman.x  =  x_est_tot(end, 2);
    sensorData.kalman.y  =  x_est_tot(end, 1);
    sensorData.kalman.vx =  x_est_tot(end, 4);   % north
    sensorData.kalman.vy =  x_est_tot(end, 5);   % east
    sensorData.kalman.vz =  x_est_tot(end, 6);   % down
end

v_ned = quatrotate(quatconj(Yf(:, 10:13)), Yf(:, 4:6));


%% Engine Control algorithm
if contains(settings.mission,'_2023')
    if Tf(end) <= settings.tb+0.5 &&...
            (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))

        if isnan(c.cp_tot(end))
            c.cp_tot(end) = 0;
        end
        if ~settings.shutdown
            [t_shutdown,settings,contSettings,predicted_apogee(iTimes),tPrediction(iTimes),estimated_mass,estimated_pressure] =...
                run_MTR_SIM (contSettings,sensorData,settings,iTimes,c,Tf,x_est_tot);
            m = estimated_mass(end);
        end

        if ~settings.shutdown && Tf(end) >= settings.tb
                t_shutdown = settings.tb;
                settings.expShutdown = 1;
                settings.timeEngineCut = t_shutdown;
                settings.expTimeEngineCut = t_shutdown;
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
            
            if contSettings.traj_choice == 1 && settings.expShutdown
                    if ~(strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
                        m = settings.ms;
                    else
                        m = estimated_mass(end);
                    end
                contSettings = trajectoryChoice_mass(m,contSettings);
            end
        end
    
        if contSettings.flagFirstControlABK % set in
    
            t_airbrakes = t0;
            t_last_arb_control = t0;
            idx_airbrakes = n_old+1;
    
        end
        if t1-t_last_arb_control >= 1/settings.frequencies.arbFrequency - 1e-5 || t_last_arb_control == t_airbrakes
    
            t_last_arb_control = Tf(end);
            ap_ref_old = ap_ref_new;
            settings.quat = [x_est_tot(end, [10,7:9])];
            [~,settings.pitch,~] = quat2angle(settings.quat,'ZYX');
            [ap_ref_new,contSettings] = run_ARB_SIM(sensorData,settings,contSettings,ap_ref_old); % "simulated" airbrakes because otherwise are run by the HIL.
    
        end
       
    else
        ap_ref_new = 0;
    end
else
    if flagAeroBrakes && mach < settings.MachControl && settings.flagNAS && settings.control
    
        if contSettings.flagFirstControlABK
    
            t_airbrakes = t0;
            t_last_arb_control = t0;
            idx_airbrakes = n_old+1;
    
        end
        if t1-t_last_arb_control >= 1/settings.frequencies.arbFrequency - 1e-6 || t_last_arb_control == t_airbrakes
            t_last_arb_control = t1(end);
            ap_ref_old = ap_ref_new;
            settings.quat = [x_est_tot(end, [10,7:9])];
            [~,settings.pitch,~] = quat2angle(settings.quat,'ZYX');
            [ap_ref_new,contSettings] = run_ARB_SIM(sensorData,settings,contSettings,ap_ref_old); % "simulated" airbrakes because otherwise are run by the HIL.
    
        end
       
    else
        ap_ref_new = 0;
    end
end


%% PARAFOIL
if ~settings.flagAscent && settings.parafoil 
    if flagPara2
        if contSettings.flagFirstControlPRF % set in
        
                t_parafoil = t1;
                t_last_prf_control = t1;
                idx_parafoil = n_old+1;
                contSettings.flagFirstControlPRF = false;
                if contSettings.payload.guidance_alg == "t-approach"
                    pos_est = sensorData.kalman.x_c(end,1:3);
                    pos_est(3) = -pos_est(3)-settings.z0;
                    [contSettings.payload.EMC,contSettings.payload.M1,contSettings.payload.M2] = setEMCpoints(pos_est,settings.payload.target,contSettings.payload.mult_EMC,contSettings.payload.d);
                end
        end
        if contSettings.payload.flagWES
                if t1 < t_parafoil + contSettings.payload.guidance_start
                    contSettings.WES.state = 1;
                else
                    contSettings.WES.state = 2;
                end
                vel_est = sensorData.kalman.x_c(end,4:5);
                [contSettings.WES] = run_WES(vel_est,contSettings.WES);
                wind_est = [contSettings.WES.wind_est];
        else
                wind_est = [0,0];
        end
    

        if t1-t_last_prf_control >= 1/settings.frequencies.prfFrequency - 1e-6 || t_last_prf_control == t_parafoil
                deltaA_ref_old = deltaA_ref_new;
                t_last_prf_control = t1;
                pos_est = sensorData.kalman.x_c(end,1:3);
                pos_est(3) = -pos_est(3)-settings.z0;
                
                [deltaA_ref_new,contSettings] = run_parafoilGuidance(pos_est, sensorData.kalman.x_c(end,4:5), wind_est, settings.payload.target, contSettings);
        end
        
    end
end
   
