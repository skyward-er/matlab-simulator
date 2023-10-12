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
    sensorData.chunk{i}(1,1:end-length(sensorData.barometer_sens{i}.measures)) = sensorData.chunk{i}(1+length(sensorData.barometer_sens{i}.measures):end);
    sensorData.chunk{i}(1,end-length(sensorData.barometer_sens{i}.measures)+1:end) = sensorData.barometer_sens{i}.measures;
    if length(sensorData.chunk{i})>SVM_model.N_sample
        warning('chunk length is greater than %d samples',SVM_model.N_sample)
    end
end
[sensorData,settings.faulty_sensors] = run_SensorFaultDetection_SVM(SVM_model,sensorData,settings.faulty_sensors,settings.flagAscent,t1);

sensorTot.barometer.pressure_measures(sensorTot.barometer.n_old:sensorTot.barometer.n_old + size(sensorData.barometer.measures' ,1) - 1,:)    = sensorData.barometer.measures';
sensorTot.barometer.altitude(sensorTot.barometer.n_old:sensorTot.barometer.n_old + size(sensorData.barometer.measures',1) - 1,:)    = sensorData.barometer.z';
sensorTot.barometer.time(sensorTot.barometer.n_old:sensorTot.barometer.n_old + size(sensorData.barometer.measures',1) - 1)    =  sensorData.barometer.time;
sensorTot.barometer.n_old = sensorTot.barometer.n_old + size(sensorData.barometer.measures,1);

%% ADA
if settings.flagADA && settings.dataNoise

    [sensorData, sensorTot, settings.ada]   =  run_ADA(sensorData, sensorTot, settings,t1);

    
end

%% Navigation system (NAS)
if settings.flagNAS && settings.dataNoise
    [sensorData, sensorTot, settings.nas]   =  run_NAS(t1,  XYZ0*0.01, sensorData, sensorTot, settings);
    


    %%%%%%%%%%%%%%%%%% DA RIVEDERE L'UTILIZZO DI QUESTE VARIABILI ASSOLUTAMENTE %%%%%%%%%%%%%%%%%%%%%%%%
    sensorData.kalman.z  =  sensorTot.nas.states(end, 3);
    sensorData.kalman.x  =  sensorTot.nas.states(end, 2);
    sensorData.kalman.y  =  sensorTot.nas.states(end, 1);
    sensorData.kalman.vx =  sensorTot.nas.states(end, 4);   % north
    sensorData.kalman.vy =  sensorTot.nas.states(end, 5);   % east
    sensorData.kalman.vz =  sensorTot.nas.states(end, 6);   % down
end

%% Engine Control algorithm
if contains(settings.mission,'_2023')
    if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))

        if isnan(sensorTot.comb_chamber.measures(end))
            sensorTot.comb_chamber.measures(end) = 0;
        end
        if ~settings.shutdown
            [sensorData,sensorTot,settings,contSettings] =run_MTR_SIM (sensorData,sensorTot,settings,contSettings,t1);
            sensorTot.mea.t_shutdown = settings.t_shutdown;

            if  Tf(end) >= settings.tb
                settings.expShutdown = true;
                settings.shutdown = true; 
                settings.t_shutdown = settings.tb;
                settings.timeEngineCut = settings.t_shutdown;
                settings.expTimeEngineCut = settings.t_shutdown;
                % settings.expMengineCut = settings.parout.m(end) - settings.ms;
                % settings = settingsEngineCut(settings);
                settings.quatCut = [sensorTot.nas.states(end,10) sensorTot.nas.states(end, 7:9)]; % why do we take the nas ones and not the simulation ones?
                [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
                sensorTot.mea.t_shutdown = settings.t_shutdown; % to pass the value out of the std_run to the structOut
            end
        end


    elseif ~(strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete')) && Tf(end) > settings.tb
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
            && Tf(end) >= contSettings.ABK_shadowmode 
        %&&Tf(end) > settings.expTimeEngineCut + contSettings.ABK_shutdown_delay
    
        if str2double(settings.mission(end)) > 2 % only for mission after october 2022
            
            if contSettings.traj_choice == 1 && settings.expShutdown
                    if ~(strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))
                        m = settings.ms;
                    else
                        m = sensorData.mea.estimated_mass(end);
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
            settings.quat = [sensorTot.nas.states(end, [10,7:9])];
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
            settings.quat = [sensorTot.nas.states(end, [10,7:9])];
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
                    pos_est = sensorData.nas.states(end,1:3);
                    pos_est(3) = -pos_est(3)-settings.z0;
                    [contSettings.payload.EMC,contSettings.payload.M1,contSettings.payload.M2] = setEMCpoints(pos_est,settings.payload.target,contSettings.payload);
                end
        end
        if contSettings.payload.flagWES
                if t1 < t_parafoil + contSettings.payload.guidance_start
                    contSettings.WES.state = 1;
                else
                    contSettings.WES.state = 2;
                end
                vel_est = sensorData.nas.states(end,4:5);
                [contSettings.WES] = run_WES(vel_est,contSettings.WES);
                wind_est = [contSettings.WES.wind_est];
                sensorTot.wes.measure(iTimes,:) = wind_est; 
                sensorTot.wes.time(iTimes) = t1; 
        else
                wind_est = [0,0];
        end
    

        if t1-t_last_prf_control >= 1/settings.frequencies.prfFrequency - 1e-6 || t_last_prf_control == t_parafoil
                deltaA_ref_old = deltaA_ref_new;
                t_last_prf_control = t1;
                pos_est = sensorData.nas.states(end,1:3);
                pos_est(3) = -pos_est(3)-settings.z0;
                
                [deltaA_ref_new,contSettings] = run_parafoilGuidance(pos_est, sensorData.nas.states(end,4:5), wind_est, settings.payload.target, contSettings);
        end
        
    end
end
   
