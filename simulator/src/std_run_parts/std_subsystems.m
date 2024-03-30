%{

This function runs all subsystems in a simulated environment

%}



%% ADA
if settings.flagADA && settings.dataNoise
    [sensorData, sensorTot, settings.ada, flagApogee, flagOpenPara]   =  run_ADA(sensorData, sensorTot, settings,t1);
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
if (contains(settings.mission,'_2023') || contains(settings.mission,'_2024')) && currentState ~= availableStates.on_ground
    if (strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete'))

        if isnan(sensorTot.comb_chamber.measures(end))
            sensorTot.comb_chamber.measures(end) = 0;
        end
        if ~settings.shutdown
            [sensorData,sensorTot,settings,contSettings] =run_MTR_SIM (sensorData,sensorTot,settings,contSettings,t1, engineT0);
            sensorTot.mea.t_shutdown = settings.t_shutdown;

            if  Tf(end)-engineT0 >= settings.tb
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


    elseif ~(strcmp(contSettings.algorithm,'engine') || strcmp(contSettings.algorithm,'complete')) && Tf(end)-engineT0 > settings.tb
        settings.shutdown = 1;
        settings.expShutdown = 1;
        settings.timeEngineCut = settings.tb;
        settings.expTimeEngineCut = settings.tb;
    end
else
    if t0-engineT0 > settings.motor.expTime(end)
        settings.expShutdown = 1;
        settings.expTimeEngineCut = settings.motor.expTime(end);
    end
end


%% ARB Control algorithm

if flagAeroBrakes && settings.flagNAS && settings.control && ...
    ~( strcmp(contSettings.algorithm,'NoControl') || strcmp(contSettings.algorithm,'engine')) 

    if Tf(end) >= settings.expTimeEngineCut + contSettings.ABK_shutdown_delay && mach <= settings.MachControl
        if (contains(settings.mission,'_2023') || contains(settings.mission,'_2024'))
            if  contSettings.traj_choice == 1
                if ~strcmp(contSettings.algorithm,'complete')
                    m = settings.ms;
                else
                    m = sensorData.mea.estimated_mass(end);
                end
                contSettings = trajectoryChoice_mass(m,contSettings);
            end
        end
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
    end
else
    ap_ref_new = 0;
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
   
