%{

This function runs all subsystems in a simulated environment

%}




%% Navigation system (NAS)
if settings.flagNAS && settings.dataNoise
    [sensorData, sensorTot, settings.nas]   =  run_NAS_FAST(Tf,  Yf, sensorData, sensorTot, settings);
    


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



   
