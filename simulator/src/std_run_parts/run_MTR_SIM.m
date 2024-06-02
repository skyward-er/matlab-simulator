function [sensorData,sensorTot,settings,contSettings] = run_MTR_SIM (sensorData,sensorTot,settings,contSettings,T1, engineT0,dt_ode)

    % impose valve position
    if T1 < settings.timeEngineCut
        u = 1;
    else
        u = 0;
    end
    if ~settings.flagMEAInit
        sensorTot.mea.time = T1-dt_ode;
        settings.flagMEAInit =  true;
    end
    if contains(settings.mission, '_2023')
        [sensorData,sensorTot] = run_MEA(sensorData,sensorTot,settings,contSettings,u,T1);
    elseif contains(settings.mission, '_2024')
        [sensorData,sensorTot] = run_MEA_LY(sensorData,sensorTot,settings,contSettings,T1);
    end
    if sensorTot.mea.prediction(end) >= settings.mea.z_shutdown
        settings.mea.counter_shutdown = settings.mea.counter_shutdown + 1*floor(settings.frequencies.MEAFrequency/settings.frequencies.controlFrequency); % the last multiplication is to take into account the frequency difference
        if ~settings.expShutdown
            if  settings.mea.counter_shutdown > contSettings.N_prediction_threshold %&& T1 > settings.mea.t_lower_shadowmode% threshold set in configControl
                settings.expShutdown = true;
                settings.t_shutdown = T1;
                settings.timeEngineCut = settings.t_shutdown + settings.shutdownValveDelay;
                settings.expTimeEngineCut = settings.t_shutdown;
            end
            if T1-engineT0 >= settings.mea.t_higher_shadowmode
                settings.expShutdown = true;
                settings.t_shutdown = T1;
                settings.timeEngineCut = settings.t_shutdown + settings.shutdownValveDelay;
                settings.expTimeEngineCut = settings.t_shutdown;
            end
        end
        if T1-engineT0 < settings.tb
            settings.IengineCut = interpLinear(settings.motor.expTime, settings.I, T1-engineT0);
        else
            settings.IengineCut = interpLinear(settings.motor.expTime, settings.I, settings.tb);
        end
        settings.expMengineCut = settings.parout.m(end) - settings.ms;
        if T1 > settings.timeEngineCut
            settings.shutdown = true;
            settings = settingsEngineCut(settings, engineT0);
            settings.quatCut = [sensorTot.nas.states(end, 10) sensorTot.nas.states(end, 7:9)];
            [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
        end

        contSettings.valve_pos = 0;
        % else
        %     settings.t_shutdown = nan;
        % end
    else
        if ~settings.expShutdown && T1-engineT0 >= settings.mea.t_higher_shadowmode
            settings.expShutdown = true;
            settings.t_shutdown = T1;
            settings.timeEngineCut = settings.t_shutdown + settings.shutdownValveDelay;
            settings.expTimeEngineCut = settings.t_shutdown;
        end
        if T1-engineT0 < settings.tb
            settings.IengineCut = interpLinear(settings.motor.expTime, settings.I, T1-engineT0);
        else
            settings.IengineCut = interpLinear(settings.motor.expTime, settings.I, settings.tb);
        end
        settings.expMengineCut = settings.parout.m(end) - settings.ms;
        if T1 > settings.timeEngineCut
            settings.shutdown = true;
            settings = settingsEngineCut(settings, engineT0);
            settings.quatCut = [sensorTot.nas.states(end, 10) sensorTot.nas.states(end, 7:9)];
            [~,settings.pitchCut,~] = quat2angle(settings.quatCut,'ZYX');
        end
        settings.counter_shutdown = 0;
    end

end


