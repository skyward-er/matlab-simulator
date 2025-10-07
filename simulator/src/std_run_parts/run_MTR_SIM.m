function [sensorData,sensorTot,settings,contSettings,rocket] = run_MTR_SIM...
        (sensorData,sensorTot,settings,contSettings,T1, engineT0,dt,rocket,environment, mission)

    % impose valve position
    if T1-engineT0 <= rocket.motor.cutoffTime
        u = 1;
    else
        u = 0;
    end
    if ~settings.flagMEAInit
        sensorTot.mea.time = T1-dt;
        settings.flagMEAInit =  true;
    end
    [sensorData,sensorTot] = run_MEA(sensorData,sensorTot,settings,contSettings,u,T1,engineT0,environment,rocket, mission);

    if sensorTot.mea.prediction(end) >= settings.mea.z_shutdown
        settings.mea.counter_shutdown = settings.mea.counter_shutdown + 1*floor(settings.frequencies.MEAFrequency/settings.frequencies.controlFrequency); % the last multiplication is to take into account the frequency difference
        if ~settings.expShutdown && (T1-engineT0) > settings.mea.t_lower_shadowmode
            if  settings.mea.counter_shutdown > contSettings.N_prediction_threshold % threshold set in configControl
                settings.expShutdown = true;
                settings.t_shutdown = T1;
                rocket.motor.cutoffTime  = settings.t_shutdown + settings.shutdownValveDelay - engineT0;
                settings.expTimeEngineCut = settings.t_shutdown;
            end
            if T1-engineT0 >= settings.mea.t_higher_shadowmode
                settings.expShutdown = true;
                settings.t_shutdown = T1;
                rocket.motor.cutoffTime = settings.t_shutdown + settings.shutdownValveDelay - engineT0;
                settings.expTimeEngineCut = settings.t_shutdown;
            end
        end

        % if settings.expShutdown
        %     if T1-engineT0 < rocket.motor.time(end)
        %         % if rocket.motor.cutoffTime > rocket.motor.time(end)
        %         %     rocket.motor.cutoffTime = rocket.motor.cutoffTime - engineT0;
        %         % end
        %         cutoff = rocket.motor.cutoffTime;
        %         rocket.motor.cutoffTime = T1 - engineT0;
        %         rocket.updateCutoff;
        %         rocket.motor.cutoffTime = cutoff;
        %         % settings.IengineCut = interpLinear(settings.motor.expTime, settings.I, T1-engineT0);
        %     else
        %         rocket.motor.cutoffTime = rocket.motor.time(end);
        %         rocket.updateCutoff;
        %         % settings.IengineCut = interpLinear(settings.motor.expTime, settings.I, settings.tb);
        %     end
        % end

        if T1 - engineT0 > rocket.motor.cutoffTime
            settings.shutdown = true;
        end
    else
        if ~settings.expShutdown && T1-engineT0 >= settings.mea.t_higher_shadowmode
            settings.expShutdown = true;
            settings.t_shutdown = T1;
            rocket.motor.cutoffTime  = settings.t_shutdown + settings.shutdownValveDelay - engineT0;
            settings.expTimeEngineCut = settings.t_shutdown;
        end
        if T1 - engineT0 > rocket.motor.cutoffTime
            settings.shutdown = true;
            settings.expShutdown = true;
            settings.t_shutdown = T1;
            rocket.motor.cutoffTime  = settings.t_shutdown + settings.shutdownValveDelay - engineT0;
            settings.expTimeEngineCut = settings.t_shutdown;
        end
        settings.mea.counter_shutdown = 0;
    end

end