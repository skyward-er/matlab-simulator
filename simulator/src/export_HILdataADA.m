%{
export configuration files for ADA algorithm (for our friends of software development)
%}

        folder = "HIL_CPP_files_ADA";
        if ~exist(ConDataPath+"/"+folder,"dir")
            mkdir(ConDataPath+"/"+folder)
        end

        % first file: configuration parameters
        configADA_export_table = table;
        configValues = [settings.frequencies.ADAFrequency,settings.ada.shadowmode, ...
            settings.ada.v_thr+2.5,settings.ada.count_thr,settings.ada.Q(1,1), ...
            settings.ada.Q(2,2),settings.ada.Q(3,3),settings.ada.R,101250,288.15,...
            settings.ada.P0(1,1),rocket.parachutes{1,1}.finalAltitude];
        configADAvarNames = {'ADA_FREQUENCY','ADA_SHADOWMODE',...
            'ADA_VELOCITY_THRESHOLD','ADA_COUNTER_THRESHOLD','ADA_Q_11',...
            'ADA_Q_22','ADA_Q_33','ADA_R','mslPressure','mslTemperature','P0',...
            'MAIN_DEPLOYMENT_ALTITUDE'};
        for i = 1:size(configValues,2)
            configADA_export_table(1,i) = table(configValues(1,i));
        end
        configADA_export_table.Properties.VariableNames = configADAvarNames;
        writetable(configADA_export_table,ConDataPath+"/"+folder+"/ADA_configADA_"+mission.name+".csv")
