%{
export configuration files for ADA algorithm (for our friends of software development)
%}
switch mission.name

    case {"Gemini_Portugal_October_2023", "Gemini_Roccaraso_September_2023"}
        folder = "HIL_CPP_files_ADA";
        if ~exist(ConDataPath+"/"+folder,"dir")
            mkdir(ConDataPath+"/"+folder)
        end

        % first file: configuration parameters
        configADA_export_table = table;
        configValues = [settings.frequencies.ADAFrequency,settings.ada.shadowmode, settings.ada.v_thr,settings.ada.count_thr,settings.ada.Q(1,1),settings.ada.Q(2,2),settings.ada.Q(3,3),settings.ada.R];
        configADAvarNames = {'ADA_FREQUENCY','ADA_SHADOWMODE','ADA_VELOCITY_THRESHOLD','ADA_COUNTER_THRESHOLD','ADA_Q_11','ADA_Q_22','ADA_Q_33','ADA_R'}; % x0, v0, a0 e pref?
        for i = 1:size(configValues,2)
            configADA_export_table(1,i) = table(configValues(1,i));
        end
        configADA_export_table.Properties.VariableNames = configADAvarNames;
        writetable(configADA_export_table,ConDataPath+"/"+folder+"/ADA_configADA_"+mission.name+".csv")


end