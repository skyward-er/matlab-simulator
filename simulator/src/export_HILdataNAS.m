%{
export configuration files for NAS algorithm (for our friends of software development)
%}
switch settings.mission

    case {"2023_Gemini_Portugal_October", "Gemini_Roccaraso_September_2023"}
        folder = "HIL_CPP_files_NAS";
        if ~exist(ConDataPath+"/"+folder,"dir")
            mkdir(ConDataPath+"/"+folder)
        end

        % first file: configuration parameters
        configNAS_export_table = table;
        configValues = [settings.frequencies.NASFrequency,nan,settings.nas.sigma_baro,settings.nas.sigma_GPS,settings.nas.sigma_mag,settings.nas.sigma_beta,settings.nas.sigma_pitot,settings.nas.sigma_w];
        configNASvarNames = {'NAS_FREQUENCY','ACCELERATION_THRESHOLD','SIGMA_BARO','SIGMA_GPS','SIGMA_MAG','SIGMA_BETA','SIGMA_PITOT','SIGMA_W'}; 
        for i = 1:size(configValues,2)
            configNAS_export_table(1,i) = table(configValues(1,i));
        end
        configNAS_export_table.Properties.VariableNames = configNASvarNames;
        writetable(configNAS_export_table,ConDataPath+"/"+folder+"/NAS_configNAS_"+settings.mission+".csv")
        
        
end