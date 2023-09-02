%% export_HILdataABK
% 
% HELP:
% 
% script that exports data for ABK hardware in the loop and software
% testing for air brakes and motor shutdown
%
% check also export_HILdataPRF for parafoil HIL and software testing 

switch settings.mission
   
    case "Gemini_Portugal_October_2023"
        if ~exist(ConDataPath+"/HIL_CPP_files_SFD","dir")
            mkdir(ConDataPath+"/HIL_CPP_files_SFD")
        end
        % first file: configuration for the sensor fault detection
        configSFD_export_table = table;
        configValues = [contSettings.reference.deltaZ, contSettings.filter_coeff0,contSettings.filterMinAltitude,contSettings.filterMaxAltitude,contSettings.criticalAltitude,contSettings.masses_vec(1),contSettings.dmass,simOutput.estimated_mass(end),contSettings.N_forward];
        configSFDvarNames = {'REFERENCE_DZ','STARTING_FILTER_VALUE','CHANGE_FILTER_MINIMUM_ALTITUDE','CHANGE_FILTER_MAXIMUM_ALTITUDE','ABK_CRITICAL_ALTITUDE','LOWEST_MASS','DELTA_MASS','ESTIMATED_MASS','N_FORWARD'};
        for i = 1:size(configValues,2)
            configSFD_export_table(1,i) = table(configValues(1,i));
        end
        configSFD_export_table.Properties.VariableNames = configSFDvarNames;
        writetable(configSFD_export_table,ConDataPath+"/HIL_CPP_files_SFD/SFD_configSFD_"+settings.mission+".csv")
        
end