%{
    export get drag coefficients
%}


switch mission.name
    case  {"Gemini_Portugal_October_2023", "Gemini_Roccaraso_September_2023"}
        folder_MTR = "HIL_CPP_files_MTR";
        if ~exist(ConDataPath+"/"+folder_MTR,"dir")
            mkdir(ConDataPath+"/"+folder_MTR)
        end

        % first file: aerodynamic coefficients
        CD_coefficients  = [contSettings.coeff_Cd.n000,contSettings.coeff_Cd.n100,...
            contSettings.coeff_Cd.n200,contSettings.coeff_Cd.n300,contSettings.coeff_Cd.n400,...
            contSettings.coeff_Cd.n500,contSettings.coeff_Cd.n600];
        CD_coeff_table = table;
        varNames = {'n000','n100','n200','n300','n400','n500','n600'};
        for i = 1:size(CD_coefficients,2)
            CD_coeff_table(:,i) = table(CD_coefficients(:,i));
        end
        CD_coeff_table.Properties.VariableNames = varNames;
        writetable(CD_coeff_table,ConDataPath+"/"+folder_MTR+"/MTR_CDcoeffShutDown_"+mission.name+".csv")

       
        % second file: configuration for mass estimation algorithm
        configMTR_export_table = table;
        configValues = [settings.frequencies.MEAFrequency,settings.mea.t_lower_shadowmode,settings.mea.t_higher_shadowmode,contSettings.N_prediction_threshold,settings.mea.Q(1,1),settings.mea.Q(2,2),settings.mea.Q(3,3),settings.mea.R,rocket.mass(1),settings.mea.z_shutdown];
        configMTRvarNames = {'MEA_FREQUENCY','MEA_SHADOWMODE','SHUTDOWN_MAX_TIME','MEA_PREDICTION_THRESHOLD','MEA_Q_11','MEA_Q_22','MEA_Q_33','MEA_R','MEA_INIT_MASS','MEA_SHUTDOWN_TARGET_APOGEE'};
        for i = 1:size(configValues,2)
            configMTR_export_table(1,i) = table(configValues(1,i));
        end
        configMTR_export_table.Properties.VariableNames = configMTRvarNames;
        writetable(configMTR_export_table,ConDataPath+"/"+folder_MTR+"/MTR_configMTR_"+mission.name+".csv")

        % third file: configuration for the air brakes
        configMEA_MODEL_export_table = table;
        configValues = [settings.mea.engine_model_A1(1,1),settings.mea.engine_model_A1(1,2),settings.mea.engine_model_A1(1,3),settings.mea.engine_model_A1(2,1),settings.mea.engine_model_A1(2,2),settings.mea.engine_model_A1(2,3),settings.mea.engine_model_A1(3,1),settings.mea.engine_model_A1(3,2),settings.mea.engine_model_A1(3,3)];
        configMEA_MODELvarNames = {'F_11','F_12','F_13','F_21','F_22','F_23','F_31','F_32','F_33'};
        for i = 1:size(configValues,2)
            configMEA_MODEL_export_table(1,i) = table(configValues(1,i));
        end
        configMEA_MODEL_export_table.Properties.VariableNames = configMEA_MODELvarNames;
        writetable(configMEA_MODEL_export_table,ConDataPath+"/"+folder_MTR+"/MTR_mea_model_"+mission.name+".csv")


end