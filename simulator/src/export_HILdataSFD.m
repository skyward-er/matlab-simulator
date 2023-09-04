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
        configValues = [settings.sfd.rough_apogee_estimate, settings.sfd.press_ref, settings.sfd.max_rough_press,settings.sfd.max_weight,settings.sfd.min_step,settings.sfd.max_step,settings.sfd.z0, settings.sfd.filter_window, settings.sfd.lowpass_filter_gain, settings.sfd.lowpass_filter_cutoff_freq, settings.sfd.lambda_baro, settings.SVM_1.Scale, settings.SVM_1.Mu(1,1), settings.SVM_1.Mu(1,2), settings.SVM_1.Mu(1,3), settings.SVM_1.Mu(1,4), settings.SVM_1.Mu(1,5), settings.SVM_1.Mu(1,6), settings.SVM_1.Sigma(1,1), settings.SVM_1.Sigma(1,2), settings.SVM_1.Sigma(1,3), settings.SVM_1.Sigma(1,4), settings.SVM_1.Sigma(1,5), settings.SVM_1.Sigma(1,6), settings.SVM_1.Beta(1,1), settings.SVM_1.Beta(2,1), settings.SVM_1.Beta(3,1), settings.SVM_1.Beta(4,1), settings.SVM_1.Beta(5,1), settings.SVM_1.Beta(6,1), settings.SVM_1.Bias, settings.SVM_2.Scale, settings.SVM_2.Mu(1,1), settings.SVM_2.Mu(1,2), settings.SVM_2.Mu(1,3), settings.SVM_2.Mu(1,4), settings.SVM_2.Mu(1,5), settings.SVM_2.Sigma(1,1), settings.SVM_2.Sigma(1,2), settings.SVM_2.Sigma(1,3), settings.SVM_2.Sigma(1,4), settings.SVM_2.Sigma(1,5), settings.SVM_2.Beta(1,1), settings.SVM_2.Beta(2,1), settings.SVM_2.Beta(3,1), settings.SVM_2.Beta(4,1), settings.SVM_2.Beta(5,1), settings.SVM_2.Bias, settings.SVM_1.N_sample, settings.SVM_1.N_sample_fft, settings.SVM_2.N_sample, settings.SVM_1.takeoff_shadowmode];
        configSFDvarNames = {'ROUGH_APOGEE_ESTIMATE','INITIAL_PRESSURE_ESTIMATE_Z0','MAXIMUM_PRESSURE_ESTIMATE','MAX_WEIGHT','MIN_WEIGHT_CHANGE','MAX_WEIGHT_CHANGE','MIN_HEIGHT_ESTIMATION', 'MEDIAN_FILTER_WINDOW_SIZE', 'LOWPASS_FILTER_GAIN', 'LOWPASS_FILTER_CUTOFF_FREQ', 'LOWPASS_FILTER_LAMBDA', 'SVM1_SCALE', 'SVM1_MU_1', 'SVM1_MU_2', 'SVM1_MU_3', 'SVM1_MU_4', 'SVM1_MU_5', 'SVM1_MU_6', 'SVM1_SIGMA_1', 'SVM1_SIGMA_2', 'SVM1_SIGMA_3', 'SVM1_SIGMA_4', 'SVM1_SIGMA_5', 'SVM1_SIGMA_6',  'SVM1_BETA_1', 'SVM1_BETA_2','SVM1_BETA_3','SVM1_BETA_4','SVM1_BETA_5', 'SVM1_BETA_6', 'SVM1_BIAS', 'SVM2_SCALE', 'SVM2_MU_1', 'SVM2_MU_2', 'SVM2_MU_3', 'SVM2_MU_4', 'SVM2_MU_5', 'SVM2_SIGMA_1', 'SVM2_SIGMA_2', 'SVM2_SIGMA_3', 'SVM2_SIGMA_4', 'SVM2_SIGMA_5',  'SVM2_BETA_1', 'SVM2_BETA_2','SVM2_BETA_3','SVM2_BETA_4','SVM2_BETA_5', 'SVM2_BIAS', 'N_SAMPLE_SVM1', 'FFT_N_SAMPLE_SVM1', 'N_SAMPLE_SVM2', 'SVM1_TAKEOFF_SHADOWMODE'};
        for i = 1:size(configValues,2)
            configSFD_export_table(1,i) = table(configValues(1,i));
        end
        configSFD_export_table.Properties.VariableNames = configSFDvarNames;
        writetable(configSFD_export_table,ConDataPath+"/HIL_CPP_files_SFD/SFD_configSFD_"+settings.mission+".csv")
        
end