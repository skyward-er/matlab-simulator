%% export_HILdataABK
% 
% HELP:
% 
% script that exports data for ABK hardware in the loop and software
% testing for air brakes and motor shutdown
%
% check also export_HILdataPRF for parafoil HIL and software testing 

switch settings.mission
   
    case {"2023_Gemini_Portugal_October", "2023_Gemini_Roccaraso_September"}
        folder = "HIL_CPP_files_ABK";
        if ~exist(ConDataPath+"/"+folder,"dir")
            mkdir(ConDataPath+"/"+folder)
        end
        % first file: trajectories, set in the following way:
        % first column - heights;
        % next N_mass columns - vertical velocity with closed air brakes;
        % next N_mass columns - vertical velocity with open air brakes;
        reference_export = zeros(size(contSettings.reference.Z,1),1+2*size(contSettings.reference.Vz,2));
        reference_export(:,1) = contSettings.reference.Z;
        varNames{1,1} = 'Heights';
        for i = 1:size(contSettings.reference.Vz,1)
            for j = 1:size(contSettings.reference.Vz,2)
                reference_export(:,1+(i-1)*size(contSettings.reference.Vz,2)+j) = contSettings.reference.Vz{i,j};
                if i == 1
                    varNames{1,1+(i-1)*size(contSettings.reference.Vz,2)+j} = ['Vz_closed_m',num2str(contSettings.masses_vec(j))];
                else
                    varNames{1,1+(i-1)*size(contSettings.reference.Vz,2)+j} = ['Vz_open_m',num2str(contSettings.masses_vec(j))];
                end
            end
        end
        varNames = replace(varNames,'.','_');
        for i = 1:size(reference_export,2)
            reference_export_table(:,i) = table(reference_export(:,i));
        end
        reference_export_table.Properties.VariableNames = varNames;
        writetable(reference_export_table,ConDataPath+"/"+folder+"/ABK_references_"+settings.mission+".csv")

        % second file: configuration for the air brakes
        configABK_export_table = table;
        configValues = [settings.frequencies.arbFrequency, contSettings.reference.deltaZ, contSettings.filter_coeff0,contSettings.filterMinAltitude,contSettings.filterMaxAltitude,contSettings.criticalAltitude,contSettings.masses_vec(1),contSettings.dmass,simOutput.sensors.mea.mass(end),contSettings.N_forward,contSettings.ABK_shutdown_delay];
        configABKvarNames = {'ABK_FREQUENCY','REFERENCE_DZ','STARTING_FILTER_VALUE','CHANGE_FILTER_MINIMUM_ALTITUDE','CHANGE_FILTER_MAXIMUM_ALTITUDE','ABK_CRITICAL_ALTITUDE','ABK_REFERENCE_LOWEST_MASS','DELTA_MASS','ESTIMATED_MASS','N_FORWARD','ABK_DELAY_FROM_SHUTDOWN'};
        for i = 1:size(configValues,2)
            configABK_export_table(1,i) = table(configValues(1,i));
        end
        configABK_export_table.Properties.VariableNames = configABKvarNames;
        writetable(configABK_export_table,ConDataPath+"/"+folder+"/ABK_configABK_"+settings.mission+".csv")

        % third file: input extrapolation:
        % first column - timestamps of the NAS system
        % second column - data with vertical position of the NAS;
        % third column - data with vertical velocity of the NAS;
        trajectory_export_table = table;
        trajectory = zeros(size(simOutput.sensors.nas.time(300:end)'));
        trajectory(:,1) = -simOutput.sensors.nas.states(300:end,3);
        trajectory(:,2) = -simOutput.sensors.nas.states(300:end,6);
        traj_varNames = {'Z','Vz'};
        for i = 1:size(trajectory,2)
            trajectory_export_table(:,i) = table(trajectory(:,i));
        end
        trajectory_export_table.Properties.VariableNames = traj_varNames;
        writetable(trajectory_export_table,ConDataPath+"/"+folder+"/ABK_trajectories_"+settings.mission+".csv")

        % fourth file: input extrapolation:
        % first column - data with air brakes timestamps;
        % second column - data with air brakes output of the simulation;
        %
        % to simplify the testing the ABK algorithm is re-run with the data
        % of the third file (trajectory)
        % recall the initial conditions for the controls
        configControl;
        configControlParams;
        configReferences;
        contSettings_choosemass = trajectoryChoice_mass(simOutput.sensors.mea.mass(end),contSettings);
        ABK_recall = zeros(length(trajectory(:,2)),1);
        ABK_recall_old = 0;
        for i = 1:length(trajectory(:,2))
            [ABK_recall(i),contSettings_choosemass] = control_Interp(trajectory(i,1),trajectory(i,2),settings,contSettings_choosemass,ABK_recall_old);
            ABK_recall_old = ABK_recall(i);
        end
        ABK_recall = ABK_recall/settings.servo.maxAngle;
        outputABK_export_table = table;
        ABK_varNames = {'ABK'};
        for i = 1:size(ABK_recall,2)
            outputABK_export_table(:,i) = table(ABK_recall(:,i));
        end
        outputABK_export_table.Properties.VariableNames = ABK_varNames;
        writetable(outputABK_export_table,ConDataPath+"/"+folder+"/ABK_outputABK_"+settings.mission+".csv")
        
        
end