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
        if ~exist(ConDataPath+"/HIL_CPP_files_ABK","dir")
            mkdir(ConDataPath+"/HIL_CPP_files_ABK")
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
        writetable(reference_export_table,ConDataPath+"/HIL_CPP_files_ABK/ABK_references_"+settings.mission+".csv")

        % second file: configuration for the air brakes
        configValues = [contSettings.reference.deltaZ, contSettings.filter_coeff0,contSettings.filterMinAltitude,contSettings.filterMaxAltitude,contSettings.criticalAltitude,contSettings.masses_vec(1),contSettings.dmass,simOutput.estimated_mass];
        configABKvarNames = {'REFERENCE_DZ','STARTING_FILTER_VALUE','CHANGE_FILTER_MINIMUM_ALTITUDE','CHANGE_FILTER_MAXIMUM_ALTITUDE','ABK_CRITICAL_ALTITUDE','LOWEST_MASS','DELTA_MASS','ESTIMATED_MASS'};
        for i = 1:size(configValues,2)
            configABK_export_table(1,i) = table(configValues(i));
        end
        configABK_export_table.Properties.VariableNames = configABKvarNames;
        writetable(configABK_export_table,ConDataPath+"/HIL_CPP_files_ABK/ABK_configABK_"+settings.mission+".csv")

        % third file: input - output extrapolation:
        % first column - timestamps of the NAS system
        % second column - data with vertical position of the NAS;
        % third column - data with vertical velocity of the NAS;
        trajectory_export_table = table;
        trajectory = zeros(size(simOutput.t_nas'));
        trajectory(:,1) = simOutput.t_nas';
        trajectory(:,2) = -simOutput.NAS(:,3);
        trajectory(:,3) = -simOutput.NAS(:,6);
        traj_varNames = {'Time','Z','Vz'};
        for i = 1:size(trajectory,2)
            trajectory_export_table(:,i) = table(trajectory(:,i));
        end
        trajectory_export_table.Properties.VariableNames = traj_varNames;
        writetable(trajectory_export_table,ConDataPath+"/HIL_CPP_files_ABK/ABK_trajectories_"+settings.mission+".csv")

        % fourth file: input - output extrapolation:
        % first column - data with air brakes timestamps;
        % second column - data with air brakes output of the simulation;
        outputABK_export_table = table;
        ABK_output = zeros(size(simOutput.t_nas'));
        ABK_output(:,1) = simOutput.t_nas';
        ABK_output(:,2) = interp1(simOutput.ARB_cmdTime',simOutput.ARB_cmd/settings.servo.maxAngle,simOutput.t_nas);
        ABK_varNames = {'Time','ABK'};
        for i = 1:size(ABK_output,2)
            outputABK_export_table(:,i) = table(ABK_output(:,i));
        end
        outputABK_export_table.Properties.VariableNames = ABK_varNames;
        writetable(outputABK_export_table,ConDataPath+"/HIL_CPP_files_ABK/ABK_outputABK_"+settings.mission+".csv")
        
        
end