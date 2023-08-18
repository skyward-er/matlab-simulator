%% export_HILdataPRF
% 
% HELP:
% 
% script that exports data for PRF hardware in the loop and software
% testing for parafoil system
%
% check also export_HILdataABK for air brakes HIL and software testing 

switch settings.mission
   
    case "Gemini_Portugal_October_2023"
        if ~exist(ConDataPath+"/HIL_CPP_files_PRF","dir")
            mkdir(ConDataPath+"/HIL_CPP_files_PRF")
        end
        % first file: input - output extrapolation:
        % first column - timestamps of the NAS system
        % second column - data with vertical position of the NAS;
        % third column - data with vertical velocity of the NAS;
        trajectory_export_table = table;
        trajectory = zeros(size(simOutput.t_nas(simOutput.events.mainChuteIndex:end)'));
        trajectory(:,1) = simOutput.t_nas(simOutput.events.mainChuteIndex:end)';
        trajectory(:,2) = simOutput.NAS(simOutput.events.mainChuteIndex:end,1);
        trajectory(:,3) = simOutput.NAS(simOutput.events.mainChuteIndex:end,2);
        trajectory(:,4) = simOutput.NAS(simOutput.events.mainChuteIndex:end,3);
        trajectory(:,5) = simOutput.NAS(simOutput.events.mainChuteIndex:end,4);
        trajectory(:,6) = simOutput.NAS(simOutput.events.mainChuteIndex:end,5);
        traj_varNames = {'Time','N','E','D','Vn','Ve'};
        for i = 1:size(trajectory,2)
            trajectory_export_table(:,i) = table(trajectory(:,i));
        end
        trajectory_export_table.Properties.VariableNames = traj_varNames;
        writetable(trajectory_export_table,ConDataPath+"/HIL_CPP_files_PRF/PRF_NAS_output_"+settings.mission+".csv")
        

        % second file: input - output extrapolation:
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
        writetable(outputABK_export_table,ConDataPath+"/HIL_CPP_files_PRF/PRF_outputABK_"+settings.mission+".csv")

end