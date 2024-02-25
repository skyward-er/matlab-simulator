%% export_HILdataPRF
% 
% HELP:
% 
% script that exports data for PRF hardware in the loop and software
% testing for parafoil system
%
% check also export_HILdataABK for air brakes HIL and software testing 

switch settings.mission
   
    case "2023_Gemini_Portugal_October"
        if ~exist(ConDataPath+"/HIL_CPP_files_PRF","dir")
            mkdir(ConDataPath+"/HIL_CPP_files_PRF")
        end
        % first file: input - output extrapolation:
        % first column - timestamps of the NAS system
        % second column - data with vertical position of the NAS;
        % third column - data with vertical velocity of the NAS;
        trajectory_export_table = table;
        PRFtrajectory = zeros(size(simOutput.t_nas(simOutput.events.mainChuteIndex:end)'));
        PRFtrajectory(:,1) = simOutput.NAS(simOutput.events.mainChuteIndex:end,1);
        PRFtrajectory(:,2) = simOutput.NAS(simOutput.events.mainChuteIndex:end,2);
        PRFtrajectory(:,3) = simOutput.NAS(simOutput.events.mainChuteIndex:end,3)+settings.z0;
        PRFtrajectory(:,4) = simOutput.NAS(simOutput.events.mainChuteIndex:end,4);
        PRFtrajectory(:,5) = simOutput.NAS(simOutput.events.mainChuteIndex:end,5);
        traj_varNames = {'N','E','D','Vn','Ve'};
        for i = 1:size(PRFtrajectory,2)
            trajectory_export_table(:,i) = table(PRFtrajectory(:,i));
        end
        trajectory_export_table.Properties.VariableNames = traj_varNames;
        writetable(trajectory_export_table,ConDataPath+"/HIL_CPP_files_PRF/PRF_trajectories_"+settings.mission+".csv")
        

        % second file: WES output extrapolation:
        % first column - x wind velocity;
        % second column - y wind velocity;
        
        % recall initial configuration
        configControl;
        outputWES_export_table = table;
        WES_output = zeros(size(PRFtrajectory(:,1:2)));
        for i = 1:length(PRFtrajectory(:,1))
            contSettings.WES = run_WES(PRFtrajectory(i,4:5),contSettings.WES);
            WES_output(i,:) = contSettings.WES.wind_est;
        end
        WES_varNames = {'Wx','Wy'};
        for i = 1:size(WES_output,2)
            outputWES_export_table(:,i) = table(WES_output(:,i));
        end
        outputWES_export_table.Properties.VariableNames = WES_varNames;
        writetable(outputWES_export_table,ConDataPath+"/HIL_CPP_files_PRF/PRF_outputWES_"+settings.mission+".csv")

        % third file: configuration file for WES
        configPRF_export_table = table;
        configValues = [settings.payload.target(1),settings.payload.target(2)...
            simOutput.payload.EMC(1),simOutput.payload.EMC(2),...
            simOutput.payload.M1(1),simOutput.payload.M1(2),...
            simOutput.payload.M2(1),simOutput.payload.M2(2),...
            contSettings.payload.Kp,contSettings.payload.Ki];
        configPRFvarNames = {'TARGET_X','TARGET_Y','EMC_X','EMC_Y','M1_X','M1_Y','M2_X','M2_Y','KP','KI'};
        for i = 1:size(configValues,2)
            configPRF_export_table(1,i) = table(configValues(1,i));
        end
        configPRF_export_table.Properties.VariableNames = configPRFvarNames;
        writetable(configPRF_export_table,ConDataPath+"/HIL_CPP_files_PRF/PRF_configGuidance_"+settings.mission+".csv")
        

        % fourth file: guidance output
        outputGuidance_export_table = table;
        guidance_output = zeros(size(PRFtrajectory(:,1:2)));
        pos_est = PRFtrajectory(:,1:3);
        pos_est(:,3) = -pos_est(:,3);
        for i = 1:length(PRFtrajectory(:,1))
            guidance_output(i,1) = atan2(settings.payload.target(2)-PRFtrajectory(i,2),settings.payload.target(1)-PRFtrajectory(i,1));
            guidance_output(i,2) = EMguidance(pos_est(i,:),settings.payload.target,simOutput.payload.EMC,simOutput.payload.M1,simOutput.payload.M2);
        end
        WES_varNames = {'PI','EMC'};
        for i = 1:size(guidance_output,2)
            outputGuidance_export_table(:,i) = table(guidance_output(:,i));
        end
        outputGuidance_export_table.Properties.VariableNames = WES_varNames;
        writetable(outputGuidance_export_table,ConDataPath+"/HIL_CPP_files_PRF/PRF_outputGuidance_"+settings.mission+".csv")

        % fifth file: control output
        % recall contSettings
        configControl;
        outputPRFcontrol_export_table = table;
        PRFcontrol_output = zeros(size(PRFtrajectory(:,1:2)));
        vel_est = PRFtrajectory(:,4:5);
        sat = [false,false];
        I = [0, 0];
        for i = 1:length(PRFtrajectory(:,1))
            
            
            % compute velocity difference
            deltaVel = vel_est(i,:) - WES_output(i,:);
            psi = atan2(deltaVel(2),deltaVel(1));
            % compute heading difference
            error_psi(1) = angdiff(psi,guidance_output(i,1)); 
            error_psi(2) = angdiff(psi,guidance_output(i,2)); 
        
            P(1) = contSettings.payload.Kp * error_psi(1);
            P(2) = contSettings.payload.Kp * error_psi(2);
            if sat(1) == false
                I(1) = contSettings.payload.Ki * error_psi(1) + I(1);
            end 
            if sat(2) == false
                I(2) = contSettings.payload.Ki * error_psi(2) + I(2);
            end 
            % compute control action
            deltaA(1) = P(1) + I(1);
            deltaA(2) = P(2) + I(2);

            [PRFcontrol_output(i,1),sat(1)] = Saturation(deltaA(1),contSettings.payload.uMin, contSettings.payload.uMax);
            [PRFcontrol_output(i,2),sat(2)] = Saturation(deltaA(2),contSettings.payload.uMin, contSettings.payload.uMax);

        end
        PRFcontrol_varNames = {'PI','EMC'};
        for i = 1:size(PRFcontrol_output,2)
            outputPRFcontrol_export_table(:,i) = table(PRFcontrol_output(:,i));
        end
        outputPRFcontrol_export_table.Properties.VariableNames = PRFcontrol_varNames;
        writetable(outputPRFcontrol_export_table,ConDataPath+"/HIL_CPP_files_PRF/PRF_outputControl_"+settings.mission+".csv")


end