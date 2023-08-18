%{

mainSimulator - this is the main script; it runs the rocket ascent with
every possible feature simulated (i.e. burning phase, attitude, air
braking, ADA, NAS, Kalman, Parafoil, landing). 

REVISIONS:
- 0     xx/xx/xxxx, Release,    Ruben Di Battista
- 1     16/04/2021, update,     Alessandro Del Duca

- 2     07/04/2022, update,     Davide Rosato, AFD Department
                    Compatibility with common functions folder

- 3     15/04/2022, update,     Marco Marchesi, SCS department
                    Latest control algorithms

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

close all; clear; clc;

%% path loading

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end
commonFunctionsPath = '../commonFunctions';
addpath(genpath(currentPath));

% Common Functions path
addpath(genpath(commonFunctionsPath));

%% CHECK IF MSA-TOOLKIT IS UPDATED
msaToolkitURL = 'https://github.com/skyward-er/msa-toolkit';
localRepoPath = '../data/msa-toolkit';
%  status = checkLastCommit(msaToolkitURL, localRepoPath, pwd);
%  submoduleAdvice(status, msaToolkitURL, localRepoPath, pwd);

%% CONFIGs

conf.script = "simulator"; % this defines which configuration scripts to run
config; 

%% ALGORITHM TUNING
% basically if this is true sets the randomic value of the wind to the same
% values for each simulation, so it has the same atmospheric conditions
% each time

if settings.tuning
  	rng('default')
end 

%% SET SPECIFIC PARAMETERS FOR A PRE LAUNCH SIMULATION

%  config_SpecialConditions;

%% START THE SIMULATION
% T = vector of time used by ODE, [s] also for Tf Ta
% Y = State = ( x y z | u v w | p q r | q0 q1 q2 q3 | thetax thetay thetaz | ap_ref ) also for Ya,Yf corresponding to T

%% simulation:
[simOutput] = std_run(settings,contSettings);

%% PLOTS
if ~exist("../commonFunctions/graphics/general-utilities/","dir")
    warning('To export file you need to download the repository, read the README file in the folder')
end
std_plots(simOutput,settings,contSettings)
sensor_plots(simOutput)

%% state visualiser
animateOrientation(simOutput.Y(:,11),simOutput.Y(:,12),simOutput.Y(:,13),simOutput.Y(:,10),simOutput.t)
% animateOrientation(simOutput.NAS(:,7),simOutput.NAS(:,8),simOutput.NAS(:,9),simOutput.NAS(:,10),simOutput.t_nas)

%% DATA-PRINTING
printOutput(simOutput,settings);

%% save data
% save("Simulation_log.mat","Tf","Yf","data_flight")

%% export data for HIL simulations /cpp usage: 
if settings.exportCSV % this is set in configReferences
    switch settings.mission
        case "Gemini_Portugal_October_2023"
            % first file: trajectories, set in the following way: 
            % first column - heights; 
            % next N_mass columns - vertical velocity with closed air brakes;
            % next N_mass columns - vertical velocity with open air brakes;
            mkdir(ConDataPath+"/Trajectories_CSV")
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
            writetable(reference_export_table,ConDataPath+"/Trajectories_CSV/references_"+settings.mission+".csv")
            
            % second file: configuration for the air brakes
            configValues = [contSettings.reference.deltaZ, contSettings.filter_coeff0,contSettings.filterMinAltitude,contSettings.filterMaxAltitude,contSettings.criticalAltitude,contSettings.masses_vec(1),contSettings.dmass];
            configABKvarNames = {'REFERENCE_DZ','STARTING_FILTER_VALUE','CHANGE_FILTER_MINIMUM_ALTITUDE','CHANGE_FILTER_MAXIMUM_ALTITUDE','ABK_CRITICAL_ALTITUDE','LOWEST_MASS','DELTA_MASS'};
            for i = 1:size(configValues,2)
                configABK_export_table(1,i) = table(configValues(i));
            end
            configABK_export_table.Properties.VariableNames = configABKvarNames;
            writetable(configABK_export_table,ConDataPath+"/Trajectories_CSV/configABK_"+settings.mission+".csv")

            % third file: input - output extrapolation:
            % first column - timestamps of the NAS system
            % second column - data with vertical position of the NAS;
            % third column - data with vertical velocity of the NAS;
            trajectory_export_table = table;
            trajectory = zeros(size(simOutput.t_nas'));
            trajectory(:,1) = simOutput.t_nas;
            trajectory(:,2) = -simOutput.NAS(:,3);
            trajectory(:,3) = -simOutput.NAS(:,6);
            traj_varNames = {'Time','Z','Vz'};
            for i = 1:size(trajectory,2)
                trajectory_export_table(:,i) = table(trajectory(:,i));
            end
            trajectory_export_table.Properties.VariableNames = traj_varNames;
            writetable(trajectory_export_table,ConDataPath+"/Trajectories_CSV/trajectories_"+settings.mission+".csv")

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
            writetable(outputABK_export_table,ConDataPath+"/Trajectories_CSV/outputABK_"+settings.mission+".csv")

    end
end
