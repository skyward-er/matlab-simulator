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
std_plots(simOutput,settings,contSettings)
sensor_plots(simOutput)

%% state visualiser
% animateOrientation(simOutput.Y(:,11),simOutput.Y(:,12),simOutput.Y(:,13),simOutput.Y(:,10),simOutput.t)
% animateOrientation(simOutput.NAS(:,7),simOutput.NAS(:,8),simOutput.NAS(:,9),simOutput.NAS(:,10),simOutput.t_nas)

%% DATA-PRINTING
printOutput(simOutput,settings);

%% save data
% save("Simulation_log.mat","Tf","Yf","data_flight")