%{

mainSimulator - this is the main script; it runs the rocket ascent with
every possible feature simulated (i.e. burning phase, attitude, air
braking, ADA, NAS, Kalman, Parafoil, landing). 

REVISIONS:
- 0     xx/xx/xxxx, Release,    Ruben Di Battista
- 1     16/04/2021, update,     Alessandro Del Duca

- 2     07/04/2022, update,     Davide Rosato, AFD Departmwent
                    Compatibility with common functions folder

- 3     15/04/2022, update,     Marco Marchesi, SCS department
                    Latest control algorithms

- 4     01/09/2023, update,     Marco Marchesi, GNC IPT
                                Giuseppe Brentino, GNC IPT
                                Pier Francesco Bachini, GNC IPT
                                Alessandro Donadi, GNC IPT
                    Parafoil simulation, latest control algorithms, 
                    sensor fault detection, engine shut-down, and much
                    more.

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

% Whether to run the simulator in minimal mode(data generation only) or not
minimal = exist('minimal','var') && minimal == true;
% Whether to check if the submodules are updated or not
flagSubmodulesUpdated = exist('flagSubmodulesUpdated','var') && flagSubmodulesUpdated == true;

close all; clc;
clearvars -except minimal flagSubmodulesUpdated;

%% Setup paths

restoredefaultpath;
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

%% Check submodules status
if ~flagSubmodulesUpdated && ~minimal
    checkSubmodules;
    flagSubmodulesUpdated = true;
end

%% Configs

conf.script = "simulator"; % this defines which configuration scripts to run
settings.montecarlo = false;
configSimulator;

%% Algorithm tuning
% basically if this is true sets the randomic value of the wind to the same
% values for each simulation, so it has the same atmospheric conditions
% each time

if settings.tuning
  	rng('default');
end 

%% Simulation
[simOutput] = std_run(settings,contSettings);

%% Plots
if ~minimal
    if ~exist("../commonFunctions/graphics/general-utilities/","dir")
        warning('To export file you need to download the repository, read the README file in the folder')
    end
    std_plots(simOutput,settings,contSettings)
    sensor_plots(simOutput)
end

%% DATA-PRINTING
printOutput(simOutput,settings);
