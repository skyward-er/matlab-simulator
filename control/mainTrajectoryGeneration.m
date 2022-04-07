%{

mainTrajectoryGeneration - this is the main script; it generates the trajectories that have been chosen in config.m

CALLED SCRIPTS: configTrajectoryGeneration

CALLED FUNCTIONS: Trajectory_generation (SIMULINK model)

REVISIONS:
- 0     16/04/2021, Release,    Alessandro Del Duca
- 1     07/04/2022, update      Davide Rosato
                    Compatibility with common functions folder

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

clear all
close all
clc

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));

%% LOAD DATA
configTrajectoryGeneration;

