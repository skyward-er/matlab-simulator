%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

REVISIONS:
- #0 07/04/2022, Release, Davide Rosato

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

%% MISSION FILE
% Choose the mision you want to simulate from rocketsData folder
% settings.mission = 'Lynx_Roccaraso_September_2021';
settings.mission = 'Lynx_Portugal_October_2021';
% settings.mission = 'Pyxis_Portugal_October_2022';
% settings.mission = 'Pyxis_Roccaraso_September_2022';

%% LOAD DATA
% Retrieve MSA-Toolkit rocket data
dataPath = strcat('../data/msa-toolkit/data/', settings.mission);
addpath(dataPath);
simulationsData

commonFunctionsPath = '../data/msa-toolkit/commonFunctions';
addpath(genpath(commonFunctionsPath))
% Retrieve Control rocket data
dataPath = strcat('../data/', settings.mission);
addpath(dataPath);
run(strcat('config',settings.mission));

%% AEROBRAKES EXTENSION DISCRETIZATION
% Lower and upper boundaries are set in rocket simulationsData file
settings.dX = 0.001;          % [m] Discretization of airbrakes radial extension

%% COMPATIBILITY SETTINGS !! DO NOT CHANGE UNLESS YOU KNOW WHAT YOU ARE DOING !!
%%% VERTICAL VELOCITY FOR BACK PROPAGATION
% Launchpad
settings.OMEGA = 85*pi/180;                       % [rad] Minimum Elevation Angle, user input in degrees (ex. 80)
settings.PHI = 0*pi/180;                          % [rad] Minimum Azimuth Angle from North Direction, user input in degrees (ex. 90)
% Wind
settings.wind.MagMin = 0.01;                      % [m/s] Minimum Magnitude
settings.wind.MagMax = 0.01;                      % [m/s] Maximum Magnitude
settings.wind.ElMin = 0*pi/180;                   % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax = 0*pi/180;                   % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin = (360)*pi/180;               % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax = (360)*pi/180;
% Event function
settings.ode.optionsascTrajGen = odeset('Events', @eventBurnOff);
settings.stoch.N = 1;
settings.wind.input = false;
settings.wind.model = false;



