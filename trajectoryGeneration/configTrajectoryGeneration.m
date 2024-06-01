%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

REVISIONS:
- #0 07/04/2022, Release, Davide Rosato, AFD Department

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

%% LOAD DATA
% Retrieve MSA-Toolkit rocket data
dataPath = strcat('../common/missions/', mission.nameMSA);
addpath(dataPath);
simulationsData

commonFunctionsPath = '../common/functions';
addpath(genpath(commonFunctionsPath))

% Retrieve Control rocket data
ConDataPath = strcat('../data/', mission.name);
addpath(ConDataPath);
run(strcat('config', mission.name));

% Control common functions
commonFunctionsPath = '../commonFunctions';
addpath(genpath(commonFunctionsPath))


%% AEROBRAKES EXTENSION DISCRETIZATION
settings.Ndx = 10;                                % [m] Number of trajectories

%% FINAL VERTICAL VELOCITY
settings.Vz_initialPerc = 0.05;                   % [-] Percentage of increasing the initial vertical velocity

%% WIND
settings.wind.MagMin = 0;                         % [m/s] Minimum Magnitude
settings.wind.MagMax = 0;                         % [m/s] Maximum Magnitude
settings.wind.ElMin = 0*pi/180;                   % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax = 0*pi/180;                   % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin = (360)*pi/180;               % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax = (360)*pi/180;

%% SAVINGS
settings.save = true;

%% PLOT
settings.plots = true;

%% ODE SETTINGS
settings.ode.optionsascTrajGen = odeset('Events', @eventAirBrake);

%% COMPATIBILITY SETTINGS !! DO NOT CHANGE UNLESS YOU KNOW WHAT YOU ARE DOING !!
settings.stoch.N = 1;
settings.wind.input = false;
settings.wind.model = false;
settings.control = 1;



