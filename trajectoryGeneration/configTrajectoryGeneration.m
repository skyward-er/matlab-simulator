%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

REVISIONS:
- #0 07/04/2022, Release, Davide Rosato, AFD Department

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

%% MISSION FILE
% Choose the mision you want to simulate from rocketsData folder
% settings.mission = 'Lynx_Roccaraso_September_2021';
% settings.mission = 'Lynx_Portugal_October_2021';
settings.mission = 'Pyxis_Portugal_October_2022';
%  settings.mission = 'Pyxis_Roccaraso_September_2022';

%% LOAD DATA
% Retrieve MSA-Toolkit rocket data
dataPath = strcat('../data/msa-toolkit/data/', settings.mission);
addpath(dataPath);
simulationsData

commonFunctionsPath = '../data/msa-toolkit/commonFunctions';
addpath(genpath(commonFunctionsPath))

% Retrieve Control rocket data
ConDataPath = strcat('../data/', settings.mission);
addpath(ConDataPath);
run(strcat('config', settings.mission));

% Control common functions
commonFunctionsPath = '../commonFunctions';
addpath(genpath(commonFunctionsPath))


%% AEROBRAKES EXTENSION DISCRETIZATION
settings.Ndx = 10;                                % [m] Number of trajectories

%% FINAL VERTICAL VELOCITY
settings.Vz_initialPerc = 0.05;                   % [-] Percentage of increasing the initial vertical velocity

%% LAUNCHPAD
settings.OMEGA = 83*pi/180;                       % [rad] Minimum Elevation Angle, user input in degrees (ex. 80)
settings.PHI = 133*pi/180;                        % [rad] Minimum Azimuth Angle from North Direction, user input in degrees (ex. 90)

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



