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
% settings.mission = 'Pyxis_Portugal_October_2022';
settings.mission = 'Pyxis_Roccaraso_September_2022';
% settings.mission = 'NewRocket_2023';

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



%% ALGORITHM TUNING
settings.tuning = true;                 % [-] True if you want to tune the algorithm

%% SIMULATION SETTINGS
settings.launchWindow      = false;  % Switch off this to avoid pausing the launch till you press the launch button
settings.electronics       = false;  % Switch on when testing with Hardware in the loop HIL - NOT IMPLEMENTED YET, STILL TO BE MERGED
settings.ascentOnly        = true; % Switch on to simulate only the ascent phase untill the apogee
settings.ballisticFligth   = true;  % Switch on to simulate the balistic fligth without any parachute
settings.control           = true;  % Switch on to simulate the control
settings.dataNoise         = true;  % Switch on to simulate the data acquisiton from sensors
settings.Kalman            = true;  % Switch on to run the kalman algorithm - note, also to run the airbrakes control algorithm this is needed true
settings.Ada               = true;  % Switch on to run the apogee detection algorithm
settings.HRE               = false; % Switch on if the rocket is mounting a Hybrid Engine, which allows the possibility to shut down the engine
settings.machControlActive = false; % Switch on the mach control in ascentControl.m

% post processing
settings.postProcessing = true;
% compatibility check - do not change
if settings.electronics
    settings.launchWindow = true;
    settings.Kalman       = false;
    settings.Ada          = false;
    settings.control      = true;


    % add path for Hardware In the Loop
    addpath('../hardware_in_the_loop/');
    addpath('../hardware_in_the_loop/serialbridge');
    run('HILconfig.m');
    serialbridge("Open", hil_settings.serial_port, hil_settings.baudrate); % Initialization of the serial port
end

%%% uncomment when the MSA toolkit is updated

% if settings.mission == 'NewRocket_2023' 
%     settings.HRE = true;
% else
%     settings.HRE = false;
% end

%% LAUNCH SETUP
% launchpad directions
% for a single run the maximum and the minimum value of the following angles must be the same.
settings.OMEGA = 83*pi/180;              % [rad] Minimum Elevation Angle, user input in degrees (ex. 80)
settings.PHI = 170*pi/180;                 % [rad] Minimum Azimuth Angle from North Direction, user input in degrees (ex. 90)

%% WIND DETAILS
% select which model you want to use:
% three different models, 
%%%%% Matlab Wind Model
settings.wind.model = false;
% matlab hswm model, wind model on altitude based on historical data

% input Day and Hour as arrays to run stochastic simulations
settings.wind.DayMin = 105;                    % [d] Minimum Day of the launch
settings.wind.DayMax = 105;                    % [d] Maximum Day of the launch
settings.wind.HourMin = 4;                     % [h] Minimum Hour of the day
settings.wind.HourMax = 4;                     % [h] Maximum Hour of the day
settings.wind.ww = 0;                          % [m/s] Vertical wind speed

%%%%% Input wind
settings.wind.input = true;
% Wind is generated for every altitude interpolating with the coefficient defined below
if settings.wind.input == true && settings.wind.model == true
    warning("you are trying to use 'input model' but is shadowed by 'wind model' ")
end

settings.wind.inputGround  = 4;                                         % [m/s] Wind magnitude at the ground
settings.wind.inputAlt     = [0 50 100 200 350 500 700 900 1100];   % [m] Altitude vector
settings.wind.inputMult    = [1 2 3 3.5 4 4.5 5 5.5 6];                  % [-] Percentage of increasing magnitude at each altitude
settings.wind.inputAzimut  = 45*pi/180*rand(1,9);%[0 0 0 0 0 0 0 0 0];         % [deg] Wind azimut angle at each altitude (toward wind incoming direction)

settings.wind.input_uncertainty = [2, 2];
% settings.wind.input_uncertainty = [a,b];      wind uncertanties:
% - a, wind magnitude percentage uncertanty: magn = magn *(1 +- a)
% - b, wind direction band uncertanty: dir = dir 1 +- b

%%%%% Random wind model
% if both the model above are false

% Wind is generated randomly from the minimum to the maximum parameters which defines the wind.
% Setting the same values for min and max will fix the parameters of the wind.
settings.wind.MagMin    =   0;                        % [m/s] Minimum Magnitude
settings.wind.MagMax    =   0;                        % [m/s] Maximum Magnitude
settings.wind.ElMin     =   0*pi/180;                 % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax     =   0*pi/180;                 % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin     =  (180)*pi/180;              % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax     =  (180)*pi/180;              % [rad] Maximum Azimuth, user input in degrees (ex. 90)

% NOTE: wind azimuth angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

%% PLOT DETAILS
settings.plots   =   true;

%% MONTECARLO 
settings.montecarlo = false;                                                % set to true to run and save montecarlo simulation plots



