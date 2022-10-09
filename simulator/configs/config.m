%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

REVISIONS:
- #0 07/04/2022, Release, Davide Rosato, AFD Department

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

%% 1) MISSION FILE
configMission;

%% 2) LOAD DATAPATH
configPath;

%% 3) SIMULATION SETTINGS
configFlags;

%% 4) LAUNCH SETUP
configLaunchRail;

%% 5) WIND DETAILS
configWind;

%% 7) PLOTS?
configPlots;

%% 8) MONTECARLO?
configMontecarlo;

%% 9) CONTROL
configControl;

%% 10) CONTROL PARAMETERS
configControlParams;

%% 11) REFERENCES
configReferences;

%% 12) DATA EXTRACTION
configSettings;