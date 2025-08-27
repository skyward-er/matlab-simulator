%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

REVISIONS:
- #0 07/04/2022, Release, Davide Rosato, AFD Department

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}


%% 2) SIMULATION SETTINGS

configFlags;


%% Mission parameters
mission = Mission(true, 'changeMatlabPath', true);
rocket = Rocket(mission);
environment = Environment(mission);
wind = Wind(mission);

%% 3) LOAD DATAPATH
configPath;

%% CONFIG CONTROL
run(strcat('config', mission.name));
if conf.HIL
    run('HILconfig.m');
end
%% 4) TRAJECTORY GENERATION SETUP
if conf.script == "trajectory generation"
    config_TrajectoryGen;
end

%% 8) PLOTS?
configPlots;

%% 9) CONTROL
configControl;

%% 10) CONTROL PARAMETERS
configControlParams;

%% 11) SENSOR FAULT GENERATION
configFaults;

%% 12) PARAFOIL DATA
configPayload;

if conf.script ~= "trajectory generation"
    %% 13) REFERENCES
    configReferences;
    %% 14) SPECIAL CONDITIONS?
    %     config_SpecialConditions;
end




