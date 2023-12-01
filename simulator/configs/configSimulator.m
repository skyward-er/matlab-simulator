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

%% 2) SIMULATION SETTINGS

configFlags;


%% 3) LOAD DATAPATH
configPath;

%% 4) TRAJECTORY GENERATION SETUP
if conf.script == "trajectory generation"
    config_TrajectoryGen;
end
%% 5) LAUNCH SETUP
configLaunchRail;

%% 6) WIND DETAILS
configWind;

%% 8) PLOTS?
configPlots;

%% 9) MONTECARLO?
configMontecarlo;

%% 10) CONTROL
configControl;

%% 11) CONTROL PARAMETERS
configControlParams;

%% 12) SENSOR FAULT GENERATION
configFaults;

%% 13) PARAFOIL DATA
configPayload;

if conf.script ~= "trajectory generation"
    %% 14) REFERENCES
    configReferences;
    %% 15) SPECIAL CONDITIONS?
%     config_SpecialConditions;
%% CONFIG SFD_HR
config_sfd_hr

end




