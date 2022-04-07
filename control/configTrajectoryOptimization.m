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
% settings.mission = 'Lynx_Portugal_October_2021';
settings.mission = 'Pyxis_Portugal_October_2022';
% settings.mission = 'Pyxis_Roccaraso_September_2022';

%% LOAD DATA
simulationsDataPath = strcat('')
run()

%% AEROBRAKES EXTENSION DISCRETIZATION
dX = 0.001;          % [m] 



