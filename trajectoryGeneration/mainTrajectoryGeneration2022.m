%{

mainTrajectoryGeneration - this is the main script; it generates the trajectories that have been chosen in config.m

CALLED SCRIPTS: configTrajectoryGeneration, simulationsData, plots

CALLED FUNCTIONS: Trajectory_generation (SIMULINK model), getDrag, getRho,
getMach

REVISIONS:
- 0     16/04/2021, Release,    Alessandro Del Duca
- 1     07/04/2022, update      Davide Rosato, AFD Department
                    Compatibility with common functions folder

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

% clearvars -except ZTARGET_CYCLE;
clear all; close all; clc;

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

%% CHECK IF MSA-TOOLKIT IS UPDATED
msaToolkitURL = 'https://github.com/skyward-er/msa-toolkit';
localRepoPath = '../data/msa-toolkit';
status = checkLastCommit(msaToolkitURL, localRepoPath, pwd);
submoduleAdvice(status, msaToolkitURL, localRepoPath, pwd);

% %% LOAD DATA
configTrajectoryGeneration;
matlab_graphics;
%% AIRBRAKES RADIAL EXTENSION
% Airbrakes extension vector
% deltaX_values = linspace(settings.hprot(1), settings.hprot(end), settings.Ndx+2);
delta_alpha_values  = linspace(settings.servo.minAngle,settings.servo.maxAngle,11);
[deltaX_values] = extension_From_Angle(delta_alpha_values, settings);
% I exclude the limits for robustness
% deltaX_values = deltaX_values(2:end-1);

%% FINAL CONDITIONS
% Impose the final condition I want to reach.

Vz_final =  settings.Vz_final;
z_final  =  settings.z_final;
Vx_final =  settings.Vx_final;   
x_final  =  settings.x_final;  
Vy_final =  settings.Vy_final;  
y_final  =  settings.y_final;  

%% INITIAL VELOCITY
% Calculate vertical velocity at which the back propagation must end
settings.OMEGA = settings.OMEGA;
settings.PHI = settings.PHI;

%%% Attitude
Q0 = angleToQuat(settings.PHI, settings.OMEGA, 0*pi/180)';

%%% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

%%% composing initial conditions for ode
Y0 = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];

%%% wind initialization
[uw, vw, ww, ~] = windConstGenerator(settings.wind);
settings.constWind = [uw, vw, ww];

%%% running the preliminar simulation that stops when air brakes open
[T, Y] = ode113(@ascent, [0, 60], Y0, settings.ode.optionsascTrajGen, settings);
vels = quatrotate(quatconj(Y(end, 10:13)),Y(end, 4:6));

Vz_initial = -vels(3);
Z_initial = 0;
% Increasing the value
%Vz_initial = Vz_initial * (1 + settings.Vz_initialPerc);

%% INTERPOLATED CA
coeffsCA = load(strcat(dataPath, '/CAinterpCoeffs.mat'));

%% NEEDED PARAMETERS
settingsSim.ms = settings.ms;
settingsSim.g0 = settings.g0;
settingsSim.z0 = settings.z0;
settingsSim.C  = settings.C;

%% COMPUTE THE TRAJECTORIES BY BACK INTEGRATION
Ntraj = length(deltaX_values);
deltaX = 0;

% Pre-allocation
trajectories = cell(Ntraj, 1);
trajectories_saving = cell(Ntraj, 1);

for index = 1:Ntraj

deltaX = deltaX_values(index);

% Start simulink simulation
generation = sim('Trajectory_generation');

% Get the output of the simulation
t_ref   = flip(30 - generation.tout); % (30seconds - time) In this way a plot the trajectories in a clearer way
Z_ref   = flip(generation.z_simul); 
VZ_ref  = flip(generation.Vz_simul);
X_ref   = flip(generation.x_simul); 
VX_ref  = flip(generation.Vx_simul);
Y_ref   = flip(generation.y_simul); 
VY_ref  = flip(generation.Vy_simul);
cd      = flip(generation.cd);

% Save the trajectories in a struct. Easier to plot
trajectories{index}        = struct('t_ref', t_ref,  'Z_ref',  Z_ref, 'VZ_ref', VZ_ref,  'X_ref',  X_ref, 'VX_ref', VX_ref,  'Y_ref',  Y_ref, 'VY_ref', VY_ref);
trajectories_saving{index} = struct('Z_ref', Z_ref, 'VZ_ref', VZ_ref,  'X_ref',  X_ref, 'VX_ref', VX_ref,  'Y_ref',  Y_ref, 'VY_ref', VY_ref);

end

%% SAVING

if ~settings.save
    warning('save is set to false')
end

% if settings.save
%     save(strcat(ConDataPath, '/Trajectories.mat'), 'trajectories_saving')
% end

%% PLOT
if settings.plots
    plots
end

%% DELETE USELESS FILES
warning off
delete('Trajectory_generation.slxc')
rmdir('slprj', 's')
warning on

