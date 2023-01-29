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


clearvars; close all; clc;

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

% Config path
addpath('../simulator/configs/');

%% CHECK IF MSA-TOOLKIT IS UPDATED
msaToolkitURL = 'https://github.com/skyward-er/msa-toolkit';
localRepoPath = '../data/msa-toolkit';
% status = checkLastCommit(msaToolkitURL, localRepoPath, pwd);
% submoduleAdvice(status, msaToolkitURL, localRepoPath, pwd);

% %% LOAD DATA
conf.script = "trajectory generation"; % this defines which configuration script to run
config;

%% AIRBRAKES RADIAL EXTENSION
% Airbrakes extension vector
delta_alpha_values  = linspace(settings.servo.minAngle,settings.servo.maxAngle,2);
[deltaX_values] = extension_From_Angle(delta_alpha_values, settings);
% I exclude the limits for robustness
% deltaX_values = deltaX_values(2:end-1);

%% FINAL CONDITIONS
% Impose the final condition I want to reach.

Vz_final =  settings.Vz_final;
z_final  =  settings.z_final;
z_final_MTR  =  settings.z_final_MTR;
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
Y0 = [X0; V0; W0; Q0; settings.Ixx(end); settings.Iyy(end); settings.Izz(end)];

%%% wind initialization
[uw, vw, ww, ~] = windConstGenerator(settings.wind);
settings.constWind = [uw, vw, ww];

Z_initial = 0;

%% INTERPOLATED CA
coeffsCA = load(strcat(dataPath, '/CAinterpCoeffs.mat'));

%% NEEDED PARAMETERS

settingsSim.g0 = settings.g0;
settingsSim.z0 = settings.z0;
settingsSim.C  = settings.C;

%% COMPUTE THE TRAJECTORIES BY BACK INTEGRATION
Ntraj_ARB = length(deltaX_values);
N_mass = contSettings.N_mass;                            % number of different Mass values

deltaX = 0;
mass = linspace(settings.ms,settings.m0,N_mass);

% Pre-allocation
trajectories_ARB = cell(Ntraj_ARB, N_mass);
trajectories_saving_ARB = cell(Ntraj_ARB, N_mass);
trajectories_MTR = cell(N_mass,1);
trajectories_saving_MTR = cell(N_mass,1);

for j = 1:N_mass
  m = mass(j);
for index = 1:Ntraj_ARB

deltaX = deltaX_values(index);

% Start simulink simulation
z_final = settings.z_final;
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
trajectories{index,j}        = struct('t_ref', t_ref,  'Z_ref',  Z_ref, 'VZ_ref', VZ_ref,  'X_ref',  X_ref, 'VX_ref', VX_ref,  'Y_ref',  Y_ref, 'VY_ref', VY_ref);
trajectories_saving{index,j} = struct('Z_ref', Z_ref, 'VZ_ref', VZ_ref,  'X_ref',  X_ref, 'VX_ref', VX_ref,  'Y_ref',  Y_ref, 'VY_ref', VY_ref);

end

deltaX = 0;
z_final = z_final_MTR;
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
trajectories_MTR{j}        = struct('t_ref', t_ref,  'Z_ref',  Z_ref, 'VZ_ref', VZ_ref,  'X_ref',  X_ref, 'VX_ref', VX_ref,  'Y_ref',  Y_ref, 'VY_ref', VY_ref);
trajectories_saving_MTR{j} = struct('Z_ref', Z_ref, 'VZ_ref', VZ_ref,  'X_ref',  X_ref, 'VX_ref', VX_ref,  'Y_ref',  Y_ref, 'VY_ref', VY_ref);


end
%% SAVING
 settings.save = false;

if ~settings.save
    warning('save is set to false')
end

if settings.save
    save(strcat(ConDataPath, '/Trajectories.mat'), 'trajectories_saving')
end

%% PLOT
if settings.plots
    tg_plots
end

%% DELETE USELESS FILES
warning off
delete('Trajectory_generation.slxc')
rmdir('slprj', 's')
warning on

