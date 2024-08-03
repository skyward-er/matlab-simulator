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


if ~exist('flagSubmodulesUpdated','var') % every first time you use the simulator checks for updates, then stops doing it (note: if you clear all vars it starts doing it)
    close all; clear; clc;
else
    close all; clc;
    clearvars -except flagSubmodulesUpdated
end

restoredefaultpath;
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

% add MSA data path
commonPath = strcat('../common');
addpath(genpath(commonPath));

%% CHECK IF MSA-TOOLKIT IS UPDATED
% msaToolkitURL = 'https://github.com/skyward-er/msa-toolkit';
% localRepoPath = '../data/msa-toolkit';
% status = checkLastCommit(msaToolkitURL, localRepoPath, pwd);
% submoduleAdvice(status, msaToolkitURL, localRepoPath, pwd);

% %% LOAD DATA
conf.script = "trajectory generation"; % this defines which configuration script to run
settings.montecarlo = false;
configSimulator;

%% AIRBRAKES RADIAL EXTENSION
% Airbrakes extension vector
delta_alpha_values  = linspace(settings.servo.minAngle,settings.servo.maxAngle,2);
[deltaX_values] = extension_From_Angle(delta_alpha_values, settings, mission);
% I exclude the limits for robustness
% deltaX_values = deltaX_values(2:end-1);

%% FINAL CONDITIONS
% Impose the final condition I want to reach.

Vz_final =  settings.Vz_final;
z_final  =  settings.z_final;
Vx_final = 17; % settings.Vx_final; % the value 17 comes from a previous montecarlo simulation
x_final  =  settings.x_final;
Vy_final = 17; %  settings.Vy_final;
y_final  =  settings.y_final;

%% INITIAL VELOCITY

%%% Attitude
Q0 = angleToQuat(environment.phi, environment.omega, 0*pi/180)'; % set initial quaternion to ramp angles

%%% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

%%% wind initialization
wind = WindCustom(mission);
[uw, vw, ww] = wind.getVels(0);
settings.constWind = [uw, vw, ww];

Z_initial = 0;

%% INTERPOLATED CA
coeffsCA = load(strcat(commonPath, '/missions/', mission.name, '/data/CAinterpCoeffs.mat'));

%% NEEDED PARAMETERS

settingsSim.g0 = environment.g0;
settingsSim.z0 = environment.z0;
settingsSim.C  = rocket.diameter;
settingsSim.CD_correction_ref = settings.CD_correction_ref;

%% COMPUTE THE TRAJECTORIES BY BACK INTEGRATION
Ntraj_ARB = length(deltaX_values);
mass = contSettings.masses_vec;                     % set in config control params
N_mass = length(mass);                              % number of different Mass values

% initialise air brakes extension
deltaX = 0;


% Pre-allocation
trajectories_ARB = cell(Ntraj_ARB, N_mass);
trajectories_saving_ARB = cell(Ntraj_ARB, N_mass);
trajectories_MTR = cell(N_mass,1);
trajectories_saving_MTR = cell(N_mass,1);

for j = 1:N_mass
    m = mass(j);
    for index = 1:Ntraj_ARB
        %%% composing initial conditions for ode
        % I_index = 1+floor(length(settings.Ixx)/(N_mass-1))*(j-1)
        Y0 = [X0; V0; W0; Q0];
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

end
%% SAVING
% settings.save = false;
% load test
if ~settings.save
    warning('save is set to false')
end

if settings.save
    save(strcat(ConDataPath, '/Trajectories.mat'), 'trajectories_saving')
end

if settings.CD_correction_ref == 1
    save traj0perc.mat trajectories_saving
elseif settings.CD_correction_ref == 0.75
    save traj-25perc.mat trajectories_saving
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

%% remove this return if you want to set hybrid references
return

%% hybrid references
if settings.CD_correction_ref == 1
    datcom_0_closed = trajectories_saving{1,end};
    datcom_0_open = trajectories_saving{2,end};
    save plot0 datcom_0_closed datcom_0_open
elseif settings.CD_correction_ref == 0.75
    datcom_25_closed = trajectories_saving{1,end};
    datcom_25_open = trajectories_saving{2,end};
    save plot25 datcom_25_closed datcom_25_open
end
%%
clearvars; close all; clc;

load plot0
load plot25

figure('Position',[100,100,600,600])
hold on;
grid on;
plot(datcom_0_open.Z_ref,datcom_0_open.VZ_ref,'k','DisplayName','datcom')
plot(datcom_0_closed.Z_ref,datcom_0_closed.VZ_ref,'k',"HandleVisibility","off")

plot(datcom_25_open.Z_ref,datcom_25_open.VZ_ref,'b','DisplayName','datcom-25')
plot(datcom_25_closed.Z_ref,datcom_25_closed.VZ_ref,'b',"HandleVisibility","off")
legend



