clc; clearvars; close all;

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

%% Matlab-simulator configs

conf.script = "trajectory generation";
configSimulator;
altitudeTarget = settings.z_final;

clearvars -except contSettings altitudeTarget ConDataPath

extVect = [0 1];

%% DATA PREPARATION

%%% Mission
mission = Mission(true, 'changeMatlabPath', true);

%%% Rocket
rocket = Rocket(true);

% Change ABK with 
rocket.airbrakes.enabled = true;

% Set maximum extension (if enabled == true)
rocket.airbrakes.deltaTime = 0;
rocket.airbrakes.extension = 1;

%%% Environment
environment = Environment(true, rocket.pins(1) + rocket.parafoil.noseLength);

%%% Wind

%%% Null wind
wind = Wind(true);
wind.altitudes = [0 100];

wind.magnitudeDistribution = ["u", "u"];
wind.azimuthDistribution   = ["u", "u"];
wind.elevationDistribution = ["u", "u"];

wind.magnitudeParameters   = [0 0; 0 0];
wind.azimuthParameters     = [0 0; 0 0];
wind.elevationParameters   = [0 0; 0 0];

rocketRef = rocket;

%% CONFIG

N_ext = length(extVect);
massVect = contSettings.masses_vec;
N_mass = length(massVect);

%% Define initial condition stdAscent

settings = Settings('ode');
settings.addprop('simulator');
settings.simulator.parachute = false;

% Hp: Descent won't take more than 1000s
tf = 1000;
t0 = 0;

options = settings.ode.optDescent;

%% Generate trajectories

clc;

figure; hold on; grid on;
ylim([0 400]); xlim([0 3e3])

Z = cell(N_mass, N_ext);
Vz = cell(N_mass, N_ext);

Y0 = [0 0 -altitudeTarget -12 -4 0]';

tSpan = linspace(tf, t0, 200);

trajectories_saving = cell(N_ext, N_mass);

for ii = 1:N_mass
    for jj = 1:N_ext
        rocket = rocketRef;
        delta = rocket.cutoffMass - massVect(ii);
        rocket.mass = rocket.mass - delta;
        rocket.airbrakes.extension = extVect(jj);

        bk_sol = ode113(@ballistic3DOF, tSpan, Y0, options, ...
            rocket, environment, wind);

        Z_ref = 0:10:altitudeTarget;
        vNED = interp1(-bk_sol.y(3,:), bk_sol.y(4:6, :)', Z_ref);
        plot(Z_ref, -vNED(:,3), 'DisplayName', strcat(num2str(massVect(ii)), "kg, ", num2str(extVect(jj)*100), "\%"));
        drawnow

        trajectories_saving{jj,ii} = struct('Z_ref', Z_ref, 'VZ_ref', -vNED(:,3),  'X_ref',  bk_sol.y(1,:), 'VX_ref', vNED(:,2),  'Y_ref',  bk_sol.y(2,:), 'VY_ref', vNED(:,2));
    end
end
save(strcat(ConDataPath, '/Trajectories.mat'), 'trajectories_saving');

