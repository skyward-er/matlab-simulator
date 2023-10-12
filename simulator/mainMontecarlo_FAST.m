%{

script that runs the Montecarlo simulations to validate the control
algorithms. Same as main, but some parameters are cicled.
CALLED FUNCTIONS: 

0 -     Marco Marchesi, SCS department, marco.marchesi@skywarder.eu
        Giuseppe Brentino, SCS department, giuseppe.brentino@skywarder.eu
        Latest control algorithms

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
tic
%% recall the first part of the MAIN script
% adds folders to the path and retrieves rocket, mission, simulation, etc
% data.

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
% msaToolkitURL = 'https://github.com/skyward-er/msa-toolkit';
% localRepoPath = '../data/msa-toolkit';
% status = checkLastCommit(msaToolkitURL, localRepoPath, pwd);
% % submoduleAdvice(status, msaToolkitURL, localRepoPath, pwd);

%% CONFIGs
conf.script = "simulator";
settings.montecarlo = true;
configSimulator;


%% MONTECARLO SETTINGS
rng default
matlab_graphics;

%% MONTECARLO ANALYSIS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONFIG%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% other parameters you want to set for the particular simulation:

clearvars   msaToolkitURL Itot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONFIG%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

settings_mont_init = struct('x',[]);

%% start simulation

contSettings.algorithm = 'complete';

%save arrays
save_thrust = cell(size(stoch.thrust,1),1);
apogee.altitude = [];
wind_Mag = zeros(N_sim,1);
wind_el = zeros(N_sim,1);
wind_az = zeros(N_sim,1);
t_shutdown.value = zeros(N_sim,1);

parfor i = 1:N_sim
    settings_mont = settings_mont_init;

    settings_mont.motor.expThrust = stoch.thrust(i,:);                      % initialize the thrust vector of the current simulation (parfor purposes)
    settings_mont.motor.expTime = stoch.expThrust(i,:);                     % initialize the time vector for thrust of the current simulation (parfor purposes)
    settings_mont.tb = max( stoch.expThrust(i,stoch.expThrust(i,:)<=settings.tb) );     % initialize the burning time of the current simulation (parfor purposes)
    settings_mont.State.xcgTime = stoch.State.xcgTime(:,i);                 % initialize the baricenter position time vector
    settings_mont.mass_offset = stoch.mass_offset(i);
    settings_mont.OMEGA = stoch.OMEGA_rail(i);
    settings_mont.PHI = stoch.PHI_rail(i);

    % Define coeffs matrix for the i-th simulation
    settings_mont.Coeffs = settings.Coeffs* (1+stoch.aer_percentage(i));


    % set the wind parameters
    switch settings.windModel
        case "constant"
            settings_mont.wind.uw = stoch.wind.uw(i);
            settings_mont.wind.vw = stoch.wind.vw(i);
            settings_mont.wind.ww = stoch.wind.ww(i);
            settings_mont.wind.Az = stoch.wind.Az(i);
            settings_mont.wind.El = stoch.wind.El(i);
        case "multiplicative"
            settings_mont.wind.Mag = stoch.wind.Mag(i);
            settings_mont.wind.Az = stoch.wind.Az(i,:);
    end

    if displayIter == true
        fprintf("simulation = " + num2str(i) + " of " + num2str(N_sim) + ", algorithm: " + contSettings.algorithm +", scenario: "+ settings.scenario +"\n");
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [simOutput] = std_run_FAST(settings,contSettings,settings_mont);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    save_thrust{i} = simOutput;

end

%% RETRIEVE INTERESING PARAMETERS:

N_ApogeeWithinTarget_10 = 0;
N_ApogeeWithinTarget_50 = 0;
% N_landings_within50m = 0;
% N_landings_within150m = 0;

for i = 1:N_sim

    % apogee
    apogee.altitude(i) = -save_thrust{i}.apogee.position(3);

    % radius of apogee (horizontal) from the initial point
    apogee.radius(i) = save_thrust{i}.apogee.radius;

    % horizontal speed at apogee
    idx_apo = save_thrust{i}.apogee.idx;
    apogee.horizontalSpeed(i) = norm(save_thrust{i}.apogee.velocity_ned(1:2)); % this is in body frame, but as the last point is the apogee we should have only  horizontal velocity, so all the components must be taken
    apogee.horizontalSpeedX(i) = norm(save_thrust{i}.apogee.velocity_ned(1));
    apogee.horizontalSpeedY(i) = norm(save_thrust{i}.apogee.velocity_ned(2));

    % time of engine shutdown
    t_shutdown.value(i) = save_thrust{i}.sensors.mea.t_shutdown;

    if ~settings.wind.model && ~settings.wind.input
        % wind magnitude
        wind_Mag(i) = save_thrust{i}.wind.Mag;
        % wind azimuth
        wind_az(i) = save_thrust{i}.wind.Az;
        %wind elevation
        wind_el(i) = save_thrust{i}.wind.El;
    end
    %reached apogee time
    apogee.times(i) = save_thrust{i}.apogee.time;
    apogee.prediction(i) = save_thrust{i}.sensors.mea.prediction(end);
    apogee.prediction_last_time(i) = save_thrust{i}.sensors.mea.time(end); % checkare

    % save apogees within +-10 meters from target:
    if abs(apogee.altitude(i) - settings.z_final)<=10
        N_ApogeeWithinTarget_10 = N_ApogeeWithinTarget_10 +1;
    end
    % save apogees within +-50 meters from target:
    if abs(apogee.altitude(i) - settings.z_final)<=50
        N_ApogeeWithinTarget_50 = N_ApogeeWithinTarget_50 +1;
    end

end

% merit parameters
apogee.altitude_mean = mean(apogee.altitude);
apogee.altitude_std = std(apogee.altitude);

% save t_shutdown
t_shutdown.mean = mean(t_shutdown.value);
t_shutdown.std = std(t_shutdown.value);

% save apogee horizontal position
apogee.radius_mean = mean(apogee.radius);
apogee.radius_std = std(apogee.radius);
apogee.radius_max = max(apogee.radius);
apogee.radius_min = min(apogee.radius);

% save apogee speed
apogee.horizontalSpeed_mean = mean(apogee.horizontalSpeed);
apogee.horizontalSpeed_std = std(apogee.horizontalSpeed);
apogee.horizontalSpeed_max = max(apogee.horizontalSpeed);
apogee.horizontalSpeed_min = min(apogee.horizontalSpeed);

% accuracy
apogee.accuracy_10 = N_ApogeeWithinTarget_10/N_sim*100; % percentage, so*100
apogee.accuracy_50 = N_ApogeeWithinTarget_50/N_sim*100; % percentage, so*100

%% compute how many simulations open air brakes
on = 0;
on_max = 0;
off = 0;
for i = 1:N_sim       
    if max(save_thrust{i}.ARB.cmdPosition) > 0
        on = on+1;
        if max(save_thrust{i}.ARB.cmdPosition) >= settings.servo.maxAngle-1e-3
            on_max = on_max + 1;
        end
    else
        off = off+1;
    end
end

meritParam = zeros(N_sim,1);
for i = 1:N_sim
    totalABKTime = save_thrust{i}.ARB.cmdTime(find(save_thrust{i}.ARB.cmdPosition>0,1,'last'))-save_thrust{i}.ARB.cmdTime(find(save_thrust{i}.ARB.cmdPosition>0,1,'first'));
    totalABKPosition = mean(save_thrust{i}.ARB.cmdPosition)/settings.servo.maxAngle;
    if ~isnan(totalABKPosition) && ~isempty(totalABKTime)
        meritParam(i) = totalABKPosition*totalABKTime/(max(save_thrust{i}.t)-contSettings.ABK_shadowmode);
    else 
        meritParam(i) = 0;
    end
end

%% PLOTS

plotsMontecarlo;

%% print
fprintf('\n')
fprintf('MIN shutdown time = %.3f\n',min(t_shutdown.value))
fprintf('MAX shutdown time = %.3f\n',max(t_shutdown.value))
fprintf('MEAN shutdown time = %.3f\n\n',t_shutdown.mean)
fprintf('%% simulations abk open = %.2f%%\n',100*on/N_sim)
fprintf('%% simulations merit param above 0.1= %.2f%%\n\n',100*sum(meritParam>0.1)/N_sim)
fprintf('Computation time %.3f\n', toc)


%% Save results.txt
folder = "Preflight_results";
if ~exist(folder,"dir")
    mkdir(folder)
end

fid = fopen( folder+"/Results_preflight.txt", 'wt' );  % CAMBIA IL NOME
%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(fid,'SIMULATION \n\n');
fprintf(fid,'Number of simulations: %d \n \n',N_sim); % Cambia n_sim
fprintf(fid,'Target apogee: %d \n',settings.z_final);
if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil
    fprintf(fid,'Target landing: %d \n',settings.payload.target);
end
fprintf(fid,'Total impulse +-5%% at 3 sigma \n');
fprintf(fid,'CA: %.2f simulation, %.2f reference \n\n\n',settings.CD_correction, settings.CD_correction_ref);

%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(fid,'AIR BRAKES \n\n');
fprintf(fid,'Algorithm: %s \n',contSettings.algorithm );
fprintf(fid,'Engine shut-down control frequency: %d Hz \n',settings.frequencies.controlFrequency);
fprintf(fid,'Airbrakes control frequency: %d Hz \n',settings.frequencies.arbFrequency );
fprintf(fid,'Initial Mach number at which the control algorithm starts: %.3f \n\n',settings.MachControl);
fprintf(fid,'Other parameters specific of the simulation: \n');
fprintf(fid,'Filter coefficient: %.3f \n', contSettings.filter_coeff);
fprintf(fid,'Target for shutdown: %d \n',settings.mea.z_shutdown);
if contSettings.algorithm == "interp" || contSettings.algorithm == "complete"
    fprintf(fid,'N_forward: %d \n', contSettings.N_forward);
    fprintf(fid,'Delta Z (reference): %d \n',contSettings.reference.deltaZ);
    fprintf(fid,'Filter linear decrease starting at: %d m\n', contSettings.filterMinAltitude);
    fprintf(fid,'Filter initial value: %d \n', contSettings.filter_coeff0);
    fprintf(fid,'Interpolation type: %s \n', contSettings.interpType);
    fprintf(fid,'Correction with current pitch angle: %s \n\n\n',contSettings.flagCorrectWithPitch);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(fid,'WIND \n\n ');
fprintf(fid,'Wind model: %s \n',settings.windModel);
if settings.windModel == "constant"
    fprintf(fid,'Wind model parameters: \n'); % inserisci tutti i parametri del vento
    fprintf(fid,'Wind Magnitude: 0-%d m/s\n',settings.wind.MagMax);
    fprintf(fid,'Wind minimum azimuth: %d [°] \n',rad2deg(settings.wind.AzMin));
    fprintf(fid,'Wind maximum azimuth: %d [°] \n',rad2deg(settings.wind.AzMax));
    fprintf(fid,'Wind minimum elevation: %d [°] \n', rad2deg(settings.wind.ElMin));
    fprintf(fid,'Wind maximum elevation: %d [°] \n\n\n',rad2deg(settings.wind.ElMax));
elseif settings.windModel == "multiplicative"
    fprintf(fid,'Wind model parameters: \n'); % inserisci tutti i parametri del vento
    fprintf(fid,'Ground wind Magnitude: 0-%d m/s\n\n\n',settings.wind.MagMax);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil
    fprintf(fid,'PARAFOIL \n\n');
    fprintf(fid,'Guidance approach %s \n',contSettings.payload.guidance_alg);
    fprintf(fid,'PID proportional gain %s \n',contSettings.payload.Kp);
    fprintf(fid,'PID integral gain %s \n',contSettings.payload.Ki);
    fprintf(fid,'Opening altitude %s \n',settings.para(1).z_cut);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(fid,'MASS: \n\n');
fprintf(fid,'Interval : +-%d at 3sigma \n',3*sigma_m );

%%%%%%%%%%%%%%%%%%%%%%%%%%%
if settings.scenario ~= "descent"
    fprintf(fid,'RESULTS APOGEE\n\n');
    fprintf(fid,'Max apogee: %.2f m\n',max(apogee.altitude));
    fprintf(fid,'Min apogee: %.2f m\n',min(apogee.altitude));
    fprintf(fid,'Mean apogee: %.2f m\n',apogee.altitude_mean);
    fprintf(fid,'Apogee standard deviation 3sigma: %.4f \n',3*apogee.altitude_std);
    fprintf(fid,'Apogees horizontal distance from origin mean : %.2f [m] \n',apogee.radius_mean);
    fprintf(fid,'Apogees horizontal distance from origin std : %.2f [m] \n\n',apogee.radius_std);
    fprintf(fid,'Apogees horizontal speed mean : %.2f [m/s] \n',apogee.horizontalSpeed_mean);
    fprintf(fid,'Apogees horizontal speed std : %.2f [m/s] \n\n',apogee.horizontalSpeed_std);
    fprintf(fid,'Mean shutdown time : %.3f [m/s] \n',t_shutdown.mean);
    fprintf(fid,'Standard deviation of shutdown time : %.3f [m/s] \n\n\n',t_shutdown.std);
end

fclose(fid);

%% save file
% saveas(gca,azwind+config"om"+num2str(settings.OMEGA))


