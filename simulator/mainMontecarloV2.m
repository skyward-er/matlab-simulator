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

clc; clearvars; close all;

%% path loading
restoredefaultpath;
filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end
addpath(genpath(currentPath));

% add common submodule path
commonPath = strcat('../common');
addpath(genpath(commonPath));

%% CONFIGs
conf.script = "simulator";
settings.montecarlo = true;
configSimulator;

% Load Montecarlo specific configs
configMontecarlo;

%% MONTECARLO SETTINGS
rng default
matlab_graphics;

%% check on the simulation profile:
go = '';
while ~strcmp(go,'yes') && ~strcmp(go,'y') && ~strcmp(go,'no') && ~strcmp(go,'n')
    go = input('Did you check the profile of the simulation? \nA.K.A. did you check that: settings.scenario = "controlled ascent" or " "descent" or whatever? (y/n) \n','s');
    if go == "yes" || go == "y"
        fprintf('All right, let''s go \n\n')
    elseif go == "no" || go == "n"
        fprintf('Set it then. Simulation is stopped. \n\n')
        return
    else
        fprintf('You typed something different from the options, retry. \n\n')
    end
end

%% Save offline simulation
flagSaveOffline = input('Do you want to save the results offline? (y/n): ','s');

%% Run simulations

N_sim = sensitivity.n;
displayIter = sensitivity.displayIter;
scenario = settings.scenario;

settings_mont_init = struct();

% Pre allocate save data
save_thrust = cell(N_sim,1);
wind_Mag = zeros(N_sim,1);
wind_el = zeros(N_sim,1);
wind_az = zeros(N_sim,1);
t_shutdown.value = zeros(N_sim,1);

rocketRef = rocket;
envRef = environment;
windRef = wind;

parfor ii = 1:N_sim
    rocket = rocketRef;
    environment = envRef;
    wind = windRef;
    [rocket, environment, wind] = updateData(rocket, environment, wind, parameters, ii);

    settings_mont = settings_mont_init;
    settings_mont.ABK.PID_coeffs = control_sensitivity.ABK_curve(ii, :);
    settings_mont.ABK.PID_ref = control_sensitivity.ABK_ref(ii);
    settings_mont.NAS.mult = control_sensitivity.NAS_mult(ii);

    if displayIter == true
        fprintf("simulation = " + num2str(ii) + " of " + num2str(N_sim) + ", algorithm: " + contSettings.algorithm +", scenario: "+ scenario +"\n");
    end

    [simOutput] = std_run(settings,contSettings,rocket,environment,wind,mission,settings_mont);
    save_thrust{ii} = simOutput;
    save_thrust{ii}.ARB.K_vals = control_sensitivity.ABK_curve(ii,:);
    save_thrust{ii}.ARB.ref = control_sensitivity.ABK_ref(ii);
    save_thrust{ii}.NAS.mult = control_sensitivity.NAS_mult(ii);

end

%% Retrieve parameters

% Define save structures
apogee = struct();
landing = struct();

N_ApogeeWithinTarget_10 = 0;
N_ApogeeWithinTarget_50 = 0;
N_landings_within50m = 0;
N_landings_within150m = 0;

for ii = 1:N_sim
    % apogee
    apogee.altitude(ii) = -save_thrust{ii}.apogee.position(3);

    % radius of apogee (horizontal) from the initial point
    apogee.radius(ii) = save_thrust{ii}.apogee.radius;

    % NAS error
    nas.error(ii, :) = mean(abs(save_thrust{ii}.sensors.nas.error), 1);
    % horizontal speed at apogee
    idx_apo = save_thrust{ii}.apogee.idx;
    apogee.horizontalSpeed(ii) = norm(save_thrust{ii}.apogee.velocity_ned(1:2)); % this is in body frame, but as the last point is the apogee we should have only  horizontal velocity, so all the components must be taken
    apogee.horizontalSpeedX(ii) = norm(save_thrust{ii}.apogee.velocity_ned(1));
    apogee.horizontalSpeedY(ii) = norm(save_thrust{ii}.apogee.velocity_ned(2));

    % time of engine shutdown
    t_shutdown.value(ii) = save_thrust{ii}.sensors.mea.t_shutdown;

    %reached apogee time
    apogee.times(ii) = save_thrust{ii}.apogee.time;
    apogee.prediction(ii) = save_thrust{ii}.sensors.mea.prediction(end);
    apogee.prediction_last_time(ii) = save_thrust{ii}.sensors.mea.time(end); % checkare

    % save apogees within +-10 meters from target:
    if abs(apogee.altitude(ii) - settings.z_final)<=10
        N_ApogeeWithinTarget_10 = N_ApogeeWithinTarget_10 +1;
    end
    % save apogees within +-50 meters from target:
    if abs(apogee.altitude(ii) - settings.z_final)<=50
        N_ApogeeWithinTarget_50 = N_ApogeeWithinTarget_50 +1;
    end

    % landing
    if ~isnan(save_thrust{ii}.PRF.landing_position)
        landing.position(ii,:) = save_thrust{ii}.PRF.landing_position;
        landing.velocities_BODY(ii,:) = save_thrust{ii}.PRF.landing_velocities_BODY;
        landing.velocities_NED(ii,:) = save_thrust{ii}.PRF.landing_velocities_NED;
        landing.distance_to_target(ii) = norm(settings.payload.target(1:2)-landing.position(ii,1:2)');

        % save apogees within 50 meters (radius) from target:
        if landing.distance_to_target(ii) <= 50
            N_landings_within50m = N_landings_within50m +1; % save how many apogees sit in the +-50 m from target
        end
        % save apogees within 150 meters (radius) from target:
        if landing.distance_to_target(ii) <= 150
            N_landings_within150m = N_landings_within150m +1; % save how many apogees sit in the +-50 m from target
        end
    elseif conf.scenario == "ballistic"
        landing.position(ii,:) = save_thrust{ii}.Y(end,[2,1,3]);
        landing.velocities_BODY(ii,:) = save_thrust{ii}.Y(end,4:6);
        landing.velocities_NED(ii,:) = quatrotate(quatconj(save_thrust{ii}.Y(end,10:13)),save_thrust{ii}.Y(end,4:6));
        landing.distance_to_target(ii) = NaN;

        N_landings_within50m = NaN; % save how many apogees sit in the +-50 m from target
        N_landings_within150m = NaN; % save how many apogees sit in the +-50 m from target
    else
        landing.position(ii,:) = [NaN NaN NaN];
        landing.velocities_BODY(ii,:) = [NaN NaN NaN];
        landing.velocities_NED(ii,:) = [NaN NaN NaN];
        landing.distance_to_target(ii) = NaN;

        N_landings_within50m = NaN; % save how many apogees sit in the +-50 m from target
        N_landings_within150m = NaN; % save how many apogees sit in the +-50 m from target
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

% save landing speed
if ~isnan(landing.velocities_NED(1,3))
    landing.distance_mean = mean(landing.distance_to_target);
    landing.distance_std = std(landing.distance_to_target);

    landing.verticalSpeed_mean = mean(landing.velocities_NED(:,3));
    landing.verticalSpeed_std = std(landing.velocities_NED(:,3));
    landing.verticalSpeed_max = max(landing.velocities_NED(:,3));
    landing.verticalSpeed_min = min(landing.velocities_NED(:,3));

    landing.horizontalSpeed_mean = mean(norm(landing.velocities_NED(:,1:2)));
    landing.horizontalSpeed_std = std(norm(landing.velocities_NED(:,1:2)));
    landing.horizontalSpeed_max = max(norm(landing.velocities_NED(:,1:2)));
    landing.horizontalSpeed_min = min(norm(landing.velocities_NED(:,1:2)));

    landing.accuracy_50 = N_landings_within50m/N_sim*100;
    landing.accuracy_150 = N_landings_within150m/N_sim*100;
else
    landing.distance_mean = NaN;
    landing.distance_std = NaN;

    landing.verticalSpeed_mean = NaN;
    landing.verticalSpeed_std = NaN;
    landing.verticalSpeed_max = NaN;
    landing.verticalSpeed_min = NaN;

    landing.horizontalSpeed_mean = NaN;
    landing.horizontalSpeed_std = NaN;
    landing.horizontalSpeed_max = NaN;
    landing.horizontalSpeed_min = NaN;

    landing.accuracy_50 = NaN;
    landing.accuracy_150 = NaN;
end

 % montecarlo parameters
 apogee_mu = zeros(N_sim,1);
 apogee_sigma = zeros(N_sim,1);
 landing_mu = zeros(N_sim,1);
 landing_sigma = zeros(N_sim,1);

 for ii = 1:N_sim
     apogee_mu(ii) = mean(apogee.altitude(1:ii));
     apogee_sigma(ii) = std(apogee.altitude(1:ii));

     landing_mu(ii) = mean(landing.distance_to_target(1:ii));
     landing_sigma(ii) = std(landing.distance_to_target(1:ii));
 end
 % gaussian 10m
 p_10 = normcdf([settings.z_final-10, settings.z_final+10],apogee.altitude_mean,apogee.altitude_std);
 apogee.accuracy_gaussian_10 =( p_10(2) - p_10(1) )*100;
 % gaussian 50m
 p_50 = normcdf([settings.z_final-50, settings.z_final+50],apogee.altitude_mean,apogee.altitude_std);
 apogee.accuracy_gaussian_50 =( p_50(2) - p_50(1) )*100;

 %% Plots

 if sensitivity.plots.enabled
     plotsMontecarlo;
 end
 if sensitivity.plots.geoPlots
    disp("MANCANO I GEOPLOTS")
 end

 %% Save data

 % save plots
 saveDate = replace(string(datetime('now','TimeZone','local')),":","_");
 saveDate = replace(saveDate," ","__");
 saveDate = replace(saveDate,"-","_");
 folder = [];

 if flagSaveOffline == "yes" || flagSaveOffline == "y"
    folder = "MontecarloResults\"+ mission.name + "\" + contSettings.algorithm + "\" + num2str(N_sim) + "sim_" + saveDate;
 
    if ~exist(folder,"dir")
        mkdir(folder)
    end
    save(folder + "\montecarloFigures",'montFigures');
    save(folder + "\saveThrust.mat", "save_thrust", "apogee", "landing", "N_sim", "settings", "parameters") % add "save_thrust", > 2GB for 1000 sim

    % Save results.txt
    fid = fopen(folder+"\"+"Results_"+ num2str(N_sim) + "Sim.txt", 'wt' );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf(fid,'SIMULATION \n\n');
    fprintf(fid,'Number of simulations: %d \n \n',N_sim); % Cambia n_sim
    fprintf(fid,'Target apogee: %d \n',settings.z_final);
    if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil
        fprintf(fid,'Target landing: %d \n',settings.payload.target);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    fprintf(fid, 'ALGORITHMS\n\n');
    fprintf(fid,'Engine shut-down control frequency: %d Hz \n',settings.frequencies.controlFrequency);
    fprintf(fid,'Airbrakes control frequency: %d Hz \n',settings.frequencies.arbFrequency );
    fprintf(fid,'Maximum Mach number below which the airbrakes control algorithm starts: %.3f \n\n',rocketRef.airbrakes.maxMach);
    fprintf(fid,'Filter coefficient: %.3f \n', contSettings.filter_coeff);
    fprintf(fid,'Target for engine shutdown: %d \n',settings.mea.z_shutdown);
    if contSettings.algorithm == "interp_PID" || contSettings.algorithm == "complete"
        fprintf(fid,'N_forward: %d \n', contSettings.N_forward);
        fprintf(fid,'Delta Z (reference): %d \n',contSettings.reference.deltaZ);
        fprintf(fid,'Filter linear decrease starting at: %d m\n', contSettings.filterMinAltitude);
        fprintf(fid,'Filter initial value: %d \n', contSettings.filter_coeff0);
        fprintf(fid,'PID coefficients: [%d, %d, %d]\n', contSettings.ABK.PID_coeffs);
        fprintf(fid,'PID reference trajectory: %d\n', contSettings.ABK.PID_ref);
    end

    fprintf(fid,'\n\n');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil
        fprintf(fid,'PARAFOIL \n\n');
        fprintf(fid,'Guidance approach %s \n',contSettings.payload.guidance_alg);
        fprintf(fid,'PID proportional gain %s \n',rocketRef.parachutes(2,2).controlParams.Kp);
        fprintf(fid,'PID integral gain %s \n',rocketRef.parachutes(2,2).controlParams.Ki);
        fprintf(fid,'Opening altitude %s \n', num2str(settings.ada.para.z_cut));
    end

    fprintf(fid,'\n\n');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    if settings.scenario ~= "descent"
        fprintf(fid,'RESULTS APOGEE\n\n');
        fprintf(fid,'Max apogee: %.2f m\n',max(apogee.altitude));
        fprintf(fid,'Min apogee: %.2f m\n',min(apogee.altitude));
        fprintf(fid,'Mean apogee: %.2f m\n',apogee.altitude_mean);
        fprintf(fid,'Apogee standard deviation 3sigma: %.4f \n',3*apogee.altitude_std);
        fprintf(fid,'Apogees within +-10m from target (gaussian): %.2f %% \n',apogee.accuracy_gaussian_10);
        fprintf(fid,'Apogees within +-10m from target (ratio): %.2f %% \n\n',apogee.accuracy_10);
        fprintf(fid,'Apogees within +-50m from target (gaussian): %.2f %% \n',apogee.accuracy_gaussian_50);
        fprintf(fid,'Apogees within +-50m from target (ratio): %.2f %% \n\n',apogee.accuracy_50);
        fprintf(fid,'Apogees horizontal distance from origin mean : %.2f [m] \n',apogee.radius_mean);
        fprintf(fid,'Apogees horizontal distance from origin std : %.2f [m] \n\n',apogee.radius_std);
        fprintf(fid,'Apogees horizontal speed mean : %.2f [m/s] \n',apogee.horizontalSpeed_mean);
        fprintf(fid,'Apogees horizontal speed std : %.2f [m/s] \n\n',apogee.horizontalSpeed_std);
        fprintf(fid,'Mean shutdown time : %.3f [m/s] \n',t_shutdown.mean);
        fprintf(fid,'Standard deviation of shutdown time : %.3f [m/s] \n\n\n',t_shutdown.std);
    end

    fprintf(fid,'\n\n');

    if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil
        fprintf(fid,'RESULTS LANDING\n\n');
        fprintf(fid,'Max distance to target: %.2f m\n',max(landing.distance_to_target));
        fprintf(fid,'Min distance to target: %.2f m\n',min(landing.distance_to_target));
        fprintf(fid,'Mean distance to target: %.2f m\n',min(landing.distance_mean));
        fprintf(fid,'Distance standard deviation 3sigma: %.4f \n',3*landing.distance_std);
        fprintf(fid,'Landings within 50m from target (ratio): %.2f %% \n\n',landing.accuracy_50);
        fprintf(fid,'Landings within 150m from target (ratio): %.2f %% \n\n',landing.accuracy_150);
        fprintf(fid,'Apogees horizontal speed mean : %.2f [m/s] \n',apogee.horizontalSpeed_mean);
        fprintf(fid,'Apogees horizontal speed std : %.2f [m/s] \n\n',apogee.horizontalSpeed_std);
        fprintf(fid,'Mean shutdown time : %.3f [m/s] \n',t_shutdown.mean);
        fprintf(fid,'Standard deviation of shutdown time : %.3f [m/s] \n\n\n',t_shutdown.std);
    end

    fclose(fid);
    fprintf('\nSaved all data to %s\n\n', folder+"\"+"Results_"+ num2str(N_sim) + "Sim.txt");
 end