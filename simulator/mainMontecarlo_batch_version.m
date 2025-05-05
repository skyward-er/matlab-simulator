%{

script that runs the Montecarlo simulations to validate the control
algorithms. Same as main, but some parameters are cicled.
CALLED FUNCTIONS: 

0 -     Marco Marchesi, SCS department, marco.marchesi@skywarder.eu
        Giuseppe Brentino, SCS department, giuseppe.brentino@skywarder.eu
        Latest control algorithms
1 -     Pier Francesco Bachini, GNC IPT, pierfrancesco.bachini@skywarder.eu
        Refactoring of the script and batch run implementation

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

%% recall the first part of the MAIN script
% adds folders to the path and retrieves rocket, mission, simulation, etc
% data.

close all; clearvars; clc;

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

% add common data path
dataPath = strcat('../common');
addpath(genpath(dataPath));

%% CONFIGs
conf.script = "simulator";
settings.montecarlo = true;
configSimulator;

%% MONTECARLO SETTINGS
rng default
matlab_graphics;

%% check on the simulation profile:
answer = questdlg({'Did you check the profile of the simulation? ', ...
    'A.K.A. did you check that: settings.scenario = "controlled ascent" or "descent" or whatever?'});

switch answer
    case 'Yes'
        fprintf('All right, let''s go \n\n')
    case 'No'
        error('Set it then. Simulation is stopped!')
    case 'Cancel'
        error('Simulation aborted!')
    otherwise
end

%% do you want to save the results?

% flagSaveOffline = questdlg('Do you want to save the results offline? ','s');
% 
% if flagSaveOffline == "Cancel"
%     error('Simulation aborted!')
% end

flagSaveOffline = "Yes";

%% Prepare batch saving
N_remain = N_sim;
pool = gcp;
N_workers = pool.NumWorkers;
emptyStruct = struct;
if exist("save_thrust.mat", "file")
    delete("save_thrust.mat");
end
save("save_thrust.mat", '-struct', 'emptyStruct', '-mat', '-v7.3');
save_thrust_MAT = matfile("save_thrust.mat", "Writable", true);

%% Prepare saving arrays

apogee.altitude = [];
wind_Mag = zeros(N_sim,1);
wind_el = zeros(N_sim,1);
wind_az = zeros(N_sim,1);
t_shutdown.value = zeros(N_sim,1);
motor_K = settings.motor.K;

pw = PoolWaitbar(N_sim, 'Simulation progression');

tic;
ticBytes(pool);

while N_remain > 0

    N_batch = min(N_remain, N_workers);
    save_thrust = cell(N_batch, 1);
    rocket_vec = cell(N_batch, 1);
    wind_vec = cell(N_batch, 1);

    try
        run_size = size(save_thrust_MAT, 'save_thrust');
    catch ME
        if strcmp(ME.identifier, 'MATLAB:MatFile:VariableNotInFile')
            run_size = 0;
        else
            rethrow(ME);
        end
    end

    run_size = max(run_size) + 1;

    parfor ii = 1:N_batch
        rocket_vec{ii} = rocket;
        settings_mont = struct;
        settings_mont.motor.expThrust = stoch.thrust(ii,:);                      % initialize the thrust vector of the current simulation (parfor purposes)
        settings_mont.motor.expTime = stoch.expTime(ii,:);                     % initialize the time vector for thrust of the current simulation (parfor purposes)
        settings_mont.motor.K = stoch.Kt(ii,:);                  %
        settings_mont.mass_offset = stoch.mass_offset(ii);
        settings_mont.OMEGA = stoch.OMEGA_rail(ii);
        settings_mont.PHI = stoch.PHI_rail(ii);

        if rocket.dynamicDerivatives
            settings_mont.State.xcgTime = stoch.State.xcgTime(:,i);                 % initialize the baricenter position time vector
        end

        if isempty(wind_vec{ii})
            wind_vec{ii} = stoch.wind.updateAll();
        end

        settings_mont.wind = wind_vec{ii};

        % Define coeffs matrix for the i-th simulation
        settings_mont.Coeffs = rocket.coefficients.total * (1+stoch.aer_percentage(i));

        % Update WaitBar
        increment(pw);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [simOutput] = std_run(settings,contSettings,rocket_vec{ii},environment,mission,settings_mont);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        save_thrust{ii} = simOutput;
    end

    save_thrust_MAT.save_thrust(run_size:run_size+N_batch-1,1) = save_thrust;
    save_thrust_MAT.rocket_vec(run_size:run_size+N_batch-1,1) = rocket_vec;
    save_thrust_MAT.wind_vec(run_size:run_size+N_batch-1,1) = wind_vec;

    N_remain = N_remain - N_batch;
end
mont_time = toc;
mont_bytes = tocBytes(pool);

return
%%
N_ApogeeWithinTarget_10 = 0;
N_ApogeeWithinTarget_50 = 0;
N_landings_within50m = 0;
N_landings_within150m = 0;


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

    % wind magnitude
    wind_Mag(i) = wind_vec{i}.magnitude(1);
    % wind azimuth
    wind_az(i) = wind_vec{i}.azimuth(1);
    %wind elevation
    wind_el(i) = wind_vec{i}.elevation(1);

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

    % landing
    if ~isnan(save_thrust{i}.PRF.landing_position)
        landing.position(i,:) = save_thrust{i}.PRF.landing_position;
        landing.velocities_BODY(i,:) = save_thrust{i}.PRF.landing_velocities_BODY;
        landing.velocities_NED(i,:) = save_thrust{i}.PRF.landing_velocities_NED;
        landing.distance_to_target(i) = norm(settings.payload.target(1:2)-landing.position(i,1:2)');

        % save apogees within 50 meters (radius) from target:
        if landing.distance_to_target(i) <= 50
            N_landings_within50m = N_landings_within50m +1; % save how many apogees sit in the +-50 m from target
        end
        % save apogees within 150 meters (radius) from target:
        if landing.distance_to_target(i) <= 150
            N_landings_within150m = N_landings_within150m +1; % save how many apogees sit in the +-50 m from target
        end
    elseif conf.scenario == "ballistic"

        landing.position(i,:) = save_thrust{i}.Y(end,[2,1,3]);
        landing.velocities_BODY(i,:) = save_thrust{i}.Y(end,4:6);
        landing.velocities_NED(i,:) = quatrotate(quatconj(save_thrust{i}.Y(end,10:13)),save_thrust{i}.Y(end,4:6));
        landing.distance_to_target(i) = NaN;

        N_landings_within50m = NaN; % save how many apogees sit in the +-50 m from target
        N_landings_within150m = NaN; % save how many apogees sit in the +-50 m from target

    else
        landing.position(i,:) = [NaN NaN NaN];
        landing.velocities_BODY(i,:) = [NaN NaN NaN];
        landing.velocities_NED(i,:) = [NaN NaN NaN];
        landing.distance_to_target(i) = NaN;

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
for i = 1:N_sim
    apogee_mu(i) = mean(apogee.altitude(1:i));
    apogee_sigma(i) = std(apogee.altitude(1:i));

    landing_mu(i) = mean(landing.distance_to_target(1:i));
    landing_sigma(i) = std(landing.distance_to_target(1:i));
end
% gaussian 10m
p_10 = normcdf([settings.z_final-10, settings.z_final+10],apogee.altitude_mean,apogee.altitude_std);
apogee.accuracy_gaussian_10 =( p_10(2) - p_10(1) )*100;
% gaussian 50m
p_50 = normcdf([settings.z_final-50, settings.z_final+50],apogee.altitude_mean,apogee.altitude_std);
apogee.accuracy_gaussian_50 =( p_50(2) - p_50(1) )*100;

%% Check ada consisencies
ada_diff = zeros(N_sim, 1);
for ii = 1:N_sim
    if save_thrust{ii}.sensors.old_ada.t_apogee ~= -1
        ada_diff(ii) = save_thrust{ii}.sensors.ada.t_apogee - save_thrust{ii}.sensors.old_ada.t_apogee;
    else
        ada_diff(ii) = -1;
    end
end

%% PLOTS

plotsMontecarlo;

%% SAVE
% save plots
saveDate = replace(string(datetime),":","_");
saveDate = replace(saveDate," ","__");
saveDate = replace(saveDate,"-","_");

if flagSaveOffline == "Yes"
    folder = "MontecarloResults\"+mission.name+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*rocket.airbrakes.maxMach)+"_"+simulationType_thrust+"_"+saveDate;

    for i = 1:length(folder)
        if ~exist(folder(i),"dir")
            mkdir(folder(i))
        end
        save(folder(i)+"\montecarloFigures",'montFigures')

        save(folder(i)+"\saveThrust.mat","save_thrust","apogee","N_sim","settings","thrust_percentage","stoch") % add "save_thrust", > 2GB for 1000 sim

        % Save results.txt
        fid = fopen( folder(i)+"\"+contSettings.algorithm+"Results"+saveDate+".txt", 'wt' );  % CAMBIA IL NOME
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
        fprintf(fid,'Initial Mach number at which the control algorithm starts: %.3f \n\n',rocket.airbrakes.maxMach);
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
        % %             switch alg_index
        % %                 case 2
        % %                     fprintf(fid,'P = %d \n',contSettings.Kp );
        % %                     fprintf(fid,'I = %d \n',contSettings.Ki );
        % %                     fprintf(fid,'Never change reference trajectory \n\n' );
        % %                 case 3
        % %                     fprintf(fid,'P = %d \n',contSettings.Kp );
        % %                     fprintf(fid,'I = %d \n',contSettings.Ki );
        % %                     fprintf(fid,'Change reference trajectory every %d seconds \n\n',contSettings.deltaZ_change );
        % %             end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        fprintf(fid,'WIND \n\n');
        fprintf(fid,'Wind model parameters: \n'); % inserisci tutti i parametri del vento
        fprintf(fid,'Wind altitudes: '+string(repmat('%.2f m ', [1 length(wind.altitudes)]))+"\n\n", wind.altitudes);

        for ii = 1:length(wind.altitudes)
            fprintf(fid,'Wind magnitude distribution at %.2f m: \"%s\"\n', wind.altitudes(ii), string(wind.magnitudeDistribution(ii)));
            switch wind.magnitudeDistribution(ii)
                case "u"
                    fprintf(fid,'Wind magnitude parameters at %.2f m: %d-%d m/s\n', wind.altitudes(ii), stoch.wind_params.MagMin(ii), stoch.wind_params.MagMax(ii));
                case "g"
                    fprintf(fid,'Wind magnitude parameters at %.2f m: %d-%d m/s\n', wind.altitudes(ii), stoch.wind_params.MagMean(ii), stoch.wind_params.MagStd(ii));
            end
        end
        fprintf(fid, '\n');

        for ii = 1:length(wind.altitudes)
            fprintf(fid,'Wind azimuth distribution at %.2f m: \"%s\"\n', wind.altitudes(ii), string(wind.azimuthDistribution(ii)));
            switch wind.azimuthDistribution(ii)
                case "u"
                    fprintf(fid,'Wind azimuth parameters at %.2f m: %d-%d [°] \n', wind.altitudes(ii), rad2deg(stoch.wind_params.AzMin(ii)), rad2deg(stoch.wind_params.AzMax(ii)));
                case "g"
                    fprintf(fid,'Wind azimuth parameters at %.2f m: %d-%d m/s\n', wind.altitudes(ii), rad2deg(stoch.wind_params.AzMean(ii)), rad2deg(stoch.wind_params.AzStd(ii)));
            end
        end
        fprintf(fid, '\n');

        for ii = 1:length(wind.altitudes)
            fprintf(fid,'Wind elevation distribution at %.2f m: \"%s\"\n', wind.altitudes(ii), string(wind.elevationDistribution(ii)));
            switch wind.elevationDistribution(ii)
                case "u"
                    fprintf(fid,'Wind elevation parameters at %.2f m: %d-%d [°] \n', wind.altitudes(ii), rad2deg(stoch.wind_params.ElMin(ii)), rad2deg(stoch.wind_params.ElMax(ii)));
                case "g"
                    fprintf(fid,'Wind elevation parameters at %.2f m: %d-%d [°] \n', wind.altitudes(ii), rad2deg(stoch.wind_params.ElMean(ii)), rad2deg(stoch.wind_params.ElStd(ii)));
            end
        end
        fprintf(fid,'\n\n');

        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        if (settings.scenario == "descent" || settings.scenario == "full flight") && settings.parafoil
            fprintf(fid,'PARAFOIL \n\n');
            fprintf(fid,'Guidance approach %s \n',contSettings.payload.guidance_alg);
            fprintf(fid,'PID proportional gain %s \n',rocket.parachutes{2,2}.controlParams.Kp);
            fprintf(fid,'PID integral gain %s \n',rocket.parachutes{2,2}.controlParams.Ki);
            fprintf(fid,'Opening altitude %s \n', num2str(settings.ada.para.z_cut));
        end
        fprintf(fid,'MASS: \n\n');
        fprintf(fid,'Interval : +-%d at 3sigma \n',3*sigma_m );
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
        fprintf('\nsaved\n\n')
    end
end