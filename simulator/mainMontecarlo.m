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

%% recall the first part of the MAIN script
% adds folders to the path and retrieves rocket, mission, simulation, etc
% data.

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


%% do you want to save the results?

flagSaveOffline = input('Do you want to save the results offline? (y/n): ','s');
flagSaveOnline = input('Do you want to save the resuts online? (oneDrive) (y/n): ','s');


%% MONTECARLO ANALYSIS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONFIG%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% other parameters you want to set for the particular simulation:

clearvars   msaToolkitURL Itot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONFIG%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

settings_mont_init = struct('x',[]);
rocket_vec = cell(N_sim, 1);

if ~exist("wind_vec", 'var')
    wind_vec = cell(N_sim, 1);
end

%% start simulation
for alg_index = 4

    contSettings.algorithm = algorithm_vec{alg_index};

    %save arrays
    save_thrust = cell(size(stoch.thrust,1),1);
    apogee.altitude = [];
    wind_Mag = zeros(N_sim,1);
    wind_el = zeros(N_sim,1);
    wind_az = zeros(N_sim,1);
    t_shutdown.value = zeros(N_sim,1);
    motor_K = settings.motor.K;

    parfor i = 1:N_sim
        rocket_vec{i} = rocket;
        settings_mont = settings_mont_init;
        settings_mont.motor.expThrust = stoch.thrust(i,:);                      % initialize the thrust vector of the current simulation (parfor purposes)
        settings_mont.motor.expTime = stoch.expTime(i,:);                     % initialize the time vector for thrust of the current simulation (parfor purposes)
        settings_mont.motor.K = stoch.Kt(i,:);
        settings_mont.mass_offset = stoch.mass_offset(i);
        settings_mont.OMEGA = stoch.OMEGA_rail(i);
        settings_mont.PHI = stoch.PHI_rail(i);
        settings_mont.ABK.PID_coeffs = stoch.ABK_curve(i, :);
        settings_mont.ABK.PID_ref = stoch.ABK_ref(i);

        if isfield(stoch, 'State')
            settings_mont.State.xcgTime = stoch.State.xcgTime(:,i);                 % initialize the baricenter position time vector
        end

        if isempty(wind_vec{i})
            wind_vec{i} = stoch.wind.updateAll();
        end
        
        settings_mont.wind = wind_vec{i};
       
        % Define coeffs matrix for the i-th simulation
        settings_mont.Coeffs = rocket.coefficients.total * (1+stoch.aer_percentage(i));

        
        if displayIter == true
            fprintf("simulation = " + num2str(i) + " of " + num2str(N_sim) + ", algorithm: " + contSettings.algorithm +", scenario: "+ settings.scenario +"\n");
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [simOutput] = std_run(settings,contSettings,rocket_vec{i},environment,mission,settings_mont);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        save_thrust{i} = simOutput;
        save_thrust{i}.ARB.K_vals = stoch.ABK_curve(i,:);
        save_thrust{i}.ARB.ref = stoch.ABK_ref(i);

    end

    clearvars   i l j jj crossCase

    %% RETRIEVE INTERESING PARAMETERS:

    N_ApogeeWithinTarget_10 = 0;
    N_ApogeeWithinTarget_50 = 0;
    N_landings_within50m = 0;
    N_landings_within150m = 0;
    
   
    for i = 1:N_sim

        % apogee
        apogee.altitude(i) = -save_thrust{i}.apogee.position(3);

        % radius of apogee (horizontal) from the initial point
        apogee.radius(i) = save_thrust{i}.apogee.radius;

        % NAS error
        nas.error(i, :) = mean(abs(save_thrust{i}.sensors.nas.error), 1);
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

    %%     
    for ii = 1:N_sim
        if sum(save_thrust{ii}.sensors.ada.apo_times == -1) > 1
            disp("Problems with ADA " + num2str(ii));
        end
    end

    %%

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
    folder = [];

    if flagSaveOffline == "yes" || flagSaveOffline == "y"
        switch  mission.name

            case 'Pyxis_Portugal_October_2022'
                folder = [folder ; "MontecarloResults\"+mission.name+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*rocket.airbrakes.maxMach)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case 'Gemini_Portugal_October_2023'
                folder = [folder ; "MontecarloResults\"+mission.name+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*rocket.airbrakes.maxMach)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case 'Pyxis_Roccaraso_September_2022'
                folder = [folder ; "MontecarloResults\"+mission.name+"\z_f_"+settings.z_final+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*rocket.airbrakes.maxMach)+"_"+simulationType_thrust+"_"+saveDate]; % offline
        
            case 'Gemini_Roccaraso_September_2023'
                folder = [folder ; "MontecarloResults\"+mission.name+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*rocket.airbrakes.maxMach)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case {'2024_Lyra_Portugal_October', '2024_Lyra_Roccaraso_September'}
                folder = [folder ; "MontecarloResults\"+mission.name+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*rocket.airbrakes.maxMach)+"_"+simulationType_thrust+"_"+saveDate]; % offline
            
            case {'2025_Orion_Portugal_October', '2025_Orion_Roccaraso_September'}
                folder = [folder ; "MontecarloResults\"+mission.name+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*rocket.airbrakes.maxMach)+"_"+simulationType_thrust+"_"+saveDate]; % offline
        end

    end
    if flagSaveOnline == "yes" || flagSaveOnline == "y"
        currentDir = pwd;
        usr = split(currentDir,'\');
        usr = usr{3};
        OneDrivePath = strcat("C:\Users\",usr,"\OneDrive - Politecnico di Milano\SKYWARD\matlab-simulator-results\");
        folder = [folder ; OneDrivePath+mission.name+"\"+conf.scenario+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"_"+simulationType_thrust+"_"+saveDate]; % online
    end

    if flagSaveOffline == "yes" || flagSaveOnline == "yes" || flagSaveOffline == "y" || flagSaveOnline == "y"
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
                fprintf(fid,'PID proportional gain %s \n',rocket.parachutes(2,2).controlParams.Kp);
                fprintf(fid,'PID integral gain %s \n',rocket.parachutes(2,2).controlParams.Ki);
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
end


%% generate file for rocketpy (euroc 2023 prelaunch)
% for i = 1:N_sim
%     [apogee.coordinates(i,1),apogee.coordinates(i,2),apogee.coordinates(i,3)] = ned2geodetic(save_thrust{i}.apogee.position(1),save_thrust{i}.apogee.position(2),save_thrust{i}.apogee.position(3),environment.lat0,environment.lon0,environment.z0,wgs84Ellipsoid);
%     [landing.coordinates(i,1),landing.coordinates(i,2),landing.coordinates(i,3)] = ned2geodetic(save_thrust{i}.Y(end,1),save_thrust{i}.Y(end,2),save_thrust{i}.Y(end,3),environment.lat0,environment.lon0,environment.z0,wgs84Ellipsoid);
% end
% landing.coordinates = landing.coordinates(:,1:2);
% 
% figure
% geoplot(landing.coordinates(:,1),landing.coordinates(:,2),'LineStyle','none','Marker','.','MarkerSize',10,'Color','blue','DisplayName','Landings')
% hold on;
% geoplot(apogee.coordinates(:,1),apogee.coordinates(:,2),'LineStyle','none','Marker','.','MarkerSize',10,'Color','red','DisplayName','Apogees')
% geobasemap satellite
% legend
% 
% if conf.scenario == "ballistic"
%     save('ballistic_simulations','apogee','landing')
% else
%     save('parachute_simulations','apogee','landing')
% end
% 
% return
% %% generate files
% load ballistic_simulations_launchDay4.mat
% ball.apogee = apogee;
% ball.landing = landing;
% load parachute_simulations_launchDay4.mat
% para.apogee = apogee;
% para.landing = landing;
% 
% figure 
% geoplot(ball.apogee.coordinates(:,1),ball.apogee.coordinates(:,2),'Marker','.','MarkerSize',10,'LineStyle','none')
% hold on
% geoplot(ball.landing.coordinates(:,1),ball.landing.coordinates(:,2),'Marker','.','MarkerSize',10,'LineStyle','none')
% geoplot(para.landing.coordinates(:,1),para.landing.coordinates(:,2),'Marker','.','MarkerSize',10,'LineStyle','none')
% geobasemap satellite
% legend
% 
% varNamesAsc = {'ApogeeLatitude', 'ApogeeLongitude', 'ApogeeAltitude'}; 
% varNamesDesc = {'ballLandingLatitude', 'ballLandingLongitude', 'paraLandingLatitude', 'paraLandingLongitude'}; 
% ascentTab = table; 
% descentTab = table; 
% ascentTab(:, 1) = table(ball.apogee.coordinates(:,1)); 
% ascentTab(:, 2) = table(ball.apogee.coordinates(:,2)); 
% ascentTab(:, 3) = table(ball.apogee.altitude'); 
% ascentTab.Properties.VariableNames = varNamesAsc; 
% writetable(ascentTab, 'ascent_MC_simulations_CL_launchDay4.csv'); 
% descentTab(:, 1) = table(ball.landing.coordinates(:,1)); 
% descentTab(:, 2) = table(ball.landing.coordinates(:,2)); 
% descentTab(:, 3) = table(para.landing.coordinates(:,1)); 
% descentTab(:, 4) = table(para.landing.coordinates(:,2)); 
% descentTab.Properties.VariableNames = varNamesDesc; 
% writetable(descentTab, 'descent_MC_simulations_CL_launchDay4.csv');