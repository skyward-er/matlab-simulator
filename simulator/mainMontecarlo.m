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
                settings_mont.wind.unc = stoch.wind.unc(i,:);
        end
        
        if displayIter == true
            fprintf("simulation = " + num2str(i) + " of " + num2str(N_sim) + ", algorithm: " + contSettings.algorithm +", scenario: "+ settings.scenario +"\n");
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [simOutput] = std_run(settings,contSettings,settings_mont);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        save_thrust{i} = simOutput;

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

    
    %% PLOTS

    plotsMontecarlo;
    
    %% SAVE
    % save plots
    saveDate = replace(string(datetime),":","_");
    saveDate = replace(saveDate," ","__");
    saveDate = replace(saveDate,"-","_");
    folder = [];

    if flagSaveOffline == "yes" || flagSaveOffline == "y"
        switch  settings.mission

            case 'Pyxis_Portugal_October_2022'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case 'Gemini_Portugal_October_2023'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case 'Pyxis_Roccaraso_September_2022'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\z_f_"+settings.z_final+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline
        
        
            case 'Gemini_Roccaraso_September_2023'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline

        end

    end
    if flagSaveOnline == "yes" || flagSaveOnline == "y"
        if computer == "Marco" || computer == "marco"
            folder = [folder ; "C:\Users\marco\OneDrive - Politecnico di Milano\SKYWARD\AIR BRAKES\MONTECARLO E TUNING\"+settings.mission+"\"+conf.scenario+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"_"+simulationType_thrust+"_"+saveDate]; % online
        end
        if computer == "Max" || computer == "max"
            folder = [folder ; "C:\Users\Max\OneDrive - Politecnico di Milano\SKYWARD\AIR BRAKES\MONTECARLO E TUNING\"+settings.mission+"\"+conf.scenario+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"_"+simulationType_thrust+"_"+saveDate]; % online
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [simOutput] = std_run(settings,contSettings,settings_mont);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    save_thrust{i} = simOutput;

end


%% generate file for rocketpy (euroc 2023 prelaunch)
for i = 1:N_sim
    [apogee.coordinates(i,1),apogee.coordinates(i,2),apogee.coordinates(i,3)] = ned2geodetic(save_thrust{i}.apogee.position(1),save_thrust{i}.apogee.position(2),save_thrust{i}.apogee.position(3),settings.lat0,settings.lon0,settings.z0,wgs84Ellipsoid);
    [landing.coordinates(i,1),landing.coordinates(i,2),landing.coordinates(i,3)] = ned2geodetic(save_thrust{i}.Y(end,1),save_thrust{i}.Y(end,2),save_thrust{i}.Y(end,3),settings.lat0,settings.lon0,settings.z0,wgs84Ellipsoid);
end
landing.coordinates = landing.coordinates(:,1:2);

figure
geoplot(landing.coordinates(:,1),landing.coordinates(:,2),'LineStyle','none','Marker','.','MarkerSize',10,'Color','blue','DisplayName','Landings')
hold on;
geoplot(apogee.coordinates(:,1),apogee.coordinates(:,2),'LineStyle','none','Marker','.','MarkerSize',10,'Color','red','DisplayName','Apogees')
geobasemap satellite
legend

if conf.scenario == "ballistic"
    save('ballistic_simulations','apogee','landing')
else
    save('parachute_simulations','apogee','landing')
end

return
%% generate files
load ballistic_simulations
ball.apogee = apogee;
ball.landing = landing;
load parachute_simulations
para.apogee = apogee;
para.landing = landing;


varNamesAsc = {'ApogeeLatitude', 'ApogeeLongitude', 'ApogeeAltitude'}; 
varNamesDesc = {'ballLandingLatitude', 'ballLandingLongitude', 'paraLandingLatitude', 'paraLandingLongitude'}; 
ascentTab = table; 
descentTab = table; 
ascentTab(:, 1) = table(ball.apogee.coordinates(:,1)); 
ascentTab(:, 2) = table(ball.apogee.coordinates(:,2)); 
ascentTab(:, 3) = table(ball.apogee.altitude'); 
ascentTab.Properties.VariableNames = varNamesAsc; 
writetable(ascentTab, 'ascent_MC_simulations_CL.csv'); 
descentTab(:, 1) = table(ball.landing.coordinates(:,1)); 
descentTab(:, 2) = table(ball.landing.coordinates(:,2)); 
descentTab(:, 3) = table(para.landing.coordinates(:,1)); 
descentTab(:, 4) = table(para.landing.coordinates(:,2)); 
descentTab.Properties.VariableNames = varNamesDesc; 
writetable(descentTab, 'descent_MC_simulations_CL.csv');
