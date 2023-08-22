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
clearvars -except ZTARGET_CYCLE
close all; clear; clc;

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
config;

%% MONTECARLO SETTINGS
rng default
settings.montecarlo = true;
matlab_graphics;

%% how many simulations
N_sim = 10; % set to at least 500
simulationType_thrust = "gaussian";  % "gaussian", "exterme"

%% stochastic parameters

switch simulationType_thrust

    case "gaussian"

        sigma_t = (1.10-1)/3;             % thrust_percentage standard deviation
        mu_t = 1;                         % thrust_percentage mean value

        thrust_percentage = normrnd(mu_t,sigma_t,N_sim,1);       %generate normally distributed values ( [0.8 1.20] = 3sigma) % serve il toolbox
        stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
        
        
        %%% in questo modo però il burning time rimane fissato perchè è
        %%% settato al'interno di simulationData.m -> possibili errori per
        %%% le simulazioni con exp_thrust aumentato se si vuole fare
        %%% spegnimento dell'ibrido
        impulse_uncertainty = normrnd(1,0.02/3,N_sim,1);
        stoch.expThrust = diag(impulse_uncertainty)*((1./thrust_percentage) * settings.motor.expTime);          % burning time - same notation as thrust here
        %%%
        for i =1:N_sim
            stoch.State.xcgTime(:,i) =  settings.State.xcgTime/settings.tb .* stoch.expThrust(i,end);  % Xcg time
        end
        
        %%% Aero coefficients uncertainty

        sigma_aer = (0.1)/3;             % aero coeffs error standard deviation
        mu_aer = 0;                      % aero coeffs error mean value
        aer_percentage = normrnd(mu_aer,sigma_aer,N_sim,1);       

        %%% wind parameters
        settings.wind.MagMin = 0;                                               % [m/s] Minimum Wind Magnitude
        settings.wind.MagMax = 10;                                               % [m/s] Maximum Wind Magnitude
        settings.wind.ElMin  = - deg2rad(45);
        settings.wind.ElMax  = + deg2rad(45);
        settings.wind.AzMin  = - deg2rad(180);
        settings.wind.AzMax  = + deg2rad(180);

        switch settings.windModel
            case "constant"
                [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust);
            case "multiplicative"
                [stoch.wind.Mag,stoch.wind.Az] = windMultGeneratorMontecarlo(settings.wind,N_sim);
        end

    case "extreme"


        thrust_percentage = [0.9;1.1]; % this is overwritten in the next step, but it sets the values to retrieve in the parameter generation

        %%% wind parameters
        [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ,thrust_percentage, N_sim] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust,thrust_percentage);

        stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
        %%%
        stoch.expThrust = (1./thrust_percentage) * settings.motor.expTime;          % burning time - same notation as thrust here
end


%% check on the simulation profile:
go = '';
while ~strcmp(go,'yes') && ~strcmp(go,'y') && ~strcmp(go,'no') && ~strcmp(go,'n')
    go = input('Did you check that the profile of the simulation? \nA.K.A. did you check settings.scenario = "controlled ascent" or " "descent" or whatever? ("yes" or "no") \n','s');
    if go == "yes" || go == "y"
        fprintf('All right, let''s go \n\n')
    elseif go == "no" || go == "n"
        fprintf('Set it then. Simulation is stopped. \n\n')
        return
    else
        fprintf('You typed something different from the options, retry. \n\n')
    end
end

%% save arrays

% algorithms
algorithm_vec = {'interp';'NoControl';'engine';'complete'; 'PID_2021'; 'shooting'}; % interpolation, no control, engine shutdown, engine+arb, PID change every 2s, shooting


%% do you want to save the results?

flagSaveOffline = input('Do you want to save the results offline? ("yes" or "no"): ','s');
flagSaveOnline = input('Do you want to save the resuts online? (oneDrive) ("yes" or "no"): ','s');

if flagSaveOnline == "yes" || flagSaveOnline == "y"
    computer = input('Who is running the simulation? ("Marco" or "Giuseppe" or "Hpe" or whatever): ','s');
end

displayIter = true; % set to false if you don't want to see the iteration number (maybe if you want to run Montecarlos on hpe)

%% MONTECARLO ANALYSIS

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONFIG%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% other parameters you want to set for the particular simulation:

clearvars   msaToolkitURL Itot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONFIG%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

settings_mont_init = struct('x',[]);

% start simulation
for alg_index = 4

    contSettings.algorithm = algorithm_vec{alg_index};

    %save arrays
    save_thrust = cell(size(stoch.thrust,1),1);
    apogee.altitude = [];
    N_ApogeeWithinTarget = 0;
    N_ApogeeWithinTarget_50 = 0;
    N_landings_within50m = 0;
    N_landings_within150m = 0;
    wind_Mag = zeros(N_sim,1);
    wind_el = zeros(N_sim,1);
    wind_az = zeros(N_sim,1);
    t_shutdown.value = zeros(N_sim,1);

    parfor i = 1:N_sim
        settings_mont = settings_mont_init;
        %         contSettings_mont = contSettings;
        %         reference_mont = reference;

        settings_mont.motor.expThrust = stoch.thrust(i,:);                      % initialize the thrust vector of the current simulation (parfor purposes)
        settings_mont.motor.expTime = stoch.expThrust(i,:);                     % initialize the time vector for thrust of the current simulation (parfor purposes)
        settings_mont.tb = max( stoch.expThrust(i,stoch.expThrust(i,:)<=settings.tb) );                              % initialize the burning time of the current simulation (parfor purposes)
        settings_mont.State.xcgTime = stoch.State.xcgTime(:,i);                 % initialize the baricenter position time vector

        % Define coeffs matrix for the i-th simulation
        settings_mont.Coeffs = settings.Coeffs* (1+aer_percentage(i));


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
        [simOutput] = std_run(settings,contSettings,settings_mont);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        save_thrust{i} = simOutput;

    end

    clearvars   i l j jj crossCase

    %% RETRIEVE INTERESING PARAMETERS:

    for i = 1:N_sim

        % apogee
        apogee.altitude(i) = save_thrust{i}.apogee_coordinates(3);

        % radius of apogee (horizontal) from the initial point
        apogee.radius(i) = save_thrust{i}.apogee_radius;

        % horizontal speed at apogee
        apogee.horizontalSpeed(i) = norm(save_thrust{i}.Y(end,4:6)); % in theory this is the body frame, but as the last point is the apogee we should have only  horizontal velocity, so all the components must be taken

        % time of engine shutdown
        t_shutdown.value(i) = save_thrust{i}.t_shutdown;

        if ~settings.wind.model && ~settings.wind.input
            % wind magnitude
            wind_Mag(i) = save_thrust{i}.windMag;
            % wind azimuth
            wind_az(i) = save_thrust{i}.windAz;
            %wind elevation
            wind_el(i) = save_thrust{i}.windEl;
        end
        %reached apogee time
        apogee.times(i) = save_thrust{i}.apogee_time;
        apogee.prediction(i) = save_thrust{i}.predicted_apogee(end);
        apogee.prediction_last_time(i) = length(save_thrust{i}.predicted_apogee)/settings.frequencies.controlFrequency;
       
        % save apogees within +-10 meters from target:
        if abs(apogee.altitude(i) - settings.z_final)<=10
            N_ApogeeWithinTarget = N_ApogeeWithinTarget +1; % save how many apogees sit in the +-50 m from target
        end
        % save apogees within +-50 meters from target:
        if abs(apogee.altitude(i) - settings.z_final)<=50
            N_ApogeeWithinTarget_50 = N_ApogeeWithinTarget_50 +1; % save how many apogees sit in the +-50 m from target
        end
       
        % landing
        landing.position(i,:) = save_thrust{i}.landing_position;
        landing.velocities_BODY(i,:) = save_thrust{i}.landing_velocities_BODY;
        landing.velocities_NED(i,:) = save_thrust{i}.landing_velocities_NED;
        landing.distance_to_target(i) = norm(settings.payload.target(1:2)-landing.position(i,1:2)');

        % save apogees within 50 meters (radius) from target:
        if landing.distance_to_target(i) <= 50
            N_landings_within50m = N_landings_within50m +1; % save how many apogees sit in the +-50 m from target
        end
        % save apogees within 150 meters (radius) from target:
        if landing.distance_to_target(i) <= 150
            N_landings_within150m = N_landings_within150m +1; % save how many apogees sit in the +-50 m from target
        end
    end
    
    % merit parameters
    apogee.altitude_mean = mean(apogee.altitude);
    apogee.altitude_std = std(apogee.altitude);

    landing.distance_mean = mean(landing.distance_to_target);
    landing.distance_std = std(landing.distance_to_target);

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

    % save landing speed
    landing.verticalSpeed_mean = mean(landing.velocities_NED(:,3));
    landing.verticalSpeed_std = std(landing.velocities_NED(:,3));
    landing.verticalSpeed_max = max(landing.velocities_NED(:,3));
    landing.verticalSpeed_min = min(landing.velocities_NED(:,3));

    
    landing.horizontalSpeed_mean = mean(norm(landing.velocities_NED(:,1:2)));
    landing.horizontalSpeed_std = std(norm(landing.velocities_NED(:,1:2)));
    landing.horizontalSpeed_max = max(norm(landing.velocities_NED(:,1:2)));
    landing.horizontalSpeed_min = min(norm(landing.velocities_NED(:,1:2)));

    % accuracy
    apogee.accuracy_10 = N_ApogeeWithinTarget/N_sim*100; % percentage, so*100
    apogee.accuracy_50 = N_ApogeeWithinTarget_50/N_sim*100; % percentage, so*100
    
    landing.accuracy_50 = N_landings_within50m/N_sim*100;
    landing.accuracy_150 = N_landings_within150m/N_sim*100;

    % montecarlo parameters
    for i = 1:N_sim
        apogee_mu(i) = mean(apogee.altitude(1:i));
        apogee_sigma(i) = std(apogee.altitude(1:i));
        landing_mu(i) = mean(landing.distance_to_target(1:i));
        landing_sigma(i) = std(landing.distance_to_target(1:i));
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
        switch  settings.mission

            case 'Pyxis_Portugal_October_2022'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case 'Gemini_Portugal_October_2023'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case 'Pyxis_Roccaraso_September_2022'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\z_f_"+settings.z_final+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline
        end

    end
    if flagSaveOnline == "yes" || flagSaveOnline == "y"
        if computer == "Marco" || computer == "marco"
            folder = [folder ; "C:\Users\marco\OneDrive - Politecnico di Milano\SKYWARD\AIR BRAKES\MONTECARLO E TUNING\"+settings.mission+"\full_flight\"+contSettings.algorithm+"\"+num2str(N_sim)+"_"+simulationType_thrust+"_"+saveDate]; % online
        end
    end

    if flagSaveOffline == "yes" || flagSaveOnline == "yes" || flagSaveOffline == "y" || flagSaveOnline == "y"
        for i = 1:length(folder)
            if ~exist(folder(i),"dir")
                mkdir(folder(i))
            end
            saveas(save_plot_histogram,folder(i)+"\histogramPlot")
            saveas(save_plotControl,folder(i)+"\controlPlot")
            saveas(save_plotApogee,folder(i)+"\apogeelPlot")
            saveas(save_plotTrajectory,folder(i)+"\TrajectoryPlot")
            saveas(save_thrust_apogee_probability,folder(i)+"\ApogeeProbabilityPlot")
            saveas(save_montecarlo_apogee_params,folder(i)+"\ApogeeMontecarloAnalysisPlot")
            saveas(save_t_shutdown_histogram,folder(i)+"\t_shutdownPlot")
            saveas(save_landing_ellipses,folder(i)+"\landingPositions")

            if exist('save_arb_deploy_histogram','var')
                saveas(save_arb_deploy_histogram,folder(i)+"\ARBdeployTimeHistogram")
            end
            if exist(' save_predicted_apogee','var')
                saveas( save_predicted_apogee,folder(i)+"\PredictedApogee")
            end
            saveas(save_apogee_histogram,folder(i)+"\ApogeeTimeHistogram")
            if ~settings.wind.model && ~settings.wind.input
                saveas(save_apogee_3D,folder(i)+"\ApogeeWindThrust")
            end
            saveas(save_dynamic_pressure_and_forces,folder(i)+"\dynamicPressureAndForces")
           
            for j = 1:N_sim
                save_thrust{j} = rmfield(save_thrust{j},'Y'); % remove ode data to save space
            end
            save(folder(i)+"\saveThrust.mat","save_thrust","apogee","N_sim","settings","thrust_percentage") % add "save_thrust", > 2GB for 1000 sim
% 
%                         exportgraphics(save_plot_histogram,'report_images\mc_Histogram.pdf','ContentType','vector')
%                         exportgraphics(save_plotApogee,'report_images\mc_Apogees.pdf','ContentType','vector')
%                          exportgraphics(save_apogee_3D,'report_images\apogee_wind.pdf','ContentType','vector')
%                          
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
            fprintf(fid,'Initial Mach number at which the control algorithm starts: %.3f \n\n',settings.MachControl);
            fprintf(fid,'Other parameters specific of the simulation: \n');
            fprintf(fid,'Filter coefficient: %.3f \n', contSettings.filter_coeff);
            fprintf(fid,'Target for shutdown: %d \n',settings.z_final_MTR );
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
            fprintf(fid,'WIND \n\n ');
            fprintf(fid,'Wind model: %s \n',settings.windModel);
            if settings.windModel == "constant"
                fprintf(fid,'Wind model parameters: \n'); % inserisci tutti i parametri del vento
                fprintf(fid,'Wind Magnitude: 0-%d m/s\n',settings.wind.MagMax);
                fprintf(fid,'Wind minimum azimuth: %d [°] \n',rad2deg(settings.wind.AzMin));
                fprintf(fid,'Wind maximum azimuth: %d [°] \n',rad2deg(settings.wind.AzMax));
                fprintf(fid,'Wind minimum elevation: %d [°] \n', rad2deg(settings.wind.ElMin));
                fprintf(fid,'Wind maximum elevation: %d [°] \n\n\n',rad2deg(settings.wind.ElMax));
            else
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
        end
    end
end
