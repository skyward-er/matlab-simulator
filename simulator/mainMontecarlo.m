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
close all; clear all; clc;

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
N_sim = 400; % set to at least 500
simulationType_thrust = "gaussian";  % "gaussian", "exterme"

%% stochastic parameters

switch simulationType_thrust

    case "gaussian"

        sigma_t = (1.20-1)/3;             % thrust_percentage standard deviation
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


        thrust_percentage = [0.8;1.2]; % this is overwritten in the next step, but it sets the values to retrieve in the parameter generation

        %%% wind parameters
        [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ,thrust_percentage, N_sim] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust,thrust_percentage);

        stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
        %%%
        stoch.expThrust = (1./thrust_percentage) * settings.motor.expTime;          % burning time - same notation as thrust here
end

%% save arrays

% algorithms
algorithm_vec = {'interp';'NoControl';'engine';'complete'; 'PID_2021'; 'shooting'}; % interpolation, no control, engine shutdown, engine+arb, PID change every 2s, shooting

%% do you want to save the results?

flagSaveOffline = input('Do you want to save the results offline? ("yes" or "no"): ','s');
flagSaveOnline = input('Do you want to save the resuts online? (oneDrive) ("yes" or "no"): ','s');

if flagSaveOnline == "yes"
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
    apogee.thrust = [];
    N_ApogeeWithinTarget = 0;
    N_ApogeeWithinTarget_50 = 0;
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
        settings_mont.tb = stoch.expThrust(i,end);                              % initialize the burning time of the current simulation (parfor purposes)
        settings_mont.State.xcgTime = stoch.State.xcgTime(:,i);                      % initialize the baricenter position time vector

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
            fprintf("simulation = " + num2str(i) + " of " + num2str(N_sim) + ", algorithm: " + contSettings.algorithm +"\n");
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
        apogee.thrust(i) = save_thrust{i}.apogee_coordinates(3);

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
            %reached apogee time
            apogee.times(i) = save_thrust{i}.apogee_time;
        end

        % within +-10 meters target apogees:
        if abs(apogee.thrust(i) - settings.z_final)<=10
            N_ApogeeWithinTarget = N_ApogeeWithinTarget +1; % save how many apogees sit in the +-50 m from target
        end
          % within +-50 meters target apogees:
          
        if abs(apogee.thrust(i) - settings.z_final)<=50
            N_ApogeeWithinTarget_50 = N_ApogeeWithinTarget_50 +1; % save how many apogees sit in the +-50 m from target
        end
          
    end


    apogee.thrust_mean = mean(apogee.thrust);
    apogee.thrust_std = std(apogee.thrust);

    t_shutdown.mean = mean(t_shutdown.value);
    t_shutdown.std = std(t_shutdown.value);

    apogee.radius_mean = mean(apogee.radius);
    apogee.radius_std = std(apogee.radius);
    apogee.radius_max = max(apogee.radius);
    apogee.radius_min = min(apogee.radius);

    apogee.horizontalSpeed_mean = mean(apogee.horizontalSpeed);
    apogee.horizontalSpeed_std = std(apogee.horizontalSpeed);
    apogee.horizontalSpeed_max = max(apogee.horizontalSpeed);
    apogee.horizontalSpeed_min = min(apogee.horizontalSpeed);

    apogee.accuracy = N_ApogeeWithinTarget/N_sim*100; % percentage, so*100
    apogee.accuracy_50 = N_ApogeeWithinTarget_50/N_sim*100; % percentage, so*100

    for i = 1:N_sim
        save_thrust{i}.mu = mean(apogee.thrust(1:i));
        save_thrust{i}.sigma = std(apogee.thrust(1:i));
    end

    %% PLOTS

    plotsMontecarlo;
    
    %% SAVE
    % save plots
    saveDate = string(datestr(date,29));
    folder = [];

    if flagSaveOffline == "yes"
        switch  settings.mission

            case 'Pyxis_Portugal_October_2022'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case 'Gemini_Portugal_October_2023'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case 'Pyxis_Roccaraso_September_2022'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\z_f_"+settings.z_final+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline
        end

    end
    if flagSaveOnline == "yes"
        if computer == "Marco" || computer == "marco"
            folder = [folder ; "C:\Users\marco\OneDrive - Politecnico di Milano\SKYWARD\AIR BRAKES\MONTECARLO E TUNING\"+settings.mission+"\wind_input\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % online
        end
    end

    if flagSaveOffline == "yes" || flagSaveOnline == "yes"
        for i = 1:length(folder)
            mkdir(folder(i))
            saveas(save_plot_histogram,folder(i)+"\histogramPlot")
            saveas(save_plotControl,folder(i)+"\controlPlot")
            saveas(save_plotApogee,folder(i)+"\apogeelPlot")
            saveas(save_plotTrajectory,folder(i)+"\TrajectoryPlot")
            saveas(save_thrust_apogee_probability,folder(i)+"\ApogeeProbabilityPlot")
            saveas(save_thrust_apogee_mean,folder(i)+"\ApogeeMeanOverNsimPlot")
            saveas(save_thrust_apogee_std,folder(i)+"\ApogeeStdOverNsimPlot")
            saveas(save_t_shutdown_histogram,folder(i)+"\t_shutdownPlot")

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

                        exportgraphics(save_plot_histogram,'report_images\mc_Histogram.pdf','ContentType','vector')
                        exportgraphics(save_plotApogee,'report_images\mc_Apogees.pdf','ContentType','vector')
                         exportgraphics(save_apogee_3D,'report_images\apogee_wind.pdf','ContentType','vector')
                         
            % Save results.txt
            fid = fopen( folder(i)+"\"+contSettings.algorithm+"Results"+saveDate+".txt", 'wt' );  % CAMBIA IL NOME
            fprintf(fid,'Algorithm: %s \n',contSettings.algorithm );
            fprintf(fid,'Number of simulations: %d \n \n',N_sim); % Cambia n_sim
            fprintf(fid,'Parameters: \n');
            fprintf(fid,'Target apogee: %d \n',settings.z_final);
            fprintf(fid,'Thrust: +-50%% at 3*sigma, total impulse constant \n');
            fprintf(fid,'Engine shut-down control frequency: %d Hz \n',settings.frequencies.controlFrequency);
            fprintf(fid,'Airbrakes control frequency: %d Hz \n',settings.frequencies.arbFrequency );
            fprintf(fid,'Initial Mach number at which the control algorithm starts: %.3f \n',settings.MachControl);
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
                fprintf(fid,'Ground wind Magnitude: 0-%d m/s\n',settings.wind.MagMax);
            end
            
            %%%%%%%%%%%%%%%
            fprintf(fid,'Results: \n');
            fprintf(fid,'Max apogee: %.2f \n',max(apogee.thrust));
            fprintf(fid,'Min apogee: %.2f \n',min(apogee.thrust));
            fprintf(fid,'Mean apogee: %.2f \n',apogee.thrust_mean);
            fprintf(fid,'Apogee standard deviation 3sigma: %.4f \n',3*apogee.thrust_std);
            fprintf(fid,'Apogees within +-10m from target (gaussian): %.2f %% \n',apogee.accuracy_gaussian);
            fprintf(fid,'Apogees within +-10m from target (ratio): %.2f %% \n\n',apogee.accuracy);
            fprintf(fid,'Apogees within +-50m from target (gaussian): %.2f %% \n',apogee.accuracy_gaussian_50);
            fprintf(fid,'Apogees within +-50m from target (ratio): %.2f %% \n\n',apogee.accuracy_50);
            fprintf(fid,'Apogees horizontal distance from origin mean : %.2f [m] \n',apogee.radius_mean);
            fprintf(fid,'Apogees horizontal distance from origin std : %.2f [m] \n\n',apogee.radius_std);
            fprintf(fid,'Apogees horizontal speed mean : %.2f [m/s] \n',apogee.horizontalSpeed_mean);
            fprintf(fid,'Apogees horizontal speed std : %.2f [m/s] \n\n',apogee.horizontalSpeed_std);
            fprintf(fid,'Mean shutdown time : %.3f [m/s] \n',t_shutdown.mean);
            fprintf(fid,'Standard deviation of shutdown time : %.3f [m/s] \n\n\n',t_shutdown.std);
            %%%%%%%%%%%%%%%
            fprintf(fid,'Other parameters specific of the simulation: \n\n');
            fprintf(fid,'Filter coefficient: %.3f \n', contSettings.filter_coeff);
            if contSettings.algorithm == "interp" || contSettings.algorithm == "complete" 
                fprintf(fid,'N_forward: %d \n', contSettings.N_forward);
                fprintf(fid,'Delta Z (reference): %d \n',contSettings.reference.deltaZ);
                fprintf(fid,'Filter diminished every: %d \n', contSettings.deltaTfilter);
                fprintf(fid,'Filter diminished by ratio: %d \n', contSettings.filterRatio);
                fprintf(fid,'Filter diminishing starts at: %d m \n', contSettings.Tfilter);
                fprintf(fid,'Interpolation type: %s \n', contSettings.interpType);

                fprintf(fid,'Correction with ground wind: no \n');
            end
            fclose(fid);
        end
    end
end
