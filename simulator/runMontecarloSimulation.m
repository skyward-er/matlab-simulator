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
msaToolkitURL = 'https://github.com/skyward-er/msa-toolkit';
localRepoPath = '../data/msa-toolkit';
status = checkLastCommit(msaToolkitURL, localRepoPath, pwd);
% submoduleAdvice(status, msaToolkitURL, localRepoPath, pwd);

%% CONFIGs
config;
matlab_graphics;


%% MONTECARLO SETTINGS
rng default
settings.montecarlo = true;

%% how many simulations
N_sim = 500; % set to at least 500
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
        impulse_uncertainty = normrnd(1,0.05/3,N_sim,1);
        stoch.expThrust = diag(impulse_uncertainty)*((1./thrust_percentage) * settings.motor.expTime);          % burning time - same notation as thrust here

        %%% check on thrust - all of them must have the same total impulse
        for i = 1:size(stoch.thrust,1)
            %     plot(stoch.expThrust(i,:),stoch.thrust(i,:))
            %     hold on;
            %     grid on;
            %     legend
            %%% check on total impulse: trapezoidal integration:
            for jj = 1:length(stoch.thrust(i,:))-1
                deltaT = stoch.expThrust(i,jj+1)-stoch.expThrust(i,jj);
                I(jj) = (stoch.thrust(i,jj+1)+stoch.thrust(i,jj))*deltaT/2;
            end
            Itot(i) = sum(I);
        end
        if Itot - Itot(1)>0.0001
            warning('The thrust vector does not return equal total impulse for each simulation')
        end
        %%% wind parameters
        settings.wind.MagMin = 0;                                               % [m/s] Minimum Wind Magnitude
        settings.wind.MagMax = 10;                                               % [m/s] Maximum Wind Magnitude
        settings.wind.ElMin  = - deg2rad(45);
        settings.wind.ElMax  = + deg2rad(45);
        settings.wind.AzMin  = - deg2rad(180);
        settings.wind.AzMax  = + deg2rad(180);

        [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust);

    case "extreme"


        thrust_percentage = [0.8;1.2]; % this is overwritten in the next step, but it sets the values to retrieve in the parameter generation

        %%% wind parameters
        [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ,thrust_percentage, N_sim] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust,thrust_percentage);

        stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
        %%%
        stoch.expThrust = (1./thrust_percentage) * settings.motor.expTime;          % burning time - same notation as thrust here
end


contSettings.deltaZ_change = 2;                         % change reference every 2seconds



%% save arrays

% algorithms
algorithm_vec = [ "interp"; "PID_2021"; "shooting" ;"NoControl"]; % interpolation, PID change every 2s, shooting, no control

%% which montecarlo do you want to run?

run_Thrust = true;
run_Filter = false;


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

contSettings.N_forward = 0;
contSettings.filter_coeff = 0.3; % 1 = no filter
contSettings.interpType = 'linear'; % set if the interp algorithm does a linear or sinusoidal interpolation of the references
contSettings.filterRatio = 2;
contSettings.Zfilter = 2000; % starting point from which the coefficient is diminished.
contSettings.deltaZfilter = 250; % every deltaZfilter the filter coefficient is diminished by a ratio of filterRatio

settings.wind.model = false;
settings.wind.input = true; % occhio che per ora non è settato esternamente con le montecarlo, quindi se questo viene settato a true abbiamo solamente incertezza sulla spinta.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONFIG%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

settings_mont_init = struct('x',[]);

% start simulation
for alg_index = 2

    contSettings.algorithm = algorithm_vec(alg_index);

    %save arrays
    save_thrust = cell(size(stoch.thrust,1),1);
    apogee.thrust = [];


    parfor i = 1:N_sim
        settings_mont = settings_mont_init;
%         contSettings_mont = contSettings;
%         reference_mont = reference;

        settings_mont.motor.expThrust = stoch.thrust(i,:);                      % initialize the thrust vector of the current simulation (parfor purposes)
        settings_mont.motor.expTime = stoch.expThrust(i,:);                     % initialize the time vector for thrust of the current simulation (parfor purposes)
        settings_mont.tb = stoch.expThrust(i,end);                              % initialize the burning time of the current simulation (parfor purposes)

       
        % set the wind parameters
        settings_mont.wind.uw = stoch.wind.uw(i);
        settings_mont.wind.vw = stoch.wind.vw(i);
        settings_mont.wind.ww = stoch.wind.ww(i);
        settings_mont.wind.Az = stoch.wind.Az(i);
        settings_mont.wind.El = stoch.wind.El(i);

        if displayIter == true
            fprintf("simulation = " + num2str(i) + " of " + num2str(N_sim) + ", algorithm: " + contSettings.algorithm +"\n");
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [simOutput] = std_runV2(settings,contSettings,settings_mont);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%STD_RUN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        save_thrust{i} = simOutput;
    end
    
return
    %% RETRIEVE INTERESING PARAMETERS:

    N_sim = length(save_thrust); % recall the number, useful if you have to just plot the saved results and you don't know the N_sim you gave

    wind_Mag = zeros(N_sim,1);
    wind_el = zeros(N_sim,1);
    wind_az = zeros(N_sim,1);

    N_ApogeeWithinTarget = 0;

    for i = 1:N_sim
        % apogee
        apogee.thrust(i) = max(-save_thrust{i}.position(:,3));
        % radius of apogee (horizontal) from the initial point
        apogee.radius(i) = sqrt(save_thrust{i}.position(end,1)^2+save_thrust{i}.position(end,2)^2);
        % horizontal speed at apogee
        apogee.horizontalSpeed(i) = norm(save_thrust{i}.speed(end,1:3)); % in theory this is the body frame, but as the last point is the apogee we should have only  horizontal velocity, so all the components must be taken
        if ~settings.wind.model && ~settings.wind.input
        % wind magnitude
        wind_Mag(i) = norm([save_thrust{i}.windParams(1), save_thrust{i}.windParams(2), save_thrust{i}.windParams(3)]);
        % wind azimuth
        wind_az(i) = save_thrust{i}.windParams(4);
        %wind elevation
        wind_el(i) = save_thrust{i}.windParams(5);
        end
        if settings.wind.input
            
        end
        % within +-50 meters target apogees:
        if abs(apogee.thrust(i) - settings.z_final)<50
            N_ApogeeWithinTarget = N_ApogeeWithinTarget +1; % save how many apogees sit in the +-50 m from target
        end
    end

    apogee.thrust_mean = mean(apogee.thrust);
    apogee.thrust_std = std(apogee.thrust);

    apogee.radius_mean = mean(apogee.radius);
    apogee.radius_std = std(apogee.radius);
    apogee.radius_max = max(apogee.radius);
    apogee.radius_min = min(apogee.radius);

    apogee.horizontalSpeed_mean = mean(apogee.horizontalSpeed);
    apogee.horizontalSpeed_std = std(apogee.horizontalSpeed);
    apogee.horizontalSpeed_max = max(apogee.horizontalSpeed);
    apogee.horizontalSpeed_min = min(apogee.horizontalSpeed);

    apogee.accuracy = N_ApogeeWithinTarget/N_sim*100; % percentage, so*100

    %% PLOTS
    
    plotsMontecarlo;


    %% SAVE FUNCTION

    %% SAVE
    % save plots
    saveDate = string(datestr(date,29));
    folder = [];

    if flagSaveOffline == "yes"
        switch  settings.mission

            case 'Pyxis_Portugal_October_2022'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline

            case 'Pyxis_Roccaraso_September_2022'
                folder = [folder ; "MontecarloResults\"+settings.mission+"\z_f_"+settings.z_final+"\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline
        end

    end
    if flagSaveOnline == "yes"
        if computer == "Marco" || computer == "marco"
            switch  settings.mission

                case 'Pyxis_Portugal_October_2022'
                    folder = [folder ; "C:\Users\marco\OneDrive - Politecnico di Milano\SKYWARD\task 1 - motor model\montecarlo e tuning\"+settings.mission+"\wind_input\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % online

                case 'Pyxis_Roccaraso_September_2022'
                    folder = [folder ; "C:\Users\marco\OneDrive - Politecnico di Milano\SKYWARD\task 1 - motor model\montecarlo e tuning\"+settings.mission+"\z_f_"+settings.z_final+"\wind_input\"+contSettings.algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % online
            end
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
            if ~settings.wind.model && ~settings.wind.input
            saveas(save_apogee_3D,folder(i)+"\ApogeeWindThrust")
            end
            saveas(save_dynamic_pressure_and_forces,folder(i)+"\dynamicPressureAndForces")
            save(folder(i)+"\saveThrust.mat","save_thrust","apogee","N_sim","settings","thrust_percentage")

            exportgraphics(save_plot_histogram,'report_images\mc_Histogram.pdf','ContentType','vector')
            exportgraphics(save_plotApogee,'report_images\mc_Apogees.pdf','ContentType','vector')


            % Save results.txt
            fid = fopen( folder(i)+"\"+contSettings.algorithm+"Results"+saveDate+".txt", 'wt' );  % CAMBIA IL NOME
            fprintf(fid,'Algorithm: %s \n',contSettings.algorithm );
            fprintf(fid,'Number of simulations: %d \n \n',N_sim); % Cambia n_sim
            fprintf(fid,'Parameters: \n');
            fprintf(fid,'Target apogee: %d \n',settings.z_final);
            fprintf(fid,'Thrust: +-20%% at 3*sigma, total impulse constant \n');
            fprintf(fid,'Control frequency: %d Hz \n',settings.frequencies.controlFrequency);
            fprintf(fid,'Initial Mach number at which the control algorithm starts: %.3f \n',settings.MachControl);
            switch alg_index
                case 2
                    fprintf(fid,'P = %d \n',contSettings.Kp );
                    fprintf(fid,'I = %d \n',contSettings.Ki );
                    fprintf(fid,'Never change reference trajectory \n\n' );
                case 3
                    fprintf(fid,'P = %d \n',contSettings.Kp );
                    fprintf(fid,'I = %d \n',contSettings.Ki );
                    fprintf(fid,'Change reference trajectory every %d seconds \n\n',contSettings.deltaZ_change );
            end
            fprintf(fid,'Wind model parameters: \n'); % inserisci tutti i parametri del vento
            fprintf(fid,'Wind Magnitude: 0-%d m/s\n',settings.wind.MagMax);
            fprintf(fid,'Wind minimum azimuth: %d [°] \n',rad2deg(settings.wind.AzMin));
            fprintf(fid,'Wind maximum azimuth: %d [°] \n',rad2deg(settings.wind.AzMax));
            fprintf(fid,'Wind minimum elevation: %d [°] \n', rad2deg(settings.wind.ElMin));
            fprintf(fid,'Wind maximum elevation: %d [°] \n\n\n',rad2deg(settings.wind.ElMax));
            %%%%%%%%%%%%%%%
            fprintf(fid,'Results: \n');
            fprintf(fid,'Max apogee: %.2f \n',max(apogee.thrust));
            fprintf(fid,'Min apogee: %.2f \n',min(apogee.thrust));
            fprintf(fid,'Mean apogee: %.2f \n',apogee.thrust_mean);
            fprintf(fid,'Apogee standard deviation 3sigma: %.4f \n',3*apogee.thrust_std);
            fprintf(fid,'Apogees within +-50m from target (gaussian): %.2f %% \n',apogee.accuracy_gaussian);
            fprintf(fid,'Apogees within +-50m from target (ratio): %.2f %% \n\n',apogee.accuracy);
            fprintf(fid,'Apogees horizontal distance from origin mean : %.2f [m] \n',apogee.radius_mean);
            fprintf(fid,'Apogees horizontal distance from origin std : %.2f [m] \n\n',apogee.radius_std);
            fprintf(fid,'Apogees horizontal speed mean : %.2f [m/s] \n',apogee.horizontalSpeed_mean);
            fprintf(fid,'Apogees horizontal speed std : %.2f [m/s] \n\n\n',apogee.horizontalSpeed_std);
            %%%%%%%%%%%%%%%
            fprintf(fid,'Other parameters specific of the simulation: \n\n');
            fprintf(fid,'Filter coefficient: %.3f \n', contSettings.filter_coeff);
            if contSettings.algorithm == "interp"
                fprintf(fid,'N_forward: %d \n', contSettings.N_forward);
                fprintf(fid,'Delta Z (reference): %d \n',contSettings.reference.deltaZ);
                fprintf(fid,'Filter diminished every: %d \n', contSettings.deltaTfilter);
                fprintf(fid,'Filter diminished by ratio: %d \n', contSettings.filterRatio);
                fprintf(fid,'Filter diminishing starts at: %d m \n', contSettings.Tfilter);
                fprintf(fid,'Interpolation type: %s \n', contSettings.interpType);
            end
            fclose(fid);
        end
    end
end
