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
% close all; clear all; clc;

%% recall the first part of the MAIN script
% adds folders to the path and retrieves rocket, mission, simulation, etc
% data.

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));

configSimulator;
configControl;
configReferences;
matlab_graphics;


%% MONTECARLO SETTINGS
rng default
settings.montecarlo = true;

%% how many simulations
N_sim = 200; % set to at least 500
simulationType_thrust = "gaussian";  % "gaussian", "exterme"

%% stochastic parameters

switch simulationType_thrust
    
    case "gaussian"
        
        sigma_t = (1.20-1)/3;             % thrust_percentage standard deviation
        mu_t = 1;                         % thrust_percentage mean value
        
        thrust_percentage = normrnd(mu_t,sigma_t,N_sim,1);       %generate normally distributed values ( [0.8 1.20] = 3sigma) % serve il toolbox
        stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
        %%%
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
algorithm_vec = [ "interp"; "std0"; "std2s";"NoControl"]; % interpolation, PID no change, PID change every 2s

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

%% MONTECARLO 1 - THRUST

if run_Thrust == true


    % other parameters you want to set for the particular simulation:
    for MachControl = [0.8]
    settings.MachControl = MachControl; % MSA sets it at 0.8
    contSettings.N_forward = 2;
    contSettings.filter_coeff = 0.3; % 1 = no filter
    contSettings.interpType = 'sinusoidal'; % set if the interp algorithm does a linear or sinusoidal interpolation of the references
    contSettings.filterRatio = 2;
    contSettings.Zfilter = 2000; % starting point from which the coefficient is diminished.
    contSettings.deltaZfilter = 250; % every deltaZfilter the filter coefficient is diminished by a ratio of filterRatio

    %simulation
    for alg_index = [1, 2, 3] 
        algorithm = algorithm_vec(alg_index);

        %save arrays
        save_thrust = cell(size(stoch.thrust,1),1);
        apogee.thrust = [];

        parfor i = 1:N_sim
            settings_mont = settings;
            contSettings_mont = contSettings;
            reference_mont = reference;

            settings_mont.motor.expThrust = stoch.thrust(i,:);                      % initialize the thrust vector of the current simulation (parfor purposes)
            settings_mont.motor.expTime = stoch.expThrust(i,:);                     % initialize the time vector for thrust of the current simulation (parfor purposes)
            settings_mont.tb = stoch.expThrust(i,end);                              % initialize the burning time of the current simulation (parfor purposes)

            settings_mont.wind.model = false;
            settings_mont.wind.input = false;

            % set the wind parameters
            settings_mont.wind.uw = stoch.wind.uw(i);
            settings_mont.wind.vw = stoch.wind.vw(i);
            settings_mont.wind.ww = stoch.wind.ww(i);
            settings_mont.wind.Az = stoch.wind.Az(i);
            settings_mont.wind.El = stoch.wind.El(i);

            if displayIter == true
                fprintf("simulation = " + num2str(i) + " of " + num2str(N_sim) + ", algorithm: " + algorithm +"\n");
            end
            switch algorithm
                case "interp"
                    contSettings_mont.filter_coeff = 1;
                    [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams,ap_ref,qdyn] = interp_run_control(settings_mont,contSettings_mont);
                case "std0"
                    contSettings_mont.z_trajChoice = 500;  % when time of flight is grater than 500s (never) change reference trajectory
                    [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams,ap_ref,qdyn] = std_run_control(settings_mont,contSettings_mont);
                case "std2s"
                    contSettings_mont.z_trajChoice = 3; % from 3s after lift off it's possible to change reference trajectory
                    [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams,ap_ref,qdyn] = std_run_control(settings_mont,contSettings_mont);
                case "NoControl"
                    settings_mont.control = false;
                    [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams,ap_ref,qdyn] = interp_run_control(settings_mont,contSettings_mont);
            end
            
            save_thrust{i}.time = Tf;
            save_thrust{i}.control = Yf(:,17);
            save_thrust{i}.position = Yf(:,1:3);
            save_thrust{i}.speed = Yf(:,4:6); 
            save_thrust{i}.windParams = windParams;
            save_thrust{i}.thrust_percentage = thrust_percentage(i);
            save_thrust{i}.qdyn = qdyn;
            save_thrust{i}.ap_ref = ap_ref;
            save_thrust{i}.paroutFlight = data_flight;
        end

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
            % wind magnitude
            wind_Mag(i) = norm([save_thrust{i}.windParams(1), save_thrust{i}.windParams(2), save_thrust{i}.windParams(3)]);
            % wind azimuth
            wind_az(i) = save_thrust{i}.windParams(4);
            %wind elevation
            wind_el(i) = save_thrust{i}.windParams(5);

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

        apogee.accuracy = N_ApogeeWithinTarget/N_sim*100; % percentage, so *100

        %% PLOT CONTROL
        save_thrust_plotControl = figure;
        for i = floor(linspace(1,N_sim,5))
            plot(save_thrust{i}.time,save_thrust{i}.control)
            hold on;
            grid on;
        end
        title('Control action')
        xlabel('Time [s]')
        ylabel('Servo angle [\alpha]')
        legend(algorithm);


        
        %% PLOT APOGEE 2D
        save_thrust_plotApogee = figure;
        for i = 1:N_sim
            plot(thrust_percentage(i),apogee.thrust(i),'*')
            hold on;
            grid on;
        end
        yline(settings.z_final-50,'r--')
        yline(settings.z_final+50,'r--')
        title('Apogee w.r.t. thrust')
        xlabel('Thrust percentage w.r.t. nominal')
        ylabel('Apogee [m]')
        xlim([min(thrust_percentage)-0.01,max(thrust_percentage)+0.01])
        ylim([settings.z_final-200,settings.z_final+200])
        text(1.1,settings.z_final + 100,"target apogee: "+num2str(settings.z_final))
        legend(algorithm);
        

        %% PLOT TRAJECTORY
        
        save_thrust_plotTrajectory = figure;
        for i = 1:size(save_thrust,1)
            plot3(save_thrust{i}.position(:,1),save_thrust{i}.position(:,2),-save_thrust{i}.position(:,3));
            hold on;
            grid on;
        end
        title('Trajectories')
        xlabel('x [m]')
        ylabel('y [m]')
        zlabel('z [m]')
        legend(algorithm);

        %% PLOT VELOCITIES
        
        save_thrust_plotVelocity = figure;
        for i = 1:size(save_thrust,1)
            plot(save_thrust{i}.time,save_thrust{i}.speed(:,1));
            hold on;
            plot(save_thrust{i}.time,save_thrust{i}.speed(:,2));
            plot(save_thrust{i}.time,save_thrust{i}.speed(:,3));
            grid on;
        end
        title('Velocities')
        xlabel('Vx_b [m/s]')
        ylabel('Vy_b [m/s]')
        zlabel('Vz_b [m/s]')
        legend(algorithm);

        %% PLOT APOGEE 3D

        save_thrust_apogee_3D = figure;
        %%%%%%%%%% wind magnitude - thrust - apogee
        subplot(2,2,1)
        hold on
        grid on
        plot3(wind_Mag,thrust_percentage*100,apogee.thrust','*')
        xlabel('Wind magnitude [m/s]')
        ylabel('Thrust percentage')
        zlabel('Apogee')
        zlim([settings.z_final-200,settings.z_final+200])
        view(30,20)
        text(min(wind_Mag),110,max(apogee.thrust) + 70,"target apogee: "+num2str(settings.z_final))
        legend(algorithm);
        %%%%%%%%%%% wind azimuth - thrust - apogee
        subplot(2,2,2)
        hold on
        grid on
        plot3(rad2deg(wind_az),thrust_percentage*100,apogee.thrust','*')
        xlabel('Wind azimuth [°]')
        ylabel('Thrust percentage')
        zlabel('Apogee')
        zlim([settings.z_final-200,settings.z_final+200])
        view(30,20)
        legend(algorithm);
        %%%%%%%%%%%% wind elevation - thrust - apogee
        subplot(2,2,3)
        hold on
        grid on
        plot3(rad2deg(wind_el),thrust_percentage*100,apogee.thrust','*')
        xlabel('Wind elevation [°]')
        ylabel('Thrust percentage [%]')
        zlabel('Apogee')
        zlim([settings.z_final-200,settings.z_final+200])
        view(30,20)
        legend(algorithm);
        %%%%%
        subplot(2,2,4)
        hold on
        grid on
        plot3(wind_el,wind_az,apogee.thrust','*')
        xlabel('Wind elevation [°]')
        ylabel('Wind azimuth [°]')
        zlabel('Apogee')
        zlim([settings.z_final-200,settings.z_final+200])
        view(30,20)
        legend(algorithm);
        %safe ellipses?
        %safe ellipses?


        %% PLOT PROBABILITY FUNCTION
        if N_sim>1
        save_thrust_apogee_probability = figure;
        pd = fitdist(apogee.thrust','Normal');    % create normal distribution object to compute mu and sigma
        % probability to reach an apogee between 2950 and 3050
        p = normcdf([settings.z_final-50, settings.z_final+50],apogee.thrust_mean,apogee.thrust_std);
        apogee.accuracy_gaussian =( p(2) - p(1) )*100;
        x_values = linspace(settings.z_final-500,settings.z_final+500,1000);   % possible apogees
        
        y = pdf(pd,x_values);                  % array of values of the probability density function
        hold on; grid on;
        xlabel('Reached apogee','Interpreter','latex','FontSize',15,'FontWeight','bold')
        ylabel('Probability density','Interpreter','latex','FontSize',15,'FontWeight','bold')
        plot(x_values,y)
        xline(settings.z_final,'r--')
        xline(10000000000)
        legend('Apogee Gaussian distribution','Target',algorithm)
        xlim([min(x_values), max(x_values)])
        end
        
        %% PLOT MEAN
        save_thrust_apogee_mean = figure;
        mu = zeros(N_sim,1);
        sigma = zeros(N_sim,1);
        for i = 1:N_sim
            mu(i) = mean(apogee.thrust(1:i));
            sigma(i) = std(apogee.thrust(1:i));
        end
        hold on
        grid on
        plot(1:N_sim,mu)
        xlabel('Number of iterations')
        ylabel('Apogee mean value')


        %% PLOT STANDARD DEVIATION
        save_thrust_apogee_std = figure;
        hold on
        grid on
        plot(1:N_sim,sigma)
        xlabel('Number of iterations')
        ylabel('Apogee standard deviation')
        
        %% PLOT DYNAMIC PRESSURE
        save_dynamic_pressure_and_forces = figure;

        %%%%%%%%%%% time - dynamic pressure
        subplot(1,2,1)
        for i = floor(linspace(1,N_sim,5))
            plot(save_thrust{i}.time,save_thrust{i}.qdyn);
            grid on;
            hold on;
        end
        title('Dynamic Pressure')
        xlabel('Time [s]')
        ylabel('Dynamic Pressure [Pa]')

        %%%%%%%%%%% time - aerodynamic load
        subplot(1,2,2)
        for i = floor(linspace(1,N_sim,5))
            dS = 3*0.009564 * save_thrust{i}.control; 
            force = save_thrust{i}.qdyn .* dS;
            force_kg = force/9.81;
            plot(save_thrust{i}.time,force_kg);
            grid on;
            hold on;
        end
        title('Aerodynamic load')
        xlabel('Time [s]')
        ylabel('Total aerodynamic load on airbrakes [kg]')

       
        %% SAVE
        % save plots
        saveDate = string(datestr(date,29));
        folder = [];
        
        if flagSaveOffline == "yes"
            switch  settings.mission 
                
                case 'Pyxis_Portugal_October_2022'
                    folder = [folder ; "MontecarloResults\"+settings.mission+"\"+algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline
                
                case 'Pyxis_Roccaraso_September_2022'
                    folder = [folder ; "MontecarloResults\"+settings.mission+"\z_f_"+settings.z_final+"\"+algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % offline
            end
        
        end
        if flagSaveOnline == "yes"
            if computer == "Marco" || computer == "marco"
                switch  settings.mission 
                
                    case 'Pyxis_Portugal_October_2022'
                        folder = [folder ; "C:\Users\marco\OneDrive - Politecnico di Milano\SKYWARD\task 1 - motor model\montecarlo e tuning\"+settings.mission+"\"+algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % online
                    
                    case 'Pyxis_Roccaraso_September_2022'
                        folder = [folder ; "C:\Users\marco\OneDrive - Politecnico di Milano\SKYWARD\task 1 - motor model\montecarlo e tuning\"+settings.mission+"\z_f_"+settings.z_final+"\"+algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+simulationType_thrust+"_"+saveDate]; % online
                end
            end
        end
        
        if flagSaveOffline == "yes" || flagSaveOnline == "yes"
            for i = 1:length(folder)
            mkdir(folder(i))
            saveas(save_thrust_plotControl,folder(i)+"\controlPlot")
            saveas(save_thrust_plotApogee,folder(i)+"\apogeelPlot")
            saveas(save_thrust_plotTrajectory,folder(i)+"\TrajectoryPlot")
            saveas(save_thrust_apogee_probability,folder(i)+"\ApogeeProbabilityPlot")
            saveas(save_thrust_apogee_mean,folder(i)+"\ApogeeMeanOverNsimPlot")
            saveas(save_thrust_apogee_std,folder(i)+"\ApogeeStdOverNsimPlot")
            saveas(save_thrust_apogee_3D,folder(i)+"\ApogeeWindThrust")
            saveas(save_dynamic_pressure_and_forces,folder(i)+"\dynamicPressureAndForces")
            save(folder(i)+"\saveThrust.mat","save_thrust","apogee")
       

        % Save results.txt
            fid = fopen( folder(i)+"\"+algorithm+"Results"+saveDate+".txt", 'wt' );  % CAMBIA IL NOME
            fprintf(fid,'Algorithm: %s \n',algorithm );
            fprintf(fid,'Number of simulations: %d \n \n',N_sim); % Cambia n_sim
            fprintf(fid,'Parameters: \n');
            fprintf(fid,'Target apogee: %d \n',settings.z_final);
            fprintf(fid,'Thrust: +-20%% at 3*sigma, total impulse constant \n');
            fprintf(fid,'Control frequency: %d Hz \n',settings.frequencies.controlFrequency);
            fprintf(fid,'Initial Mach number at which the control algorithm starts: %.3f \n',settings.MachControl);
            switch alg_index
                case 2
                    fprintf(fid,'P = %d \n',contSettings.Kp_1 );
                    fprintf(fid,'I = %d \n',contSettings.Ki_1 );
                    fprintf(fid,'Never change reference trajectory \n\n' );
                case 3
                    fprintf(fid,'P = %d \n',contSettings.Kp_1 );
                    fprintf(fid,'I = %d \n',contSettings.Ki_1 );
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
            if algorithm == "interp"
                fprintf(fid,'N_forward: %d \n', contSettings.N_forward);
                fprintf(fid,'Delta Z (reference): %d \n',reference.deltaZ);
                fprintf(fid,'Filter diminished every: %d \n', contSettings.deltaZfilter);
                fprintf(fid,'Filter diminished by ratio: %d \n', contSettings.filterRatio);
                fprintf(fid,'Filter diminishing starts at: %d m \n', contSettings.Zfilter);
                fprintf(fid,'Interpolation type: %s \n', contSettings.interpType);
            end
            fclose(fid);
            end
        end
    end
    end
end














%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





























%% Notify the end of simulation:
load gong.mat;
sound(y);
