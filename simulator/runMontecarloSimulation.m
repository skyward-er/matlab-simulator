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

addpath(genpath(currentPath));

configSimulator;
configControl;
configReferences;



%% MONTECARLO SETTINGS
rng default
settings.montecarlo = true;

%% how many simulations
N_sim = 400; % set to at least 500

%% stochastic parameters
sigma_t = (1.20-1)/3;             % thrust_percentage standard deviation
mu_t = 1;                         % thrust_percentage mean value
thrust_percentage= normrnd(mu_t,sigma_t,N_sim,1);       %generate normally distributed values ( [0.8 1.20] = 3sigma) % serve il toolbox
tauServo_percentage = linspace(0.8,1.2,N_sim);          % defined for plot purposes

%%%
stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
impulse_uncertainty = normrnd(1,0.05/3,N_sim,1);
stoch.expThrust = diag(impulse_uncertainty)*((1./thrust_percentage) * settings.motor.expTime);          % burning time - same notation as thrust here

stoch.tauServo = tauServo_percentage * settings.servo.tau;                  % servo motor time constant
stoch.controlFrequency = [1, 2, 5, 10];                                     % control frequency (sets how fast the control input reference is given to the servo)
stoch.MachControl = 0.6:0.05:1;                                             % Mach at which the air brakes can be deployed

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

contSettings.deltaZ_change = 2;                                         % change reference every 2seconds

%% wind parameters
settings.wind.MagMin = 0;                                               % [m/s] Minimum Wind Magnitude
settings.wind.MagMax = 10;                                               % [m/s] Maximum Wind Magnitude
settings.wind.ElMin  = - deg2rad(45);
settings.wind.ElMax  = + deg2rad(45);
settings.wind.AzMin  = - deg2rad(180);
settings.wind.AzMax  = + deg2rad(180);


[stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El] = windConstGeneratorMontecarlo(settings.wind,N_sim);
% fare shuffle - edit: sono già non correlate, non serve in teoria

%% save arrays
save_Mach = cell(size(stoch.MachControl,1),1);

% algorithms
algorithm_vec = [ "interp"; "std0"; "std2s";"NoControl"]; % interpolation, PID no change, PID change every 2s

%% which montecarlo do you want to run?

run_Thrust = true;
run_Filter = false;


%% do you want to save the results?

flagSave = input('do you want to save the results? ("yes" or "no"): ','s');
% flagSave = "yes";

displayIter = true; % set to false if you don't want to see the iteration number (maybe if you want to run Montecarlos on hpe)

%% MONTECARLO 1 - THRUST

if run_Thrust == true


    % other parameters you want to set for the particular simulation:
    settings.MachControl = 0.8; % MSA sets it at 0.8
    contSettings.N_forward = 2;
    contSettings.filter_coeff = 0.3; % 1 = no filter
    contSettings.interpType = 'sinusoidal'; % set if the interp algorithm does a linear or sinusoidal interpolation of the references
    contSettings.filterRatio = 2;
    contSettings.Zfilter = 2000; % starting point from which the coefficient is diminished.
    contSettings.deltaZfilter = 250; % every deltaZfilter the filter coefficient is diminished by a ratio of filterRatio



    %simulation
    for alg_index = 1%:length(algorithm_vec)
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
        end

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
            apogee.thrust(i) = max(-save_thrust{i}.position(:,3));
            plot(thrust_percentage(i),apogee.thrust(i),'*')
            hold on;
            grid on;
        end
        yline(2950,'r--')
        yline(3050,'r--')
        title('Apogee w.r.t. thrust')
        xlabel('Thrust percentage w.r.t. nominal')
        ylabel('Apogee [m]')
        xlim([min(thrust_percentage)-0.01,max(thrust_percentage)+0.01])
        ylim([2800,3200])
        legend(algorithm);


        %% PLOT TRAJECTORY
        apogee.thrust_mean = mean(apogee.thrust);
        apogee.thrust_variance = std(apogee.thrust);

        save_thrust_plotTrajectory = figure;
        for i = 1:size(save_thrust,1)
            plot3(save_thrust{i}.position(:,1),save_thrust{i}.position(:,2),-save_thrust{i}.position(:,3));
            hold on;
            grid on;
        end
        title('Trajectories')
        xlabel('x')
        ylabel('y')
        zlabel('z')
        legend(algorithm);


        %% PLOT APOGEE 3D

        save_thrust_apogee_3D = figure;
        subplot(2,2,1)
        hold on
        grid on
        wind_Mag = zeros(N_sim,1);
        for i = 1:N_sim
            wind_Mag(i) = norm([save_thrust{i}.windParams(1), save_thrust{i}.windParams(2), save_thrust{i}.windParams(3)]);
        end
        plot3(wind_Mag,thrust_percentage*100,apogee.thrust','*')
        xlabel('Wind magnitude [m/s]')
        ylabel('Thrust percentage')
        zlabel('Apogee')
        zlim([2800,3200])
        view(30,20)
        legend(algorithm);
        %%%%%%%%%%%
        subplot(2,2,2)
        hold on
        grid on
        wind_az = zeros(N_sim,1);
        for i = 1:N_sim
            wind_az(i) = rad2deg(save_thrust{i}.windParams(4));
        end
        plot3(wind_az,thrust_percentage*100,apogee.thrust','*')
        xlabel('Wind azimuth [°]')
        ylabel('Thrust percentage')
        zlabel('Apogee')
        zlim([2800,3200])
        view(30,20)
        legend(algorithm);
        %%%%%%%%%%%%
        subplot(2,2,3)
        hold on
        grid on
        wind_el = zeros(N_sim,1);
        for i = 1:N_sim
            wind_el(i) = rad2deg(save_thrust{i}.windParams(5)); % in degrees
        end
        plot3(wind_el,thrust_percentage*100,apogee.thrust','*')
        xlabel('Wind elevation [°]')
        ylabel('Thrust percentage [%]')
        zlabel('Apogee')
        zlim([2800,3200])
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
        zlim([2800,3200])
        view(30,20)
        legend(algorithm);
        %safe ellipses?
        %safe ellipses?


        %% PLOT PROBABILITY FUNCTION
        if N_sim>1
        save_thrust_apogee_probability = figure;
        pd = fitdist(apogee.thrust','Normal');    % create normal distribution object to compute mu and sigma
        % probability to reach an apogee between 2950 and 3050
        p = normcdf([2950 3050],apogee.thrust_mean,apogee.thrust_variance);
        accuracy =( p(2) - p(1) )*100;
        
        x_values = linspace(2000,4000,1000);   % possible apogees
        
        y = pdf(pd,x_values);                  % array of values of the probability density function
        hold on; grid on;
        xlabel('Reached apogee','Interpreter','latex','FontSize',15,'FontWeight','bold')
        ylabel('Probability density','Interpreter','latex','FontSize',15,'FontWeight','bold')
        plot(x_values,y)
        xline(3000,'r--')
        xline(10000000000)
        legend('Apogee Gaussian distribution','Target',algorithm)
        xlim([2000 4000])
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
        subplot(1,2,1)
        for i = floor(linspace(1,N_sim,5))
            plot(save_thrust{i}.time,save_thrust{i}.qdyn);
            grid on;
            hold on;
        end
        title('Dynamic Pressure')
        xlabel('Time [s]')
        ylabel('Dynamic Pressure [Pa]')

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
        folder = "MontecarloResults\Thrust\"+algorithm+"\"+num2str(N_sim)+"sim_Mach"+num2str(100*settings.MachControl)+"_"+saveDate;
        if flagSave == "yes"
            mkdir(folder)
            saveas(save_thrust_plotControl,folder+"\controlPlot")
            saveas(save_thrust_plotApogee,folder+"\apogeelPlot")
            saveas(save_thrust_plotTrajectory,folder+"\TrajectoryPlot")
            saveas(save_thrust_apogee_probability,folder+"\ApogeeProbabilityPlot")
            saveas(save_thrust_apogee_mean,folder+"\ApogeeMeanOverNsimPlot")
            saveas(save_thrust_apogee_std,folder+"\ApogeeStdOverNsimPlot")
            saveas(save_thrust_apogee_3D,folder+"\ApogeeWindThrust")
            saveas(save_dynamic_pressure_and_forces,folder+"\dynamicPressureAndForces")
            save(folder+"\saveThrust.mat","save_thrust","apogee")
       

        % Save results.txt
            fid = fopen( folder+"\"+algorithm+"Results"+saveDate+".txt", 'wt' );  % CAMBIA IL NOME
            fprintf(fid,'Algorithm: %s \n',algorithm );
            fprintf(fid,'Number of simulations: %d \n \n',N_sim); % Cambia n_sim
            fprintf(fid,'Parameters: \n');
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
            fprintf(fid,'Wind minimum azimuth: %d degrees \n',settings.wind.AzMin);
            fprintf(fid,'Wind maximum azimuth: %d degrees \n',settings.wind.AzMax);
            fprintf(fid,'Wind minimum elevation: %d degrees \n', settings.wind.ElMin);
            fprintf(fid,'Wind maximum elevation: %d degrees \n\n',settings.wind.ElMax);
            fprintf(fid,'Results: \n');
            fprintf(fid,'Max apogee: %.2f \n',max(apogee.thrust));
            fprintf(fid,'Min apogee: %.2f \n',min(apogee.thrust));
            fprintf(fid,'Mean apogee: %.2f \n',apogee.thrust_mean);
            fprintf(fid,'Apogee standard deviation 3sigma: %.4f \n',3*apogee.thrust_variance);
            fprintf(fid,'Apogees within +-50m from target: %.2f %% \n\n\n',accuracy);
            fprintf(fid,'Other parameters specific of the simulation: \n\n');
            fprintf(fid,'N_forward: %d \n', contSettings.N_forward);
            fprintf(fid,'Delta Z (reference): %d \n',reference.deltaZ);
            fprintf(fid,'Filter coefficient: %.3f \n', contSettings.filter_coeff);
            fprintf(fid,'Interpolation type: %s \n', contSettings.interpType);
            fprintf(fid,'Filter diminished every: %d \n', contSettings.deltaZfilter);
            fprintf(fid,'Filter diminished by ratio: %d \n', contSettings.filterRatio);
            fprintf(fid,'Filter diminishing starts at: %d m \n', contSettings.Zfilter);
            fclose(fid);
        end
    end

end














%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5





























%% Notify the end of simulation:
load gong.mat;
sound(y);
