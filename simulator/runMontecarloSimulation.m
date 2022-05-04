%{

script that runs the Montecarlo simulations to validate the control
algorithms. Same as main, but some parameters are cicled.
CALLED FUNCTIONS: 

0 -     Marco Marchesi, SCS department, marco.marchesi@skywarder.eu
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

settings.montecarlo = true;

N_Threads = 1; % number of threads of your computer (change it to run the simulation)
N_IterPerThread = 1;

thrust_percentage = linspace(0.95,1.05,N_Threads*N_IterPerThread)';                     % defined for plot purposes
% sigma_t = (1.20-1)/3;             % thrust_percentage standard deviation
% mu_t = 1;                         %thrust_percentage mean value
% thrust_percentage= normrnd(mu_t,sigma_t,N_Threads*N_IterPerThread,1); %generate normally distributed values ( [0.8 1.20] = 3sigma) % serve il toolbox
tauServo_percentage = linspace(0.8,1.2,N_Threads*N_IterPerThread);                      % defined for plot purposes

% stochastic parameters
stoch.thrust = thrust_percentage*settings.motor.expThrust;                              % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
stoch.expThrust = (1./thrust_percentage) * settings.motor.expTime;                      % burning time - same notation as thrust here

for i = 1:size(stoch.thrust)
    plot(stoch.expThrust(i,:),stoch.thrust(i,:))
    hold on;
end
legend
stoch.wind = [1,2,3]; % set to 1 for 'wind model', 2 for 'input model', 3 for randomic  % wind model
stoch.tauServo = tauServo_percentage * settings.servo.tau;                              % servo motor time constant
stoch.controlFrequency = [1, 2, 5, 10];                             % control frequency (sets how fast the control input reference is given to the servo)
stoch.MachControl = [0.6:0.05:1];                                                       % Mach at which the air brakes can be deployed

% save arrays
save_thrust = cell(size(stoch.thrust,1),1);

save_Mach = cell(size(stoch.MachControl,1),1);

algorithm_vec = ["interp"; "std0";"std2s"]; % interpolation, PID no change, PID change every 2s

%% which montecarlo do you want to run?

run_Thrust = true;
run_Mach = false;
run_ControlFrequency = false;

%% do you want to save the results?

flagSave = true;

%% MONTECARLO 1 - THRUST

if run_Thrust == true

    for alg_index = 1%:length(algorithm_vec)
        algorithm = algorithm_vec(alg_index);

        % how many simulations do you want to run with different wind (per thrust percentage)?
        N_windSim = 1;
        save_thrust = cell(size(stoch.thrust,1),N_windSim);

        for i = 1:size(stoch.thrust,1)

            settings_mont = settings;
            contSettings_mont = contSettings;
            reference_mont = reference;

            settings_mont.motor.expThrust = stoch.thrust(i,:);                      % initialize the thrust vector of the current simulation (parfor purposes)
            settings_mont.motor.expTime = stoch.expThrust(i,:);                     % initialize the time vector for thrust of the current simulation (parfor purposes)
            settings_mont.tb = settings.tb/thrust_percentage(i);                    % initialize the burning time of the current simulation (parfor purposes)

            settings_mont.wind.model = false;
            settings_mont.wind.input = false;

            for ww = 1:N_windSim

                fprintf("simulation = " + num2str((i-1)*N_windSim + ww) + " of " + num2str(size(stoch.thrust,1)*N_windSim) + "\n")

                % wind parameters (randomized)

                settings_mont.wind.MagMin = 0; % [m/s] Minimum Magnitude
                settings_mont.wind.MagMax = 12; % [m/s] Maximum Magnitude
                settings_mont.wind.ElMin  = - 45;
                settings_mont.wind.ElMax  = + 45;
                settings_mont.wind.AzMin  = - 180;
                settings_mont.wind.AzMax  = + 180;
                % meglio crearli fuori dal for così crea tutta la gaussiana,
                % invece di avere i valori centrati sulla gaussiana, però
                % per adesso lo teniamo così
                switch algorithm
                    case 'interp'
                        [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams] = interp_run_control(settings_mont,contSettings_mont);

                    case 'std0'
                        [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams] = std_run_control(settings_mont,contSettings_mont);

                    case 'std2s'
                        [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight,windParams] = std_new_run_control(settings_mont,contSettings_mont);

                end
                save_thrust{i,ww} = struct('time',Tf,'control',Yf(:,17),'position',Yf(:,1:3), 'windParams', windParams, 'thrust_percentage', thrust_percentage(i)); 
            end
        end
    
        save_thrust_plotControl = figure;
        for i = 1:size(save_thrust,1)
            for j = 1: size(save_thrust,2)
                plot(save_thrust{i,j}.time,save_thrust{i,j}.control)
                hold on;
                grid on;
            end
        end
    
        title('Control action')
        xlabel('Time [s]')
        ylabel('Servo angle [\alpha]')
    
        apogee.thrust = [];
        save_thrust_plotApogee = figure;
        for i = 1:size(save_thrust,1)
            for j = 1: size(save_thrust,2)
                plot(thrust_percentage(i),max(-save_thrust{i,j}.position(:,3)),'*')
                hold on;
                grid on;
                apogee.thrust = [apogee.thrust max(-save_thrust{i,j}.position(:,3))];
            end
        end
        title('Apogee w.r.t. thrust')
        xlabel('Thrust percentage w.r.t. nominal')
        ylabel('Apogee [m]')
        xlim([min(thrust_percentage)-0.01,max(thrust_percentage)+0.01])
    
        apogee.thrust_mean = mean(apogee.thrust);
        apogee.thrust_variance = std(apogee.thrust);
    
        save_thrust_plotTrajectory = figure;
        for i = 1:size(save_thrust,1)
            for j = 1: size(save_thrust,2)
                plot3(save_thrust{i,j}.position(:,1),save_thrust{i,j}.position(:,2),-save_thrust{i,j}.position(:,3));
                hold on;
                grid on;
            end
        end
        title('Trajectories')
        xlabel('x')
        ylabel('y')
        zlabel('z')
    
        save_thrust_apogee_probability = figure;
        pd = fitdist(apogee.thrust','Normal');    % create normal distribution object to compute mu and sigma
        % probability to reach an apogee between 2950 and 3050
        p = normcdf([2950 3050],apogee.thrust_mean,apogee.thrust_variance);
        accuracy =( p(2) - p(1) )*100;
        x_values = linspace(2800,3200,1000);   % possible apogees
        y = pdf(pd,x_values);                  %array of values of the probability density function
        hold on
        xlabel('Reached apogee','Interpreter','latex','FontSize',15,'FontWeight','bold')
        ylabel('Probability density','Interpreter','latex','FontSize',15,'FontWeight','bold')
        plot(x_values,y)

        % save plots
        if flagSave == true
            saveas(save_thrust_plotControl,"MontecarloResults\Thrust\"+algorithm+"\controlPlot")
            saveas(save_thrust_plotApogee,"MontecarloResults\Thrust\"+algorithm+"\apogeelPlot")
            saveas(save_thrust_plotTrajectory,"MontecarloResults\Thrust\"+algorithm+"\TrajectoryPlot")
            saveas(save_thrust_apogee_probability,"MontecarloResults\Thrust\"+algorithm+"\ApogeeProbabilityPlot")
            save("MontecarloResults\Thrust\"+algorithm+"\saveThrust.mat","save_thrust","apogee")    
        end
        for i = 1    % Save results.txt
            fid = fopen( "MontecarloResults\Thrust\"+algorithm+"\"+algorithm+"Results.txt", 'wt' );  % CAMBIA IL NOME
            fprintf(fid,'Algorithm: %s',algorithm );
            fprintf(fid,'Number of simulations: %d \n \n',N_Threads*N_IterPerThread*N_windSim); % Cambia n_sim
            fprintf(fid,'Parameters: \n');
            fprintf(fid,'Thrust: +-20%% at 3*sigma, total impulse constant \n');
            fprintf(fid,'Control frequency: %d Hz \n',settings.controlFrequency);
            fprintf(fid,'Initial Mach number at which the control algorithm start: %.1f \n\n',settings.MachControl);
            fprintf(fid,'Wind model parameters: \n '); % inserisci tutti i parametri del vento
            fprintf(fid,'Wind Magnitude: 0-%d \n',settings_mont.wind.MagMax);
            fprintf(fid,'Wind minimum azimuth: %d degrees \n',settings_mont.wind.AzMin);
            fprintf(fid,'Wind maximum azimuth: %d degrees \n',settings_mont.wind.AzMax);
            fprintf(fid,'Wind minimum elevation: %d degrees \n', settings_mont.wind.ElMin);
            fprintf(fid,'Wind maximum elevation: %d degrees \n\n',settings_mont.wind.ElMax);
            fprintf(fid,'Results: \n');
            fprintf(fid,'Max apogee: %.1f \n',max(apogee.thrust));
            fprintf(fid,'Min apogee: %.1f \n',min(apogee.thrust));
            fprintf(fid,'Mean apogee: %.1f \n',apogee.thrust_mean);
            fprintf(fid,'Apogee standard deviation 3sigma: %.4f \n',3*apogee.thrust_variance);
            fprintf(fid,'Apogees within +-50m from target: %.2f %% \n',accuracy);
            fclose(fid);
        end
        save_thrust = cell(size(stoch.thrust,1),1);

    end
end














%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5










%% MONTECARLO 2 - Mach Control

if run_Mach == true

    clearvars Yf Tf t_ada t_kalman cpuTimes flagMatr
    algorithm = 'interp';

    maxP = 1; % wind "for" parameter
    maxS = 1; % wind "for" parameter

    parfor i = 1:length(stoch.MachControl)

        settings_mont = settings;
        contSettings_mont = contSettings;
        reference_mont = reference;

        settings_mont.MachControl = stoch.MachControl(i);                      % initialize the thrust vector of the current simulation (parfor purposes)

        for p = 1:maxP
            for s = 1:maxS
                settings_mont.wind.MagMin = p; % [m/s] Minimum Magnitude
                settings_mont.wind.MagMax = p; % [m/s] Maximum Magnitude
                settings_mont.wind.ElMin  = s;
                settings_mont.wind.ElMax  = 2*s;
                switch algorithm
                    case 'interp'
                        if settings.electronics
                            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr] = interp_run_control(settings_mont, contSettings_mont,reference_mont);
                        else
                            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight] = interp_run_control(settings_mont,contSettings_mont,reference_mont);
                        end
                    case 'std'
                        if settings.electronics
                            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr] = std_run_control(settings_mont, contSettings_mont);
                        else
                            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight] = std_run_control(settings_mont,contSettings_mont);
                        end
                end

                save_Mach_interp{i} = struct('time',Tf,'control',Yf(:,17),'position',Yf(:,1:3),'speed',Yf(:,4:6)); % da capire come salvare per i diversi valori di vento, in un modo sensato che vada bene anche per plottarli poi
            end
        end

    end

    save_Mach_plotControl = figure;
    for i = 1:length(save_Mach_interp)
        for p = 1: maxP
            for s = 1:maxS
                plot(save_Mach_interp{i}.time,save_Mach_interp{i}.control)
                hold on;
                grid on;
            end
        end
    end
    title('Control action')
    xlabel('Time [s]')
    ylabel('Servo angle [\alpha]')

    mean_apogee.Mach = [];
    save_Mach_plotApogee = figure;
    for i = 1:length(save_Mach_interp)
        plot(stoch.MachControl(i),max(-save_Mach_interp{i}.position(:,3)),'*')
        hold on;
        grid on;
        mean_apogee.Mach = [mean_apogee.Mach max(-save_Mach_interp{i}.position(:,3))];
    end
    title('Apogee w.r.t. MachControl')
    xlabel('MachControl')
    ylabel('Apogee [m]')

    mean_apogee.Mach = mean(mean_apogee.Mach);

    save_Mach_plotTrajectory = figure;
    for i = 1:length(save_Mach_interp)
        plot3(save_Mach_interp{i}.position(:,1),save_Mach_interp{i}.position(:,2),-save_Mach_interp{i}.position(:,3));
        hold on;
        grid on;
    end
    title('Trajectories')
    xlabel('x')
    ylabel('y')
    zlabel('z')


    save_Mach_plotSpeed = figure;
    for i = 1:length(save_Mach_interp)
        plot(save_Mach_interp{i}.time,save_Mach_interp{i}.speed(:,1));
        hold on;
        grid on;
    end
    title('speed')
    xlabel('time')
    ylabel('Vx body [m/s]')

    % save plots
    if flagSave == true
        saveas(save_Mach_plotControl,'MontecarloResults\MachControl\controlPlot')
        saveas(save_Mach_plotApogee,'MontecarloResults\MachControl\apogeelPlot')
        saveas(save_Mach_plotTrajectory,'MontecarloResults\MachControl\TrajectoryPlot')
        saveas(save_Mach_plotSpeed,'MontecarloResults\MachControl\speedPlot')
    end
end


%% MONTECARLO 3 - Control Frequency

if run_ControlFrequency == true

    clearvars Yf Tf t_ada t_kalman cpuTimes flagMatr
    algorithm = 'interp';

    maxP = 1; % wind "for" parameter
    maxS = 1; % wind "for" parameter

    parfor i = 1:length(stoch.controlFrequency)

        settings_mont = settings;
        contSettings_mont = contSettings;
        reference_mont = reference;

        settings_mont.frequencies.controlFrequency = stoch.controlFrequency(i);                      % control frequency parameter

        for p = 1:maxP
            for s = 1:maxS
                settings_mont.wind.MagMin = p; % [m/s] Minimum Magnitude
                settings_mont.wind.MagMax = p; % [m/s] Maximum Magnitude
                settings_mont.wind.ElMin  = s;
                settings_mont.wind.ElMax  = 2*s;
                switch algorithm
                    case 'interp'
                        if settings.electronics
                            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr] = interp_run_control(settings_mont, contSettings_mont,reference_mont);
                        else
                            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight] = interp_run_control(settings_mont,contSettings_mont,reference_mont);
                        end
                    case 'std'
                        if settings.electronics
                            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr] = std_run_control(settings_mont, contSettings_mont);
                        else
                            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight] = std_run_control(settings_mont,contSettings_mont);
                        end
                end

                save_ControlFrequency{i} = struct('time',Tf,'control',Yf(:,17),'position',Yf(:,1:3),'speed',Yf(:,4:6));
            end
        end

    end

    save_ControlFrequency_plotControl = figure;
    for i = 1:length(save_ControlFrequency)
        for p = 1: maxP
            for s = 1:maxS
                plot(save_ControlFrequency{i}.time,save_ControlFrequency{i}.control)
                hold on;
                grid on;
            end
        end
    end
    title('Control action')
    xlabel('Time [s]')
    ylabel('Servo angle [\alpha]')

    mean_apogee.ControlFrequency = [];
    save_ControlFrequency_plotApogee = figure;
    for i = 1: length(save_ControlFrequency)
        plot(stoch.controlFrequency(i),max(-save_ControlFrequency{i}.position(:,3)),'*')
        hold on;
        grid on;
        mean_apogee.ControlFrequency = [mean_apogee.ControlFrequency max(-save_ControlFrequency{i}.position(:,3))];
    end
    title('Apogee w.r.t. ControlFrequency')
    xlabel('Control Frequency')
    ylabel('Apogee [m]')

    mean_apogee.ControlFrequency = mean(mean_apogee.ControlFrequency);

    save_ControlFrequency_plotTrajectory = figure;
    for i = 1:length(save_ControlFrequency)
        plot3(save_ControlFrequency{i}.position(:,1),save_ControlFrequency{i}.position(:,2),-save_ControlFrequency{i}.position(:,3));
        hold on;
        grid on;
    end
    title('Trajectories')
    xlabel('x')
    ylabel('y')
    zlabel('z')


    save_ControlFrequency_plotSpeed = figure;
    for i = 1:length(save_ControlFrequency)
        plot(save_ControlFrequency{i}.time,save_ControlFrequency{i}.speed(:,1));
        hold on;
        grid on;
    end
    title('speed')
    xlabel('time')
    ylabel('Vx body [m/s]')

    % save plots
    if flagSave == true
        saveas(save_ControlFrequency_plotControl,'MontecarloResults\ControlFrequency\controlPlot')
        saveas(save_ControlFrequency_plotApogee,'MontecarloResults\ControlFrequency\apogeelPlot')
        saveas(save_ControlFrequency_plotTrajectory,'MontecarloResults\ControlFrequency\TrajectoryPlot')
        saveas(save_ControlFrequency_plotSpeed,'MontecarloResults\ControlFrequency\speedPlot')
    end

end









