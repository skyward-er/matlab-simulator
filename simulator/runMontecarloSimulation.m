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

thrust_percentage = linspace(0.95,1.05,22)';                                            % defined for plot purposes
tauServo_percentage = linspace(0.8,1.2,22);                                             % defined for plot purposes

% stochastic parameters
stoch.thrust = thrust_percentage*settings.motor.expThrust;                              % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
stoch.expThrust = (1./thrust_percentage) * settings.motor.expTime;                      % burning time - same notation as thrust here

stoch.wind = [1,2,3]; % set to 1 for 'wind model', 2 for 'input model', 3 for randomic  % wind model
stoch.tauServo = tauServo_percentage * settings.servo.tau;                              % servo motor time constant
stoch.controlFrequency = [1, 2, 5, 10];                             % control frequency (sets how fast the control input reference is given to the servo)
stoch.MachControl = [0.6:0.05:1];                                                       % Mach at which the air brakes can be deployed


% save arrays
save_thrust = cell(size(stoch.thrust,1),1);
save_Mach = cell(size(stoch.MachControl,1),1);


%which montecarlo do you want to run?
run_Thrust = false;
run_Mach = false;
run_ControlFrequency = true;

%% ALGORITHM TUNING
% basically if this is true sets the randomic value of the wind to the same
% values for each simulation, so it has the same atmospheric conditions
% each time
if settings.tuning
    rng('default')
end

%% MONTECARLO 1 - THRUST

if run_Thrust == true

    algorithm = 'interp';

    maxP = 1; % wind "for" parameter
    maxS = 1; % wind "for" parameter

    parfor i = 1:size(stoch.thrust,1)

        settings_mont = settings;
        contSettings_mont = contSettings;
        reference_mont = reference;

        settings_mont.motor.expThrust = stoch.thrust(i,:);                      % initialize the thrust vector of the current simulation (parfor purposes)
        settings_mont.motor.expTime = stoch.expThrust(i,:);                     % initialize the time vector for thrust of the current simulation (parfor purposes)
        settings_mont.tb = settings.tb/thrust_percentage(i);                    % initialize the burning time of the current simulation (parfor purposes)

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

                save_thrust{i} = struct('time',Tf,'control',Yf(:,17),'position',Yf(:,1:3),'speed',Yf(:,4:6)); % da capire come salvare per i diversi valori di vento, in un modo sensato che vada bene anche per plottarli poi
            end
        end

    end

    save_thrust_plotControl = figure;
    for i = 1:length(save_thrust)
        for p = 1: maxP
            for s = 1:maxS
                plot(save_thrust{i}.time,save_thrust{i}.control)
                hold on;
                grid on;
            end
        end
    end
    title('Control action')
    xlabel('Time [s]')
    ylabel('Servo angle [\alpha]')


    save_thrust_plotApogee = figure;
    for i = 1: length(save_thrust)
        plot(thrust_percentage(i),max(-save_thrust{i}.position(:,3)),'*')
        hold on;
        grid on;
    end
    title('Apogee w.r.t. thrust')
    xlabel('Thrust percentage w.r.t. nominal')
    ylabel('Apogee [m]')

    save_thrust_plotTrajectory = figure;
    for i = 1:length(save_thrust)
        plot3(save_thrust{i}.position(:,1),save_thrust{i}.position(:,2),-save_thrust{i}.position(:,3));
        hold on;
        grid on;
    end
    title('Trajectories')
    xlabel('x')
    ylabel('y')
    zlabel('z')


    save_thrust_plotSpeed = figure;
    for i = 1:length(save_thrust)
        plot(save_thrust{i}.time,save_thrust{i}.speed(:,1));
        hold on;
        grid on;
    end
    title('speed')
    xlabel('Thrust percentage w.r.t. nominal')
    ylabel('Apogee [m]')

end
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

                save_Mach{i} = struct('time',Tf,'control',Yf(:,17),'position',Yf(:,1:3),'speed',Yf(:,4:6)); % da capire come salvare per i diversi valori di vento, in un modo sensato che vada bene anche per plottarli poi
            end
        end

    end

    save_Mach_plotControl = figure;
    for i = 1:length(save_Mach)
        for p = 1: maxP
            for s = 1:maxS
                plot(save_Mach{i}.time,save_Mach{i}.control)
                hold on;
                grid on;
            end
        end
    end
    title('Control action')
    xlabel('Time [s]')
    ylabel('Servo angle [\alpha]')


    save_Mach_plotApogee = figure;
    for i = 1:length(save_Mach)
        plot(stoch.MachControl(i),max(-save_Mach{i}.position(:,3)),'*')
        hold on;
        grid on;
    end
    title('Apogee w.r.t. MachControl')
    xlabel('MachControl')
    ylabel('Apogee [m]')

    save_Mach_plotTrajectory = figure;
    for i = 1:length(save_Mach)
        plot3(save_Mach{i}.position(:,1),save_Mach{i}.position(:,2),-save_Mach{i}.position(:,3));
        hold on;
        grid on;
    end
    title('Trajectories')
    xlabel('x')
    ylabel('y')
    zlabel('z')


    save_Mach_plotSpeed = figure;
    for i = 1:length(save_Mach)
        plot(save_Mach{i}.time,save_Mach{i}.speed(:,1));
        hold on;
        grid on;
    end
    title('speed')
    xlabel('time')
    ylabel('Vx body [m/s]')
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


    save_ControlFrequency_plotApogee = figure;
    for i = 1: length(save_ControlFrequency)
        plot(stoch.controlFrequency(i),max(-save_ControlFrequency{i}.position(:,3)),'*')
        hold on;
        grid on;
    end
    title('Apogee w.r.t. ControlFrequency')
    xlabel('Control Frequency')
    ylabel('Apogee [m]')

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
end




%% DATA-PRINTING
%
% Na = length(Yf(:,1));
%
% % POSITIONS
% xa =  Yf(:,1);
% ya =  Yf(:,2);
% za = -Yf(:,3);
% Xa = [xa, ya, za];
%
% [max_z, i_apo] = max(za);
% T_apo = Tf(i_apo);
%
% % VELOCITIES
% ua =  Yf(:,4);
% va =  Yf(:,5);
% wa = -Yf(:,6);
% Va = [ua, va, wa];
%
% % MAXIMUM POSITIONS, VELOCITIES AND ACCELERATION
% abs_X = vecnorm(Xa');
% abs_V = vecnorm(Va');
%
% [max_dist, imax_dist] = max(abs_X);
% [max_v, imax_v] = max(abs_V);
%
% % DATA RECORD (display)
% fprintf('OUTCOMES:\n\n\n')
%
% fprintf('total computational Time: %.3f [s]: \n', sum(cpuTimes))
% fprintf('mean step computational Time: %.3f [s]: \n', mean(cpuTimes))
% fprintf('max step omputational Time: %.3f [s]: \n\n', max(cpuTimes))
%
% fprintf('apogee: %.1f [m] \n', max_z);
% fprintf('@time: %g [sec] \n\n', T_apo)
%
% fprintf('max speed reached: %g [m/s] \n', max_v)
% fprintf('@altitude: %g [m] \n', za(imax_v))
% fprintf('@time: %g [sec] \n\n', Tf(imax_v))
%
% if not(settings.electronics)
%     M = data_flight.interp.M;
%     [max_M, imax_M] = max(M);
%     A = data_flight.accelerations.body_acc;
%     abs_A = vecnorm(A');
%     [max_a, imax_a] = max(abs_A);
%
%     fprintf('max Mach reached: %g [-] \n', max_M)
%     fprintf('@altitude: %g [m] \n', za(imax_M))
%     fprintf('@velocity: %g [m/s] \n', abs_V(imax_M))
%     fprintf('@time: %g [sec] \n\n', Tf(imax_M))
%
%     fprintf('max acceleration reached: %g [m/s2] = %g [g] \n', max_a, max_a/9.81)
%     fprintf('@velocity: %g [m/s] \n', abs_V(imax_a))
%     fprintf('@time: %g [sec] \n\n', Tf(imax_a))
%
% %% Apogee detection time
%     fprintf('ADA apogee detection time: %g [sec] \n', t_ada)
%     fprintf('Kalman apogee detection time: %g [sec] \n', t_kalman)
%     fprintf('Simulated apogee time : %g [sec] \n', T_apo)
% end
%
% delete('launchFlag.txt')
%
% clearvars -except Yf data_flight settings