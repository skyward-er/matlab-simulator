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

settings.montecarlo = true;

%% MONTECARLO SETTINGS
thrust_percentage = linspace(0.95,1.05,2)';                                            % defined for plot purposes
tauServo_percentage = linspace(0.8,1.2,2);                                             % defined for plot purposes

stoch.thrust = thrust_percentage*settings.motor.expThrust;                              % thrust
stoch.wind = [1,2,3]; % set to 1 for 'wind model', 2 for 'input model', 3 for randomic  % wind model 
stoch.tauServo = tauServo_percentage * settings.servo.tau;                              % servo motor time constant
stoch.controlFrequency = 1:10;                                                          % control frequency (sets how fast the control input reference is given to the servo)
stoch.MachControl = [0.6:0.05:1];                                                       % Mach at which the air brakes can be deployed

%% ALGORITHM TUNING
% basically if this is true sets the randomic value of the wind to the same
% values for each simulation, so it has the same atmospheric conditions
% each time
if settings.tuning
	rng('default')
end

%% START THE CHOSEN SIMULATION
% T = vector of time used by ODE, [s] also for Tf Ta
% Y = State = ( x y z | u v w | p q r | q0 q1 q2 q3 | thetax thetay thetaz | ap_ref ) also for Ya,Yf corresponding to T

algorithm = 'interp';

save.thrust.time = zeros(size(stoch.thrust,1));
save.thrust.control = zeros(size(stoch.thrust,1));
save.thrust.position = [];
save.thrust.speed = [];

parfor i = 1:size(stoch.thrust,1) 

    settings_mont = settings;
    contSettings_mont = contSettings;
    reference_mont = reference;


    settings_mont.motor.expThrust = stoch.thrust(i,:);
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
    
    time_interp = interp1(linspace(Tf(1),Tf(end),1000),Tf);
    control_interp = interp1(linspace(Yf(1,17),Yf(end,17),1000),Yf(:,17));
    
    position_interp = zeros(1000,3);
    for ind = 1:3
    position_interp(ind) = interp1(linspace(Yf(1,ind),Yf(end,ind),1000),Yf(:,ind));
    end
    speed_interp = zeros(1000,3);
    for ind2 = 4:6
    speed_interp(ind2) = interp1(linspace(Yf(1,ind2),Yf(end,ind2),1000),Yf(:,ind2));
    end

    save.thrust.time(:,i) = time_interp;
    save.thrust.control(:,i) = control_interp;
%     save.thrust.position(i) = position_interp;
%     save.thrust.speed(i) = speed_interp;

end

plotControl = figure;
for i = 1: length(save.thrust)
    plot(save.thrust{i}.time,save.thrust{i}.control)
    hold on;
    grid on;
end
title('Control action')
xlabel('Time [s]')
ylabel('Servo angle [\alpha]')


plotApogee = figure;
for i = 1: length(save.thrust)
    plot(thrust_percentage(i),-save.thrust{i}.position(end,3),'*')
    hold on;
    grid on;
end
title('Apogee w.r.t. thrust')
xlabel('Thrust percentage w.r.t. nominal')
ylabel('Apogee [m]')

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
 