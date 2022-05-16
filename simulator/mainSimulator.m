%{

mainSimulator - this is the main script; it runs the rocket ascent with
every possible feature simulated (i.e. burning phase, attitude, air
braking, ADA, NAS, Kalman, Parafoil, landing)

CALLED SCRIPTS: configSimulator, simulationsData
                in configSimulator are set all the options for the
                simulation, which basically choose if kalman, ADA, NAS, air
                brake system and all the rest is simulated or not.

CALLED FUNCTIONS: 

REVISIONS:
- 0     16/04/2021, Release,    Alessandro Del Duca

- 1     07/04/2022, update,     Davide Rosato, AFD Department
                    Compatibility with common functions folder

- 2     15/04/2022, update,     Marco Marchesi, SCS department
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
switch algorithm
    case 'interp'
        if settings.electronics
            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr] = interp_run_control_test(settings, contSettings);
        else
            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight] = interp_run_control_test(settings,contSettings);
        end
    case 'std'
        if settings.electronics
            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr] = std_run_control(settings, contSettings);
        else
            [Yf, Tf, t_ada, t_kalman, cpuTimes, flagMatr, data_flight] = std_run_control(settings,contSettings);
        end
end

%% DATA-PRINTING

Na = length(Yf(:,1));

% POSITIONS
xa =  Yf(:,1);
ya =  Yf(:,2);
za = -Yf(:,3);
Xa = [xa, ya, za];

[max_z, i_apo] = max(za);
T_apo = Tf(i_apo);

% VELOCITIES
ua =  Yf(:,4);
va =  Yf(:,5);
wa = -Yf(:,6);
Va = [ua, va, wa];

% MAXIMUM POSITIONS, VELOCITIES AND ACCELERATION
abs_X = vecnorm(Xa');
abs_V = vecnorm(Va');

[max_dist, imax_dist] = max(abs_X);
[max_v, imax_v] = max(abs_V);

% DATA RECORD (display)
fprintf('OUTCOMES:\n\n\n')

fprintf('total computational Time: %.3f [s]: \n', sum(cpuTimes))
fprintf('mean step computational Time: %.3f [s]: \n', mean(cpuTimes))
fprintf('max step omputational Time: %.3f [s]: \n\n', max(cpuTimes))

fprintf('apogee: %.1f [m] \n', max_z);
fprintf('@time: %g [sec] \n\n', T_apo)

fprintf('max speed reached: %g [m/s] \n', max_v)
fprintf('@altitude: %g [m] \n', za(imax_v))
fprintf('@time: %g [sec] \n\n', Tf(imax_v))

if not(settings.electronics)
    M = data_flight.interp.M;
    [max_M, imax_M] = max(M);
    A = data_flight.accelerations.body_acc;
    abs_A = vecnorm(A');
    [max_a, imax_a] = max(abs_A);
    
    fprintf('max Mach reached: %g [-] \n', max_M)
    fprintf('@altitude: %g [m] \n', za(imax_M))
    fprintf('@velocity: %g [m/s] \n', abs_V(imax_M))
    fprintf('@time: %g [sec] \n\n', Tf(imax_M))
    
    fprintf('max acceleration reached: %g [m/s2] = %g [g] \n', max_a, max_a/9.81)
    fprintf('@velocity: %g [m/s] \n', abs_V(imax_a))
    fprintf('@time: %g [sec] \n\n', Tf(imax_a))

%% Apogee detection time
    fprintf('ADA apogee detection time: %g [sec] \n', t_ada)
    fprintf('Kalman apogee detection time: %g [sec] \n', t_kalman)
    fprintf('Simulated apogee time : %g [sec] \n', T_apo)
end
fprintf('apogee: %.1f [m] \n', max_z);
delete('launchFlag.txt')

clearvars -except Yf data_flight settings
 