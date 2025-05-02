%{

mainSimulator - this is the main script; it runs the rocket ascent with
every possible feature simulated (i.e. burning phase, attitude, air
braking, ADA, NAS, Kalman, Parafoil, landing). 

REVISIONS:
- 0     xx/xx/xxxx, Release,    Ruben Di Battista
- 1     16/04/2021, update,     Alessandro Del Duca

- 2     07/04/2022, update,     Davide Rosato, AFD Departmwent
                    Compatibility with common functions folder

- 3     15/04/2022, update,     Marco Marchesi, SCS department
                    Latest control algorithms

- 4     01/09/2023, update,     Marco Marchesi, GNC IPT
                                Giuseppe Brentino, GNC IPT
                                Pier Francesco Bachini, GNC IPT
                                Alessandro Donadi, GNC IPT
                    Parafoil simulation, latest control algorithms, 
                    sensor fault detection, engine shut-down, and much
                    more.

- 5     07/02/2025, update,     Stefano Belletti, GNC IPT
                                Samuel Flore, GNC IPT
                    Added new sensor classes and noises, added new noise
                    analyzer, fix plots.

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

%% path loading
restoredefaultpath;
filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end
addpath(genpath(currentPath));

% add common submodule path
commonPath = strcat('../common');
addpath(genpath(commonPath));

%% CHECK SUBMODULES UPDATES
% if ~exist('flagSubmodulesUpdated','var')
%     checkSubmodules;
%     flagSubmodulesUpdated = true;
% end

%% CONFIGs
conf.script = "simulator"; % this defines which configuration scripts to run
settings.montecarlo = false;
configSimulator; 

%% ALGORITHM TUNING
% basically if this is true sets the randomic value of the wind to the same
% values for each simulation, so it has the same atmospheric conditions
% each time

if settings.tuning
  	rng('default');
end 

%% SET SPECIFIC PARAMETERS FOR A PRE LAUNCH SIMULATION
%  config_SpecialConditions;

%% START THE SIMULATION
% T = vector of time used by ODE, [s] also for Tf Ta
% Y = State = ( x y z | u v w | p q r | q0 q1 q2 q3 | thetax thetay thetaz | ap_ref ) also for Ya,Yf corresponding to T

%% simulation:
[simOutput] = std_run(settings,contSettings,rocket,environment,mission);

%% PLOTS
if ~exist("../commonFunctions/graphics/general-utilities/","dir")
    warning('To export file you need to download the repository, read the README file in the folder')
end
% std_plots(simOutput,settings,contSettings,mission,environment)
% sensor_plots(simOutput, environment, rocket, settings);
% report_plots(simOutput,settings,contSettings)

%% state visualiser
% animateOrientation(simOutput.Y(:,11),simOutput.Y(:,12),simOutput.Y(:,13),simOutput.Y(:,10),simOutput.t)
% animateOrientation(simOutput.NAS(:,7),simOutput.NAS(:,8),simOutput.NAS(:,9),simOutput.NAS(:,10),simOutput.t_nas)

%% DATA-PRINTING
printOutput(simOutput,settings);

%% save data
% save("Simulation_log.mat","Tf","Yf","data_flight")

%% export data for HIL simulations /cpp usage: 
% the files are stored in the folder 
if settings.flagExportCSV % this is set in configFlags
    % air brakes
    export_HILdataMTR;
    export_HILdataABK;
    % parafoil
    if settings.scenario == "descent" || settings.scenario == "full flight"
        export_HILdataPRF;
    end
    export_HILdataADA
    export_HILdataNAS
end

% Reset to default plot
% reset(groot);


%%
close all
zvk = simOutput.sensors.zvk;

figure()
subplot(3,1,1)
plot(zvk.time,ones(1,length(zvk.time)).*-0.15, 'r',...
     zvk.time,zvk.states(:,11), 'r', 'LineWidth',2);
legend('X','bias accelerometro x')
subplot(3,1,2)
plot(zvk.time,ones(1,length(zvk.time)).*0.12, 'g',...
     zvk.time,zvk.states(:,12), 'g','LineWidth',2);
legend('Y','bias accelerometro y')
subplot(3,1,3)
plot(zvk.time,ones(1,length(zvk.time)).*-0.05, 'b',...
     zvk.time,zvk.states(:,13),'b', 'LineWidth',2);
legend('Z','bias acceleromtro z')

figure()
subplot(3,1,1)
plot(zvk.time,ones(1,length(zvk.time)).*0.008,'r',...
     zvk.time,zvk.states(:,14), 'r', 'LineWidth',2);
legend('X','bias gyro x')
subplot(3,1,2)
plot(zvk.time,ones(1,length(zvk.time)).*-0.005, 'g',...
     zvk.time,zvk.states(:,15), 'g','LineWidth',2);
legend('Y','bias gyro y')
subplot(3,1,3)
plot(zvk.time,ones(1,length(zvk.time)).*0.002, 'b',...
     zvk.time,zvk.states(:,16), 'b','LineWidth',2);
legend('Z','bias gyro z')




[a,b,c] = quat2angle(zvk.states(:,1:4));
a = wrapTo2Pi(a);

a = rad2deg(a);
b = rad2deg(b);
c = rad2deg(c);

figure()
subplot(3,1,1)
hold on
plot(zvk.time,a)
yline(a(1))
subplot(3,1,2)
hold on
plot(zvk.time,b)
yline(b(1))
subplot(3,1,3)
hold on
plot(zvk.time,c)
yline(c(1))







quat_init = zvk.states(1,1:4);
quat_end = zvk.states(end,1:4);

disp(quat_init)
disp(quat_end )

disp(rad2deg(quat2eul(quat_init)))
disp(rad2deg(quat2eul(quat_end)))


A_init   = [quat_init(1)^2 - quat_init(2)^2 - quat_init(3)^2 + quat_init(4)^2,               2*(quat_init(1)*quat_init(2) + quat_init(3)*quat_init(4)),                 2*(quat_init(1)*quat_init(3) - quat_init(2)*quat_init(4));
     2*(quat_init(1)*quat_init(2) - quat_init(3)*quat_init(4)),      -quat_init(1)^2 + quat_init(2)^2 - quat_init(3)^2 + quat_init(4)^2,                2*(quat_init(2)*quat_init(3) + quat_init(1)*quat_init(4)) ;
     2*(quat_init(1)*quat_init(3) + quat_init(2)*quat_init(4)),               2*(quat_init(2)*quat_init(3) - quat_init(1)*quat_init(4)),       -quat_init(1)^2 - quat_init(2)^2 + quat_init(3)^2 + quat_init(4)^2];

A_end   = [quat_end(1)^2 - quat_end(2)^2 - quat_end(3)^2 + quat_end(4)^2,               2*(quat_end(1)*quat_end(2) + quat_end(3)*quat_end(4)),                 2*(quat_end(1)*quat_end(3) - quat_end(2)*quat_end(4));
     2*(quat_end(1)*quat_end(2) - quat_end(3)*quat_end(4)),      -quat_end(1)^2 + quat_end(2)^2 - quat_end(3)^2 + quat_end(4)^2,                2*(quat_end(2)*quat_end(3) + quat_end(1)*quat_end(4)) ;
     2*(quat_end(1)*quat_end(3) + quat_end(2)*quat_end(4)),               2*(quat_end(2)*quat_end(3) - quat_end(1)*quat_end(4)),       -quat_end(1)^2 - quat_end(2)^2 + quat_end(3)^2 + quat_end(4)^2];


% disp(A_end-A_init)
