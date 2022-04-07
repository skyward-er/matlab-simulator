%{

PLOTS - this script plots the computed data

CALLED FUNCTIONS: 

REVISIONS:
- #0 13/01/2021, Release, Alessandro Del Duca

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

%% Z(t)
figure('Name','Altitude vs Time','NumberTitle','off');
for i = 1:Ntraj
    plot(trajectories{i}.t_ref, trajectories{i}.Z_ref);
    grid on; hold on;
end
xlabel('Time [s]'); ylabel('Altitude [m]'); title('Altitude vs Time')

%% Vz(t)
figure('Name','Vertical velocity vs Time','NumberTitle','off');
for i = 1:Ntraj
    plot(trajectories{i}.t_ref, trajectories{i}.VZ_ref);
    grid on; hold on;
end
xlabel('Time [s]'); ylabel('$V_{z}$ [m/s]'); title('Vertical velocity vs Time')

%% X(t)
figure('Name','Latitude vs Time','NumberTitle','off');
for i = 1:Ntraj
    plot(trajectories{i}.t_ref, trajectories{i}.X_ref);
    grid on; hold on;
end
xlabel('Time [s]'); ylabel('Latitude [m]'); title('Latitude vs Time')

%% Vx(t)
figure('Name','Horizontal velocity vs Time','NumberTitle','off');
for i = 1:Ntraj
    plot(trajectories{i}.t_ref, trajectories{i}.VX_ref);
    grid on; hold on;
end
xlabel('Time [s]'); ylabel('$V_{x}$ [m/s]'); title('Horizontal velocity vs Time')

%% Y(t)
figure('Name','Latitude vs Time','NumberTitle','off');
for i = 1:Ntraj
    plot(trajectories{i}.t_ref, trajectories{i}.Y_ref);
    grid on; hold on;
end
xlabel('Time [s]'); ylabel('Latitude [m]'); title('Latitude vs Time')

%% Vy(t)
figure('Name','Horizontal velocity vs Time','NumberTitle','off');
for i = 1:Ntraj
    plot(trajectories{i}.t_ref, trajectories{i}.VY_ref);
    grid on; hold on;
end
xlabel('Time [s]'); ylabel('$V_{y}$ [m/s]'); title('Horizontal velocity vs Time')

%% Vz(z)
figure('Name','Vertical velocity vs Altitude','NumberTitle','off');
for i = 1:Ntraj
    indexes = find(trajectories{i}.VZ_ref < Vz_initial);
    plot(trajectories{i}.Z_ref(indexes), trajectories{i}.VZ_ref(indexes));
    grid on; hold on;
end
xlabel('Altitude [m]'); ylabel('$V_{z}$ [m/s]'); title('Vertical velocity vs Altitude')