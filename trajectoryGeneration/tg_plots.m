%{

PLOTS - this script plots the computed data

CALLED FUNCTIONS: 

REVISIONS:
- #0 13/01/2021, Release, Alessandro Del Duca

Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}


% % % % Z(t)
% % % figure('Name','Altitude vs Time','NumberTitle','off');
% % % for i = 1:Ntraj
% % %     plot(trajectories{i}.t_ref, trajectories{i}.Z_ref);
% % %     grid on; hold on;
% % % end
% % % xlabel('Time [s]'); ylabel('Altitude [m]'); title('Altitude vs Time')
% % % 
% % % % Vz(t)
% % % figure('Name','Vertical velocity vs Time','NumberTitle','off');
% % % for i = 1:Ntraj
% % %     plot(trajectories{i}.t_ref, trajectories{i}.VZ_ref);
% % %     grid on; hold on;
% % % end
% % % xlabel('Time [s]'); ylabel('$V_{z}$ [m/s]'); title('Vertical velocity vs Time')
% % % 
% % % % X(t)
% % % figure('Name','Latitude vs Time','NumberTitle','off');
% % % for i = 1:Ntraj
% % %     plot(trajectories{i}.t_ref, trajectories{i}.X_ref);
% % %     grid on; hold on;
% % % end
% % % xlabel('Time [s]'); ylabel('Latitude [m]'); title('Latitude vs Time')
% % % 
% % % % Vx(t)
% % % figure('Name','Horizontal velocity vs Time','NumberTitle','off');
% % % for i = 1:Ntraj
% % %     plot(trajectories{i}.t_ref, trajectories{i}.VX_ref);
% % %     grid on; hold on;
% % % end
% % % xlabel('Time [s]'); ylabel('$V_{x}$ [m/s]'); title('Horizontal velocity vs Time')
% % % 
% % % % Y(t)
% % % figure('Name','Latitude vs Time','NumberTitle','off');
% % % for i = 1:Ntraj
% % %     plot(trajectories{i}.t_ref, trajectories{i}.Y_ref);
% % %     grid on; hold on;
% % % end
% % % xlabel('Time [s]'); ylabel('Latitude [m]'); title('Latitude vs Time')
% % % 
% % % % Vy(t)
% % % figure('Name','Horizontal velocity vs Time','NumberTitle','off');
% % % for i = 1:Ntraj
% % %     plot(trajectories{i}.t_ref, trajectories{i}.VY_ref);
% % %     grid on; hold on;
% % % end
% % % xlabel('Time [s]'); ylabel('$V_{y}$ [m/s]'); title('Horizontal velocity vs Time')

% Vz(z)_ARB
ref = figure('Name','Vertical velocity vs Altitude','NumberTitle','off');
for j = 1:N_mass
for i = 1:Ntraj_ARB
%     indexes = find(trajectories{i}.VZ_ref < Vz_initial);
%     plot(trajectories{i}.Z_ref(indexes), trajectories{i}.VZ_ref(indexes));
plot(trajectories{i,j}.Z_ref, trajectories{i,j}.VZ_ref);
    grid on; hold on;
end
end
xlabel('Altitude [m]'); ylabel('V_{z} [m/s]'); title('Vertical velocity vs Altitude for ARB')

  exportgraphics(ref,'references.pdf','ContentType','vector')

% Vz(z)_MTR
figure('Name','Vertical velocity vs Altitude','NumberTitle','off');
for j = 1:N_mass
plot(trajectories_MTR{j}.Z_ref, trajectories_MTR{j}.VZ_ref);
    grid on; hold on;
end

xlabel('Altitude [m]'); ylabel('$V_{z}$ [m/s]'); title('Vertical velocity vs Altitude for MTR')