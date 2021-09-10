clearvars
close all
clc

addpath('Cd_rho_computation');
addpath('../simulator')
run('configEuroc.m')

%% Parameters needed for simulation

% Impose the final condition I want to reach.
% Then I start integrating backwards until I reach z = 0
flight = 'Euroc';
    switch flight
        case 'Roccaraso'
            Vz_final =  0;
            z_final  =  1350;
            Vx_final =  40;  
            x_final  =  500;  
            Vy_final =  0;  
            y_final  =  0;  
        case 'Euroc'
            Vz_final =  0;
            z_final  =  3000;
            Vx_final =  70;  
            x_final  =  1500;  
            Vy_final =  0;  
            y_final  =  0; 
    end

%% Compute the trajectories by back integration

deltaS_values = 0.001:0.001:0.009; % I exclude the limits for robustness

for index=1:length(deltaS_values)

deltaS = deltaS_values(index);

% Start simulink simulation
generation = sim('Trajectory_generation');

% Get the output of the simulation
t_ref   = flip(30 - generation.tout); % (30seconds - time) In this way a plot the trajectories in a clearer way
Z_ref   = flip(generation.z_simul); 
VZ_ref  = flip(generation.Vz_simul);
X_ref   = flip(generation.x_simul); 
VX_ref  = flip(generation.Vx_simul);
Y_ref   = flip(generation.y_simul); 
VY_ref  = flip(generation.Vy_simul);
cd      = flip(generation.cd);

% Save the trajectories in a struct. Easier to plot
trajectories(index) = struct('t_ref',t_ref,'Z_ref',Z_ref,'VZ_ref',VZ_ref,'X_ref',X_ref,'VX_ref',VX_ref, 'Y_ref',Y_ref,'VY_ref',VY_ref);
trajectories_saving(index) = struct('Z_ref',Z_ref,'VZ_ref',VZ_ref,'X_ref',X_ref,'VX_ref',VX_ref, 'Y_ref',Y_ref,'VY_ref',VY_ref);

end

% % Save the trajectories in a unique file
switch flight
    case 'Roccaraso'
        save('Trajectories_roccaraso.mat','trajectories_saving');
    case 'Euroc'
        save('Trajectories_euroc.mat','trajectories_saving')
end

%% Plot trajectories

% Plot z(t) 
for index=1:length(deltaS_values)
figure(1);
title('Altitude plot')
hold on
plot(trajectories(index).t_ref, trajectories(index).Z_ref)
%scatter(trajectories(index).t_ref, trajectories(index).Z_ref,'.')
xlabel('Time'); ylabel('Altitude');
hold off
end

% Plot  Vz(t)
for index=1:length(deltaS_values)
figure(2);
hold on
title('Vertical velocity plot')
plot(trajectories(index).t_ref, trajectories(index).VZ_ref)
%scatter(trajectories(index).t_ref, trajectories(index).V_ref,'.')
xlabel('Time'); ylabel('Velocity');
% axis([10 30 0 300])
hold off
end

% Plot x(t) 
for index=1:length(deltaS_values)
figure(3);
title('Latitude plot')
hold on
plot(trajectories(index).t_ref, trajectories(index).X_ref)
%scatter(trajectories(index).t_ref, trajectories(index).Z_ref,'.')
xlabel('Time'); ylabel('Latitude');
hold off
end

% Plot  Vx(t)
for index=1:length(deltaS_values)
figure(4);
hold on
title('Horizontal velocity plot')
plot(trajectories(index).t_ref, trajectories(index).VX_ref)
%scatter(trajectories(index).t_ref, trajectories(index).V_ref,'.')
xlabel('Time'); ylabel('Velocity');
%axis([10 30 0 250])
hold off
end

% Plot y(t) 
for index=1:length(deltaS_values)
figure(5);
title('Latitude plot')
hold on
plot(trajectories(index).t_ref, trajectories(index).Y_ref)
%scatter(trajectories(index).t_ref, trajectories(index).Z_ref,'.')
xlabel('Time'); ylabel('Latitude');
hold off
end

% Plot  Vy(t)
for index=1:length(deltaS_values)
figure(6);
hold on
title('Horizontal velocity plot')
plot(trajectories(index).t_ref, trajectories(index).VY_ref)
%scatter(trajectories(index).t_ref, trajectories(index).V_ref,'.')
xlabel('Time'); ylabel('Velocity');
%axis([10 30 0 250])
hold off
end

% Plot Vz(z)
for index=1:length(deltaS_values)
index_plot = find(trajectories(index).VZ_ref < 250); % Aero-brakes start when Vz < 0.7*MACH = 250 m/s
Z_plot = trajectories(index).Z_ref(index_plot);
V_plot = trajectories(index).VZ_ref(index_plot);

figure(7);
hold on
title('Vertical velocity and altitude plot')
plot(Z_plot, V_plot)
%scatter(Z_plot, V_plot)
xlabel('Altitude'); ylabel('Velocity');
hold off
end




% % Plot Cd(t) 
% for index=1:length(deltaS_values)
% figure(6);
% title('Drag coefficient plot')
% hold on
% size(trajectories(index).t_ref)
% size(trajectories(index).cd)
% plot(trajectories(index).t_ref, trajectories(index).cd)
% %scatter(trajectories(index).t_ref, trajectories(index).cd,'.')
% xlabel('Time'); ylabel('Cd');
% hold off
% end

