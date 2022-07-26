%this is a code that is pretty useless until you have to test the air
%brakes from the simulations. Given the save_thrust from montecarlo
%simulations, this code just picks one of the simulated trajectories and
%save a log.m file that contains all data of that simulation.


%%
% load("saveThrust.mat") % this needs to be in the folder, otherwise comment this line and just load it manually.

% pick a number for the simulation, you can randomize it if you want
N = 35;

% generate log
clearvars llogg
llogg(:,1) = save_thrust{N,1}.time*1e6;
llogg(:,2) = save_thrust{N,1}.control;
llogg(:,3:5) = save_thrust{N,1}.position;
llogg(:,6:8) = save_thrust{N,1}.speed;

% = save_thrust{80,1}.ap_ref(:,2);

% sample to have it at the set value of hz
obsw_freq = 10; %Hz

time_max = max(llogg(:,1));
time_vec = 0:1/obsw_freq * 1e6:time_max;

log_sampled = interp1(llogg(:,1),llogg,time_vec);

%export csv file
csvwrite('MontecarloResults\log_simulation_std0.csv',log_sampled)