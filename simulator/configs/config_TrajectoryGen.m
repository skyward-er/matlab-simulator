%{

trajectory generation configuration script - it is used only when the
trajectory generation is run.

%}

%% AEROBRAKES EXTENSION DISCRETIZATION
settings.Ndx = 10;                                % [m] Number of trajectories

%% FINAL VERTICAL VELOCITY
settings.Vz_initialPerc = 0.05;                   % [-] Percentage of increasing the initial vertical velocity

%% SAVE TRAJECTORIES? 
settings.save = true;

%% ODE SETTINGS
settings.ode.optionsascTrajGen = odeset('Events', @eventAirBrake);

%% COMPATIBILITY SETTINGS !! DO NOT CHANGE UNLESS YOU KNOW WHAT YOU ARE DOING !!
settings.stoch.N = 1;
settings.wind.input = false;
settings.wind.model = false;
settings.control = 1;