%{

config_specialConditions 

VERY IMPORTANT
use the "conf" struct to change parameters, so that you don't need to
change the confSettings.m script

%}

% 
conf.motor_Thrust_Factor = 1.; % default: = 1;
conf.motor_Time_Factor = 1; % default: 

% mach control
conf.mach_control = 0.85; % default: = rocket.airbrakes.maxMach;

% steps in advance
contSettings.N_forward = 2;
contSettings.interpType = 'linear'; % set if the interp algorithm does a linear or sinusoidal interpolation of the references
contSettings.filterRatio = 2;

settings.wind.model = false;
settings.wind.input = false;





%% changed parameters

rocket.motor.thrust = conf.motor_Thrust_Factor.* rocket.motor.thrust;
rocket.motor.time = conf.motor_Time_Factor .* rocket.motor.time;

settings.mach_control = conf.mach_control;
