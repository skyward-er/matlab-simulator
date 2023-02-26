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
conf.mach_control = 0.85; % default: = settings.MachControl;

% steps in advance
contSettings.N_forward = 2;
contSettings.interpType = 'linear'; % set if the interp algorithm does a linear or sinusoidal interpolation of the references
contSettings.filterRatio = 2;

% settings.wind.model = false;
% settings.wind.input = false;





%% changed parameters

settings.motor.expThrust = conf.motor_Thrust_Factor.* settings.motor.expThrust;
settings.motor.expTime = conf.motor_Time_Factor .* settings.motor.expTime;

settings.mach_control = conf.mach_control;
