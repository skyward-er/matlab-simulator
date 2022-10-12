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
conf.mach_control = settings.MachControl; % default: = settings.MachControl;




%% changed parameters

settings.motor.expThrust = conf.motor_Thrust_Factor.* settings.motor.expThrust;
settings.motor.expTime = conf.motor_Time_Factor .* settings.motor.expTime;

settings.mach_control = conf.mach_control;
