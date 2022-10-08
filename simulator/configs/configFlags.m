%{

flag configuration

%}

% FLAGS:
settings.launchWindow       = false;  % Switch off this to avoid pausing the launch till you press the launch button
settings.electronics        = false;  % Switch on when testing with Hardware in the loop HIL - NOT IMPLEMENTED YET, STILL TO BE MERGED
settings.ascentOnly         = true;  % Switch on to simulate only the ascent phase untill the apogee
settings.ballisticFligth    = true;  % Switch on to simulate the balistic fligth without any parachute
settings.control            = true;  % Switch on to simulate the control
settings.dataNoise          = true;  % Switch on to simulate the data acquisiton from sensors
settings.Kalman             = true;  % Switch on to run the kalman algorithm - note, also to run the airbrakes control algorithm this is needed true
settings.Ada                = true;  % Switch on to run the apogee detection algorithm
settings.HRE                = false; % Switch on if the rocket is mounting a Hybrid Engine, which allows the possibility to shut down the engine
settings.machControlActive  = false; % Switch on the mach control in ascentControl.m

% compatibility checks - do not change unless you really know what you are
% doing
if settings.electronics
    settings.launchWindow = true;
    settings.Kalman       = false;
    settings.Ada          = false;
    settings.control      = true;


    % add path for Hardware In the Loop
    addpath('../hardware_in_the_loop/');
    addpath('../hardware_in_the_loop/serialbridge');
    run('HILconfig.m');
    serialbridge("Open", hil_settings.serial_port, hil_settings.baudrate); % Initialization of the serial port
end

%% ALGORITHM TUNING
settings.tuning = true;                 % [-] True if you want to tune the algorithm

%%% uncomment when the MSA toolkit is updated

% if settings.mission == 'NewRocket_2023' 
%     settings.HRE = true;
% else
%     settings.HRE = false;
% end