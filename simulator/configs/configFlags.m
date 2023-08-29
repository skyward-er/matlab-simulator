%{

flag configuration script


possible scenarios: "free ascent", "controlled ascent" , "descent",
"ballistic", "full flight"

scenarios explanation:

- "free ascent"       : only up to apogee, no control, no kalman, no ada, no
                        nothing.

- "controlled ascent" : only up to apogee, with airbrakes, with kalman,
                        with ada, with everything.

- "descent"           : to investigate how to do it, we should be able to
                        simulate only from apogee to ground.

- "ballistic"         : all flight, no parachutes, active ascent control (airbrakes or engine shutdown). So ballistic: ground
                        apogee ground.

- "full flight"       : all flight, including parachutes and active control
                        (airbrakes or engine shutdown).

%}


% scenario configuration
conf.scenario = "full flight";
conf.board = "main";            % Either "main" or "payload"
conf.HIL = true;

% WIP flags
settings.machControlActive  = false; % Switch on the mach control in ascentControl.m
settings.HRE                = false; % Switch on if the rocket is mounting a Hybrid Engine, which allows the possibility to shut down the engine

% std_run integration flags
settings.nmax = 100000; % max iterations of the while cycle
settings.flagStopIntegration     =   true;                                           % while this is true the integration runs
settings.flagAscent              =   false;                                          % while this is false...
settings.flagMatr                =   false(settings.nmax, 6);                                 % while this value are false...
settings.lastLaunchFlag = true; % LEAVE THIS TO TRUE UNLESS YOU KNOW WHAT YOU ARE DOING (other wise it won't stop if you set only ascent simulation)

% ALGORITHM TUNING
settings.tuning = true;                 % [-] True if you want to tune the algorithm (resets the random seed)

% Identification
settings.identification = false;
% EXPORT GRAPHICS (from simulations)
settings.flagExportPLOTS = false;

% export csv files for CPP implementation?
settings.flagExportCSV = false;



%% ------------------------------- don't modify unless you really know what you are doing, touch only the flags before this line ---------------------------- %%
if not(conf.HIL) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
    switch conf.scenario
     
        case "free ascent" 

            settings.launchWindow       = false;  % Switch off this to avoid pausing the launch till you press the launch button
            settings.electronics        = false;  % Switch on when testing with Hardware in the loop HIL - NOT IMPLEMENTED YET, STILL TO BE MERGED
            settings.ascentOnly         = true;   % Switch on to simulate only the ascent phase untill the apogee
            settings.ballisticFligth    = true;   % Switch on to simulate the balistic fligth without any parachute
            settings.control            = false;  % Switch on to simulate the control
            settings.dataNoise          = false;  % Switch on to simulate the data acquisiton from sensors
            settings.flagNAS            = false;  % Switch on to run the kalman algorithm - note, also to run the airbrakes control algorithm this is needed true
            settings.flagADA            = false;  % Switch on to run the apogee detection algorithm
        
        case "controlled ascent"

            settings.launchWindow       = false;  % Switch off this to avoid pausing the launch till you press the launch button
            settings.electronics        = false;  % Switch on when testing with Hardware in the loop HIL - NOT IMPLEMENTED YET, STILL TO BE MERGED
            settings.ascentOnly         = true;   % Switch on to simulate only the ascent phase untill the apogee
            settings.ballisticFligth    = true;   % Switch on to simulate the balistic fligth without any parachute
            settings.control            = true;   % Switch on to simulate the control
            settings.dataNoise          = true;   % Switch on to simulate the data acquisiton from sensors
            settings.flagNAS            = true;   % Switch on to run the kalman algorithm - note, also to run the airbrakes control algorithm this is needed true
            settings.flagADA            = true;   % Switch on to run the apogee detection algorithm

        case "descent"     

            settings.launchWindow       = false;  % Switch off this to avoid pausing the launch till you press the launch button
            settings.electronics        = false;  % Switch on when testing with Hardware in the loop HIL - NOT IMPLEMENTED YET, STILL TO BE MERGED
            settings.ascentOnly         = false;   % Switch on to simulate only the ascent phase untill the apogee
            settings.ballisticFligth    = false;   % Switch on to simulate the balistic fligth without any parachute
            settings.control            = false;  % Switch on to simulate the control
            settings.dataNoise          = true;   % Switch on to simulate the data acquisiton from sensors
            settings.flagNAS            = true;   % Switch on to run the kalman algorithm - note, also to run the airbrakes control algorithm this is needed true
            settings.flagADA            = true;   % Switch on to run the apogee detection algorithm

        case "ballistic"

            settings.launchWindow       = false;  % Switch off this to avoid pausing the launch till you press the launch button
            settings.electronics        = false;  % Switch on when testing with Hardware in the loop HIL - NOT IMPLEMENTED YET, STILL TO BE MERGED
            settings.ascentOnly         = false;   % Switch on to simulate only the ascent phase untill the apogee
            settings.ballisticFligth    = true;   % Switch on to simulate the balistic fligth without any parachute
            settings.control            = true;  % Switch on to simulate the control
            settings.dataNoise          = true;   % Switch on to simulate the data acquisiton from sensors
            settings.flagNAS            = true;   % Switch on to run the kalman algorithm - note, also to run the airbrakes control algorithm this is needed true
            settings.flagADA            = true;   % Switch on to run the apogee detection algorithm

        case "full flight"

            settings.launchWindow       = false;  % Switch off this to avoid pausing the launch till you press the launch button
            settings.electronics        = false;  % Switch on when testing with Hardware in the loop HIL - NOT IMPLEMENTED YET, STILL TO BE MERGED
            settings.ascentOnly         = false;   % Switch on to simulate only the ascent phase untill the apogee
            settings.ballisticFligth    = false;   % Switch on to simulate the balistic fligth without any parachute
            settings.control            = true;   % Switch on to simulate the control
            settings.dataNoise          = true;   % Switch on to simulate the data acquisiton from sensors
            settings.flagNAS            = true;   % Switch on to run the kalman algorithm - note, also to run the airbrakes control algorithm this is needed true
            settings.flagADA            = true;   % Switch on to run the apogee detection algorithm

    end

elseif conf.HIL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    settings.launchWindow       = true;           % Switch off this to avoid pausing the launch till you press the launch button
    settings.electronics        = true;           % Switch on when testing with Hardware in the loop HIL - NOT IMPLEMENTED YET, STILL TO BE MERGED
    settings.flagNAS            = false;          % Switch on to run the kalman algorithm - note, also to run the airbrakes control algorithm this is needed true
    settings.flagADA            = false;          % Switch on to run the apogee detection algorithm
    
    switch conf.scenario
     
        case "free ascent" 
            
            error('free ascent and Hardware In the Loop are not compatible scenarios')
            
        case "controlled ascent"

            settings.ascentOnly         = true;   % Switch on to simulate only the ascent phase untill the apogee
            settings.ballisticFligth    = true;   % Switch on to simulate the balistic fligth without any parachute
            settings.control            = true;   % Switch on to simulate the control
            settings.dataNoise          = true;   % Switch on to simulate the data acquisiton from sensors
            
        case "descent" % WIP    

            settings.ascentOnly         = true;   % Switch on to simulate only the ascent phase untill the apogee
            settings.ballisticFligth    = true;   % Switch on to simulate the balistic fligth without any parachute
            settings.control            = false;  % Switch on to simulate the control
            settings.dataNoise          = true;   % Switch on to simulate the data acquisiton from sensors
            
        case "ballistic"

            settings.ascentOnly         = true;   % Switch on to simulate only the ascent phase untill the apogee
            settings.ballisticFligth    = true;   % Switch on to simulate the balistic fligth without any parachute
            settings.control            = true;   % Switch on to simulate the control
            settings.dataNoise          = true;   % Switch on to simulate the data acquisiton from sensors
            

        case "full flight"

            settings.ascentOnly         = false;   % Switch on to simulate only the ascent phase untill the apogee
            settings.ballisticFligth    = false;   % Switch on to simulate the balistic fligth without any parachute
            settings.control            = true;   % Switch on to simulate the control
            settings.dataNoise          = true;   % Switch on to simulate the data acquisiton from sensors
            
    end

end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch conf.board
    case "main"
        settings.parafoil = false;
    case "payload"
        settings.parafoil = true;
end

if not(settings.ballisticFligth) && settings.ascentOnly
    error('To simulate a landing with the parachutes, settings.ascentOnly must be false')
end
%% %% uncomment when the MSA toolkit is updated

% if settings.mission == 'NewRocket_2023' 
%     settings.HRE = true;
% else
%     settings.HRE = false;
% end