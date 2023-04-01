%{

wind configuration script



NOTE: wind azimuth angle indications (wind directed towards):
    - 360 deg (use 360 instead of 0)-> North
    - 90 deg                        -> East
    - 180 deg                       -> South
    - 270 deg                       -> West
%}


% select which model you want to use:
settings.windModel = "constant"; % choose between: "constant" "multiplicative" "atmospheric"



%% ---------------------------------- settings from here ------------------------------------ %%

if conf.script == "simulator"

    switch settings.windModel
    
        case "atmospheric"
           
            settings.wind.input = false;
            settings.wind.model = true;
            % matlab hswm model, wind model on altitude based on historical data
            
            % input Day and Hour as arrays to run stochastic simulations
            settings.wind.DayMin = 105;                    % [d] Minimum Day of the launch
            settings.wind.DayMax = 105;                    % [d] Maximum Day of the launch
            settings.wind.HourMin = 4;                     % [h] Minimum Hour of the day
            settings.wind.HourMax = 4;                     % [h] Maximum Hour of the day
            settings.wind.ww = 0;                          % [m/s] Vertical wind speed
            
            
    
        case "multiplicative"
    
            % Wind is generated for every altitude interpolating with the coefficient defined below
            settings.wind.input = true;
            settings.wind.model = false;
            settings.wind.inputGround  = 9;                                         % [m/s] Wind magnitude at the ground
            settings.wind.inputAlt     = 4000/1100*[0 50 100 200 350 500 700 900 1100];       % [m] Altitude vector
            settings.wind.inputMult    = [0 1 2 3 4 4.5 5 5.5 6]*50;                 % [-] Percentage of increasing magnitude at each altitude
            settings.wind.inputAzimut  = 200*pi/180*ones(1,9);                       % [deg] Wind azimut angle at each altitude (toward wind incoming direction)
            settings.wind.input_uncertainty = [0,0];
            % settings.wind.input_uncertainty = [a,b];      wind uncertanties:
            % - a, wind magnitude percentage uncertanty: magn = magn *(1 +- a)
            % - b, wind direction band uncertanty: dir = dir 1 +- b
            
        case "constant"
    
            % Wind is generated randomly from the minimum to the maximum parameters which defines the wind.
            % Setting the same values for min and max will fix the parameters of the wind.
            settings.wind.input = false;
            settings.wind.model = false;

            settings.wind.MagMin    =   0;                        % [m/s] Minimum Magnitude
            settings.wind.MagMax    =   0;                        % [m/s] Maximum Magnitude
            settings.wind.ElMin     =   0*pi/180;                 % [rad] Minimum Elevation, user input in degrees (ex. 0)
            settings.wind.ElMax     =   0*pi/180;                 % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
            settings.wind.AzMin     =  (180)*pi/180;              % [rad] Minimum Azimuth, user input in degrees (ex. 90)
            settings.wind.AzMax     =  (180)*pi/180;              % [rad] Maximum Azimuth, user input in degrees (ex. 90)
               
                 
    end

elseif conf.script == "trajectory generation"
    
            settings.wind.MagMin = 0;                         % [m/s] Minimum Magnitude
            settings.wind.MagMax = 0;                         % [m/s] Maximum Magnitude
            settings.wind.ElMin = 0*pi/180;                   % [rad] Minimum Elevation, user input in degrees (ex. 0)
            settings.wind.ElMax = 0*pi/180;                   % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
            settings.wind.AzMin = (360)*pi/180;               % [rad] Minimum Azimuth, user input in degrees (ex. 90)
            settings.wind.AzMax = (360)*pi/180;               % [rad] Maximum Azimuth, user input in degrees (ex. 90)
            
            settings.wind.model = false;
            settings.wind.input = false;
end