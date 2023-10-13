%{

wind configuration script



NOTE: wind azimuth angle indications (wind directed towards):
    - 360 deg (use 360 instead of 0)-> North
    - 90 deg                        -> East
    - 180 deg                       -> South
    - 270 deg                       -> West
%}


% select which model you want to use:
settings.windModel = "multiplicative"; % choose between: "constant" "multiplicative" "atmospheric"



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
    
            settings.wind.input = true;
            windAltInput = [0 100 200 300 400 600 700 1000 1500 1700 1900 2000 2300 2600 2700 3000 3200 3900 4500];
            windMagInput = [1.4 4 5 5 7 9 10 11 12 13 14 15 16 17 18 19 18 18 17];
            windAzInput  = [220 190 190 190 200 210 210 210 220 220 220 230 230 240 240 240 240 240 240];
    % Wind is generated for every altitude interpolating with the coefficient defined below
        settings.wind.inputGround = 3;                                                                    % wind magnitude at the ground [m/s]
        settings.wind.inputAlt       = windAltInput;                                                 % altitude vector [m]
        settings.wind.inputMagnitude = windMagInput;
        settings.wind.inputAzimut    = windAzInput;
        settings.wind.inputMult = (settings.wind.inputMagnitude./settings.wind.inputGround - 1) * 100;
        settings.wind.input_uncertainty = [10, 10];
            % settings.wind.input_uncertainty = [a,b];      wind uncertanties:
            % - a, wind magnitude percentage uncertanty: magn = magn *(1 +- a)
            % - b, wind direction band uncertanty: dir = dir 1 +- b
            settings.wind.input = true;
            settings.wind.model = false;

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