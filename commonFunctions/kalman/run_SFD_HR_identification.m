function [sensorData]   =  run_SFD_HR_identification(sensorData, sensorTot, settings)

% Author: Federico Disarò 
% Skyward Experimental Rocketry | AVN Dept 
% email: federico.disaro@skywarder.eu
% Release date: 24/11/2023

%{
-----------DESCRIPTION OF FUNCTION:------------------
This function takes the data from the barometers and detects the ones which
behavoir deviates from the expected behavoir

The algorithm follows these steps:
    1) Consider faulty the sensor that have a constant value for n-times
    2) Using the previous output values, compute the trend of the next step
    3) Consider faulty the sensor which measure is not inside
    the interval ±threshold from the trend
    4) Return a vector of booleans (0 = Healty, 1 = Faulty) where each
    position represents the state of a barometer.
        (EX fault = [ 1 0 0 ] means " baro1 faulty, baro2 ok, baro3 ok ")

INPUTS: 
    - n_freeze:             [1x1] Number of times after which a sensor is considered frozen

    - n_points:             [1x1] How many points the algorithm look back to
                            calculate the pressure trend

    - threshold:            [1x1] Interval inside which the barometers are considered valid

    - freeze_counter        [1x3] Number of consecutive constant values for each baro

    - previous_sfd_values:  [( n_points+1) x 1] Previous output of the SFD_HR algorithm   

OUTPUTS:

    - freeze_counter
        
    - sensorData.sfd_hr.output
        
        - barometers        [1x3] Measures of the barometers at this interation
        
        - faults            [1x3] Healty/Faulty state of sensors

        - trend             [1x1] Predicted value for the next step

-----------------------------------------------------------------------
%}

%% Declare variables

n_freeze = settings.sfd_hr.n_freeze;

n_points = settings.sfd_hr.n_points;

threshold = settings.sfd_hr.threshold;

freeze_counter = sensorData.sfd_hr.freeze_counter;

previous_sfd_values = sensorData.sfd_hr.prev_sfd_values;

faults = [0, 0, 0];


% take data from 3 barometers

curr_values = [ sensorTot.barometer_sens{1}.pressure_measures(end), sensorTot.barometer_sens{2}.pressure_measures(end), sensorTot.barometer_sens{3}.pressure_measures(end)] ;

num_prec = sum( double( previous_sfd_values ~= 0 ) );


%% Exclude sensor which have a constant value for n_freeze times

if num_prec > 1 
   
    prec_values = [ sensorTot.barometer_sens{1}.pressure_measures(end - 1), sensorTot.barometer_sens{2}.pressure_measures(end - 1), sensorTot.barometer_sens{3}.pressure_measures(end - 1)] ;

    score = double( curr_values == prec_values );
    
    % update freeze_counter
    freeze_counter = freeze_counter .* score + score; 

    % if a sensor is frozen, swap it with 0
    if ( freeze_counter(1) >= n_freeze ) || ( freeze_counter(2) >= n_freeze ) || ( freeze_counter(3) >= n_freeze ) 
        index =  freeze_counter >= [n_freeze, n_freeze, n_freeze] ;
        curr_values( index ) = 0;
    end

end

%% Calculate trend with previous values of the SFD 

if  num_prec > n_points % if there are enough points

    trend = previous_sfd_values(end) - ( previous_sfd_values(end) - previous_sfd_values( end - n_points) ) / n_points;

else

    trend = mean(curr_values);

end

%% Consider faulty the barometers which distance form the trend is grater then the threshold

for i = 1:length(curr_values)
    if ( abs( curr_values(i) - trend ) > threshold )
        faults(i) = 1;
    end
end

%% Update

sensorData.sfd_hr.freeze_counter = freeze_counter;

sensorData.sfd_hr.output.barometers = curr_values;

sensorData.sfd_hr.output.faults = faults;

sensorData.sfd_hr.output.trend = trend;

return