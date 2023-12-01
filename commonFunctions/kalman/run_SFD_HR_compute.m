function [sensorData, sensorTot]  =  run_SFD_HR_compute(sensorData, sensorTot)

% Author: Federico Disarò 
% Skyward Experimental Rocketry | AVN Dept 
% email: federico.disaro@skywarder.eu
% Release date: 24/11/2023

%{

-----------DESCRIPTION OF FUNCTION:------------------
This function takes as input the struct sensorData.sfd_hr and  compute a 
pressure value to pass to the ADA

The algorithm works with this logic:

    - If there are 0 or 1 faults: compute a mean between the valid sensors and the trend

    - If there are more then 2 faults: find the barometers which measure is
      closer to the trend and follow it keeping a constant distance


INPUTS: 

    - ind:              [1x1] By default is -1. If the algorithm is following a sensor it's
                              the index of the sensor
    
    - distance          [1x1] By default is 0. If the algorithm is following a sensor it's
                              the distance that the algorithm keeps form the sensor

    - sensorData.sfd_hr.output

        - barometers        [1x3] Measures of the barometers at this interation
        
        - faults            [1x3] Healty/Faulty state of sensors

        - trend             [1x1] Predicted value for the next step

OUTPUTS:

    - ind

    - distance

    - pressure          [1x1] Mean pressure value for the ADA 


-----------------------------------------------------------------------
%}

%% Declare variables

ind = sensorData.sfd_hr.ind;

distance = sensorData.sfd_hr.distance;

faults = sensorData.sfd_hr.output.faults;

barometes = sensorData.sfd_hr.output.barometers;

follow_ind = sensorTot.sfd_hr.follow_ind;

trend = sensorData.sfd_hr.output.trend;

log_pres = sensorTot.sfd_hr.pressure;

%% Compute pressure 

if sum( faults ) > 1 || ind ~= -1

    if distance == 0
        % take the distance from the closest / only valid barometer
        o = barometes ;

        for i = 1:(length(barometes) - 1)
            for j = 1:( length(barometes) - i)
                if abs(o(j+i) - trend ) < abs(o(i) - trend)
                    temp = o(i);
                    o(i) = o(j+i);
                    o(j+i) = temp;
                end
            end
        end
    
        distance = o(1) - trend;
        ind = find( barometes == o(1) ); 
        follow_ind = length( log_pres );
    
    end

    % keep a sort of 'constant distance' form the valid sensor
    pressure = ( barometes(ind) - distance + trend )/2;

else

    % compute a mean between the valid barometes and the trend
    pressure = ( barometes * ( 1 - faults )' + trend ) / ( 4 - sum( faults )  );
    
end

%% Update previous SFD values

sensorData.sfd_hr.ind = ind;

sensorData.sfd_hr.distance = distance;

sensorTot.sfd_hr.follow_ind = follow_ind;

sensorData.sfd_hr.prev_sfd_values = [ sensorData.sfd_hr.prev_sfd_values(2:end); pressure ];

sensorTot.sfd_hr.pressure = vertcat(log_pres, pressure);



