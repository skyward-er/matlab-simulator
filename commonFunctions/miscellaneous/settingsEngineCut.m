function [settings, rocket] = settingsEngineCut(settings, engineT0, rocket)
%{
    settingsEngineCut - This function computes specifc parameters at engine
                        cut event
    
    INPUTS:
            - settings, struct
    
    OUTPUTS:
            - settings, struct, contains modified settings data to consider
                                engine cutoff
    
    CALLED FUNCTIONS: -
    
    REVISIONS:
    - 0    19/10/2022, Release, Riccardo Cadamuro, Maria Teresa Cazzola
    - 1    25/02/2022, Update,  Riccardo Cadamuro
                                cut-off transient added

    Copyright © 2022, Skyward Experimental Rocketry, AFD department
    All rights reserved
    
    SPDX-License-Identifier: GPL-3.0-or-later
%}
    
    % rocket.motor.cutoffTime = rocket.motor.cutoffTime - engineT0;
    if (rocket.motor.cutoffTime) > 0 && ( rocket.motor.cutoffTime  <= (rocket.motor.time(end) - rocket.motor.cutoffTransient) )
        tEC = rocket.motor.cutoffTime;           % controlled shutoff moment, 0.3 is the delay
        tCO = rocket.motor.cutoffTransient;                     % cutoff transient duration

        for i = 1: length(rocket.motor.time)
            if rocket.motor.time(i) >= rocket.motor.cutoffTime
                
                rocket.motor.thrust(i) = rocket.motor.thrust(i) * (1 - (rocket.motor.time(i) - tEC)/tCO ); 

                if rocket.motor.time(i) > tEC + tCO || rocket.motor.thrust(i) < 0
                    rocket.motor.thrust(i) = 0; 
                end

            end
        end
        
        rocket.motor.cutoffTime = rocket.motor.cutoffTime + rocket.motor.cutoffTransient;
        rocket.updateCutoff;
        
    elseif rocket.motor.cutoffTime >= (rocket.motor.time(end) - rocket.motor.cutoffTransient)
    
        rocket.motor.cutoffTime = rocket.motor.time(end);
    
    elseif rocket.motor.cutoffTime <= 0
        error('rocket.motor.cutoffTime must be grater than zero');
    end
    
    rocket.updateCutoff;

end