function settings = settingsEngineCut(settings, engineT0)
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
    
    settings.timeEngineCut = settings.timeEngineCut - engineT0;

    if (settings.timeEngineCut) > 0 && ( settings.timeEngineCut  <= (settings.tb - settings.tCO) )
        
        tEC = settings.timeEngineCut;           % controlled shutoff moment, 0.3 is the delay
        tCO = settings.tCO;                     % cutoff transient duration

        for i = 1: length(settings.motor.expTime)
            if settings.motor.expTime(i) >= settings.timeEngineCut
                
                settings.motor.expThrust(i) = settings.motor.expThrust(i) * (1 - (settings.motor.expTime(i) - tEC)/tCO ); 

                if settings.motor.expTime(i) > tEC + tCO || settings.motor.expThrust(i) < 0
                    settings.motor.expThrust(i) = 0; 
                end

            end
        end
        
        settings.timeEngineCut = settings.timeEngineCut + settings.tCO;

        settings.expMengineCut = interpLinear(settings.motor.expTime, settings.motor.expM, settings.timeEngineCut);
        settings.IengineCut(1) = interpLinear(settings.motor.expTime,  settings.I(1,:), settings.timeEngineCut);
        settings.IengineCut(2) = interpLinear(settings.motor.expTime,  settings.I(2,:), settings.timeEngineCut);
        settings.IengineCut(3) = interpLinear(settings.motor.expTime,  settings.I(3,:), settings.timeEngineCut);

    elseif settings.timeEngineCut >= (settings.tb - settings.tCO)
    
        settings.timeEngineCut = inf;
    
    elseif settings.timeEngineCut <= 0
        error('settings.timeEngineCut must be grater than zero');
    end

    settings.timeEngineCut = settings.timeEngineCut + engineT0;
end