%{

[control variables] parameter configuration script

%}

% Possible range of values for the control variable
switch settings.mission

    case 'Lynx_Portugal_October_2021'

        contSettings.delta_S_available = (0.0:0.001/4:0.01017)';
    
    case 'Lynx_Roccaraso_September_2021'

        contSettings.delta_S_available = (0.0:0.001/4:0.01017)';
    
    case 'Pyxis_Portugal_October_2022'
        % only for PID 
        contSettings.delta_S_available = (0.0:0.001/4:0.009564*settings.servo.maxAngle)'; 
        
        %filtering
        contSettings.filterRatio = 2;
        
        contSettings.Zfilter = 2000; % starting point from which the coefficient is diminished.
        contSettings.deltaZfilter = 250; % every deltaZfilter [m] the filter coefficient is diminished by a ratio of filterRatio
        
        contSettings.Tfilter = 12; % starting time from which the coefficient is diminished.
        contSettings.deltaTfilter = 2.5; % every deltaTfilter [s] the filter coefficient is diminished by a ratio of filterRatio
    
    case 'Pyxis_Roccaraso_September_2022'
    
        contSettings.delta_S_available = (0.0:0.001/4:0.009564*settings.servo.maxAngle)'; 
        
        % filtering
        contSettings.filterRatio = 2;
        contSettings.Zfilter = 600; % starting point from which the coefficient is diminished.
        contSettings.deltaZfilter = 100; % every deltaZfilter the filter coefficient is diminished by a ratio of filterRatio
        
        contSettings.Tfilter = 8; % starting time from which the coefficient is diminished.
        contSettings.deltaTfilter = 2; % every deltaTfilter [s] the filter coefficient is diminished by a ratio of filterRatio

end