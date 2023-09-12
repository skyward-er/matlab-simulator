%{

[control variables] parameter configuration script

%}

% Possible range of values for the control variable
switch settings.mission

    case 'Lynx_Portugal_October_2021'

        contSettings.delta_S_available = (0.0:0.001/4:0.01017)';
        contSettings.filter_coeff0 = 0.9; % starting value, then linear decrease until max Altitude
        contSettings.filterMinAltitude = 1000;
        contSettings.filterMaxAltitude = 3000;
        % set altitude at which open to max
        contSettings.criticalAltitude = 2990;
        
        settings.stopPitotAltitude = 2800;
        settings.CD_correction = 1; % set to 1 if you want to use CD from DATCOM in the simulation (and also in the accelerometer ascent), otherwise multiplies CD (only CD, not the others) for it

    case 'Lynx_Roccaraso_September_2021'

        contSettings.delta_S_available = (0.0:0.001/4:0.01017)';
    
    case 'Pyxis_Portugal_October_2022'
        % only for PID 
        contSettings.delta_S_available = (0.0:0.001/4:settings.arb.surfPol*settings.servo.maxAngle)'; 
        
        %filtering
        contSettings.filterRatio = 2;      
        contSettings.Tfilter = 12; % starting time from which the coefficient is diminished.
        contSettings.deltaTfilter = 2.5; % every deltaTfilter [s] the filter coefficient is diminished by a ratio of filterRatio
            % linear filter with altitude
        contSettings.filter_coeff0 = 0.9; % starting value, then linear decrease until max Altitude
        contSettings.filterMinAltitude = 1000;
        contSettings.filterMaxAltitude = 3000;
        % set altitude at which open to max
        contSettings.criticalAltitude = 2990;
        
        settings.stopPitotAltitude = 2800;
        settings.CD_correction = 1; % set to 1 if you want to use CD from DATCOM in the simulation (and also in the accelerometer ascent), otherwise multiplies CD (only CD, not the others) for it

    case 'Pyxis_Roccaraso_September_2022'
    
        contSettings.delta_S_available = (0.0:0.001/4:settings.arb.surfPol*settings.servo.maxAngle)'; 
        
        % filtering
        contSettings.filterRatio = 2;

        contSettings.Zfilter = 600; % starting point from which the coefficient is diminished.
        contSettings.deltaZfilter = 100; % every deltaZfilter the filter coefficient is diminished by a ratio of filterRatio
        
        contSettings.Tfilter = 8; % starting time from which the coefficient is diminished.
        contSettings.deltaTfilter = 2; % every deltaTfilter [s] the filter coefficient is diminished by a ratio of filterRatio
        
        % linear filter with altitude
        contSettings.filter_coeff0 = 0.9; % starting value, then linear decrease until max Altitude
        contSettings.filterMinAltitude = 400;
        contSettings.filterMaxAltitude = 1500;
        % set altitude at which open to max
        contSettings.criticalAltitude = 1500;
        
        settings.stopPitotAltitude = 800;
        settings.CD_correction = 1; % set to 1 if you want to use CD from DATCOM in the simulation (and also in the accelerometer ascent), otherwise multiplies CD (only CD, not the others) for it

    case 'Gemini_Portugal_October_2023' 
     
        contSettings.traj_choice = 1; % if 1 performs trajectory choice, if zero it doesn't
        N_mass = 11;     % number of references to generate
        mass_min = 26;   % [kg] min mass for trajectory choice
        mass_max = 30;   % [kg] max mass for trajectory choice
        contSettings.dmass = (mass_max-mass_min)/(N_mass-1);
        contSettings.masses_vec = mass_min:contSettings.dmass:mass_max; % masses vector for trajectory generation and choice
        

        contSettings.delta_S_available = (0.0:0.001/4:settings.arb.surfPol*settings.servo.maxAngle)'; 
        
        % filtering
        contSettings.filterRatio = 2;

        contSettings.Zfilter = 2400; % starting point from which the coefficient is diminished.
        contSettings.deltaZfilter = 300; % every deltaZfilter the filter coefficient is diminished by a ratio of filterRatio
        
        contSettings.Tfilter = 13; % starting time from which the coefficient is diminished.
        contSettings.deltaTfilter = 3; % every deltaTfilter [s] the filter coefficient is diminished by a ratio of filterRatio
        
        % linear filter with altitude
        contSettings.filter_coeff0 = 0.9; % starting value, then linear decrease until max Altitude
        contSettings.filterMinAltitude = 1000;
        contSettings.filterMaxAltitude = 3000;
        % set altitude at which open to max
        contSettings.criticalAltitude = 2990;
        settings.stopPitotAltitude = 2800;

        settings.CD_correction = 0.75; % set to 1 if you want to use CD from DATCOM in the simulation (and also in the accelerometer ascent), otherwise multiplies CD (only CD, not the others) for it

    case 'Gemini_Roccaraso_September_2023' 
     
        contSettings.traj_choice = 1; % if 1 performs trajectory choice, if zero it doesn't
        N_mass = 11;     % number of references to generate
        mass_min = 26;   % [kg] min mass for trajectory choice
        mass_max = 30;   % [kg] max mass for trajectory choice
        contSettings.dmass = (mass_max-mass_min)/(N_mass-1);
        contSettings.masses_vec = mass_min:contSettings.dmass:mass_max; % masses vector for trajectory generation and choice
        

        contSettings.delta_S_available = (0.0:0.001/4:settings.arb.surfPol*settings.servo.maxAngle)'; 
        
        % filtering
        contSettings.filterRatio = 2;

        contSettings.Zfilter = 2400; % starting point from which the coefficient is diminished.
        contSettings.deltaZfilter = 300; % every deltaZfilter the filter coefficient is diminished by a ratio of filterRatio
        
        contSettings.Tfilter = 13; % starting time from which the coefficient is diminished.
        contSettings.deltaTfilter = 3; % every deltaTfilter [s] the filter coefficient is diminished by a ratio of filterRatio
        
        % linear filter with altitude
        contSettings.filter_coeff0 = 0.9; % starting value, then linear decrease until max Altitude
        contSettings.filterMinAltitude = 600;
        contSettings.filterMaxAltitude = 1000;
        % set altitude at which open to max
        contSettings.criticalAltitude = 990;
        settings.stopPitotAltitude = 800;

        settings.CD_correction = 0.75; % set to 1 if you want to use CD from DATCOM in the simulation (and also in the accelerometer ascent), otherwise multiplies CD (only CD, not the others) for it
    
end

% quantities independent from mission
settings.CD_correction_ref = 0.75; % same, but for the trajectory generation.
