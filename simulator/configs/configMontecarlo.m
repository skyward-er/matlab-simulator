%{

montecarlo settings 

%}

%% MONTECARLO
% settings.stoch.OMEGA;
% settings.stoch.uncert;
% settings.stoch.Day;
% settings.stoch.Hour;

if settings.montecarlo 

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% settable parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % how many simulations
    N_sim = 500; % set to at least 500
    simulationType_thrust = "gaussian";  % "gaussian", "exterme"
    displayIter = true; % set to false if you don't want to see the iteration number (maybe if you want to run Montecarlos on hpe)
    %note: added config. for simulating only faults in the mainMontecarlo
    %file and not here, there's a flag "settings.montecarlo_only_fault_sim" that deals with this 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% %%%%%%%%%%%%%%%%% untouchable parameters (unless you know really well what you are doing) %%%%%%%%%%%%%%%
    %% stochastic parameters
    if ~settings.montecarlo_only_fault_sim
        switch simulationType_thrust
            case "gaussian"
                
                %%% thrust uncertainty
                sigma_t = (1.10-1)/3;             % thrust_percentage standard deviation
                mu_t = 1;                         % thrust_percentage mean value
    
                thrust_percentage = normrnd(mu_t,sigma_t,N_sim,1);       %generate normally distributed values ( [0.8 1.20] = 3sigma) % serve il toolbox
                stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
    
    
                %%% in questo modo però il burning time rimane fissato perchè è
                %%% settato al'interno di simulationData.m -> possibili errori per
                %%% le simulazioni con exp_thrust aumentato se si vuole fare
                %%% spegnimento dell'ibrido
                impulse_uncertainty = normrnd(1,0.05/3,N_sim,1);
                stoch.expThrust = diag(impulse_uncertainty)*((1./thrust_percentage) * settings.motor.expTime);          % burning time - same notation as thrust here
                %%%
                for i =1:N_sim
                    stoch.State.xcgTime(:,i) =  settings.State.xcgTime/settings.tb .* stoch.expThrust(i,end);  % Xcg time
                end
    
                %%% Aero coefficients uncertainty
    
                sigma_aer = (0.1)/3;             % aero coeffs error standard deviation
                mu_aer = -0.05;                      % aero coeffs error mean value
                stoch.aer_percentage = normrnd(mu_aer,sigma_aer,N_sim,1);
    
                %%% wind parameters
                settings.wind.MagMin = 2.7;                                                % [m/s] Minimum Wind Magnitude
                settings.wind.MagMax = 3.3;                                               % [m/s] Maximum Wind Magnitude
                settings.wind.ElMin  = - deg2rad(5);
                settings.wind.ElMax  = + deg2rad(5);
                settings.wind.AzMin  = + deg2rad(170);
                settings.wind.AzMax  = + deg2rad(210);
    
                switch settings.windModel
                    case "constant"
                        [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust);
                    case "multiplicative"
                        [stoch.wind.Mag,stoch.wind.Az,stoch.wind.unc] = windMultGeneratorMontecarlo(settings.wind,N_sim);
                end
    
                %%% mass offset distribution
                sigma_m = 1/3; % 1.5 [kg] of offset (uncertainty on refueling mass)
                mu_m = 0;
                stoch.mass_offset = normrnd(mu_m,sigma_m,N_sim,1);
    
    
                % launch rail orientation
                sigma_om = 2/3 *pi/180; % 2 [deg] of offset (uncertainty on ramp elevation angle)
                mu_om = settings.OMEGA;
                stoch.OMEGA_rail = normrnd(mu_om,sigma_om,N_sim,1);
    
                % launch rail orientation
                sigma_phi = 5/3 *pi/180; % 2 [deg]of offset (uncertainty on ramp azimuth angle)
                mu_phi = settings.PHI;
                stoch.PHI_rail = normrnd(mu_phi,sigma_phi,N_sim,1);
    
            case "extreme"
    
                thrust_percentage = [0.9;1.1]; % this is overwritten in the next step, but it sets the values to retrieve in the parameter generation
    
                %%% wind parameters
                [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ,thrust_percentage, N_sim] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust,thrust_percentage);
    
                stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
                %%%
                stoch.expThrust = (1./thrust_percentage) * settings.motor.expTime;          % burning time - same notation as thrust here      
        end
    else
        switch simulationType_thrust

            case "gaussian"
                
                %%% thrust uncertainty
                thrust_percentage = 1;                         % thrust_percentage mean value
    
                stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
    
    
                %%% in questo modo però il burning time rimane fissato perchè è
                %%% settato al'interno di simulationData.m -> possibili errori per
                %%% le simulazioni con exp_thrust aumentato se si vuole fare
                %%% spegnimento dell'ibrido
                stoch.expThrust = diag(ones(N_sim,1))*(1./(thrust_percentage*ones(N_sim,1)) * settings.motor.expTime);          % burning time - same notation as thrust here
                %%%
                for i =1:N_sim
                    stoch.State.xcgTime(:,i) =  settings.State.xcgTime/settings.tb .* stoch.expThrust(i,end);  % Xcg time
                end
    
                %%% Aero coefficients uncertainty
    
                sigma_aer = (0.1)/3;             % aero coeffs error standard deviation
                mu_aer = -0.05;                      % aero coeffs error mean value
                stoch.aer_percentage = mu_aer;
    
                %%% wind parameters
                settings.wind.MagMin = 2.7;                                                % [m/s] Minimum Wind Magnitude
                settings.wind.MagMax = 3.3;                                               % [m/s] Maximum Wind Magnitude
                settings.wind.ElMin  = - deg2rad(5);
                settings.wind.ElMax  = + deg2rad(5);
                settings.wind.AzMin  = + deg2rad(170);
                settings.wind.AzMax  = + deg2rad(210);
    
                switch settings.windModel
                    case "constant"
                        [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust);
                    case "multiplicative"
                        [stoch.wind.Mag,stoch.wind.Az,stoch.wind.unc] = windMultGeneratorMontecarlo(settings.wind,N_sim);
                end
    
                %%% mass offset distribution
                sigma_m = 1/3; % 1.5 [kg] of offset (uncertainty on refueling mass)
                stoch.mass_offset = 0;
    
    
                % launch rail orientation
                stoch.OMEGA_rail = settings.OMEGA;
    
                % launch rail orientation
                stoch.PHI_rail = settings.PHI;
        end
    end
    %% save arrays
    
    % algorithms
    algorithm_vec = {'interp';'NoControl';'engine';'complete'; 'PID_2021'; 'shooting'}; % interpolation, no control, engine shutdown, engine+arb, PID change every 2s, shooting
else
    settings.mass_offset = -1;%2*(-0.5+rand(1)); % initialise to 0 the value of the mass offset, in order to not consider its uncertainty on the nominal simulations
    % mass_flow_rate = diff(settings.motor.expM([end,1]))/settings.tb; % [kg/s]
    % real_tb = (settings.mass_offset +  settings.motor.mOx)/ mass_flow_rate;
    % if real_tb < settings.tb
    %     settings.tb = real_tb;
    % end
end