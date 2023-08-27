%{

montecarlo settings 

%}

%% MONTECARLO
settings.montecarlo = false;                                                % set to true to run and save montecarlo simulation plots

% settings.stoch.OMEGA;
% settings.stoch.uncert;
% settings.stoch.Day;
% settings.stoch.Hour;

if settings.montecarlo 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% settable parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% how many simulations
N_sim = 10; % set to at least 500
simulationType_thrust = "gaussian";  % "gaussian", "exterme"
displayIter = true; % set to false if you don't want to see the iteration number (maybe if you want to run Montecarlos on hpe)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%%%%%%%%%%%%%%% untouchable parameters (unless you know really well what you are doing) %%%%%%%%%%%%%%%
%% stochastic parameters

    switch simulationType_thrust

        case "gaussian"

            sigma_t = (1.10-1)/3;             % thrust_percentage standard deviation
            mu_t = 1;                         % thrust_percentage mean value

            thrust_percentage = normrnd(mu_t,sigma_t,N_sim,1);       %generate normally distributed values ( [0.8 1.20] = 3sigma) % serve il toolbox
            stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array


            %%% in questo modo però il burning time rimane fissato perchè è
            %%% settato al'interno di simulationData.m -> possibili errori per
            %%% le simulazioni con exp_thrust aumentato se si vuole fare
            %%% spegnimento dell'ibrido
            impulse_uncertainty = normrnd(1,0.02/3,N_sim,1);
            stoch.expThrust = diag(impulse_uncertainty)*((1./thrust_percentage) * settings.motor.expTime);          % burning time - same notation as thrust here
            %%%
            for i =1:N_sim
                stoch.State.xcgTime(:,i) =  settings.State.xcgTime/settings.tb .* stoch.expThrust(i,end);  % Xcg time
            end

            %%% Aero coefficients uncertainty

            sigma_aer = (0.1)/3;             % aero coeffs error standard deviation
            mu_aer = 0;                      % aero coeffs error mean value
            aer_percentage = normrnd(mu_aer,sigma_aer,N_sim,1);

            %%% wind parameters
            settings.wind.MagMin = 0;                                               % [m/s] Minimum Wind Magnitude
            settings.wind.MagMax = 10;                                               % [m/s] Maximum Wind Magnitude
            settings.wind.ElMin  = - deg2rad(45);
            settings.wind.ElMax  = + deg2rad(45);
            settings.wind.AzMin  = - deg2rad(180);
            settings.wind.AzMax  = + deg2rad(180);

            switch settings.windModel
                case "constant"
                    [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust);
                case "multiplicative"
                    [stoch.wind.Mag,stoch.wind.Az] = windMultGeneratorMontecarlo(settings.wind,N_sim);
            end

        case "extreme"


            thrust_percentage = [0.9;1.1]; % this is overwritten in the next step, but it sets the values to retrieve in the parameter generation

            %%% wind parameters
            [stoch.wind.uw, stoch.wind.vw, stoch.wind.ww, stoch.wind.Az, stoch.wind.El ,thrust_percentage, N_sim] = windConstGeneratorMontecarlo(settings.wind,N_sim,simulationType_thrust,thrust_percentage);

            stoch.thrust = thrust_percentage*settings.motor.expThrust;                  % thrust - the notation used creates a matrix where each row is the expThrust multiplied by one coefficient in the thrust percentage array
            %%%
            stoch.expThrust = (1./thrust_percentage) * settings.motor.expTime;          % burning time - same notation as thrust here
    end


    %% save arrays
    
    % algorithms
    algorithm_vec = {'interp';'NoControl';'engine';'complete'; 'PID_2021'; 'shooting'}; % interpolation, no control, engine shutdown, engine+arb, PID change every 2s, shooting

end